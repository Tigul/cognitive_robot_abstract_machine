"""
Starting a localization from the command line, as
``semantic-digital-twin-localization``.

The process fetches the world from a running
:class:`~semantic_digital_twin.adapters.ros.world_fetcher.FetchWorldServer`, follows its
changes, and localizes the robot it holds.
"""

from __future__ import annotations

import argparse
import threading
from dataclasses import dataclass
from enum import StrEnum

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from semantic_digital_twin.adapters.ros.exceptions import (
    RobotToLocalizeNotFound,
    RobotWithoutBaseLidar,
)
from semantic_digital_twin.adapters.ros.localization import (
    CoordinateFrame,
    LocalizationNode,
)
from semantic_digital_twin.adapters.ros.world_fetcher import fetch_world_from_service
from semantic_digital_twin.adapters.ros.world_synchronizer import WorldSynchronizer
from semantic_digital_twin.robots.robot_part_mixins import HasLidar, HasMobileBase
from semantic_digital_twin.robots.robot_parts import AbstractRobot
from semantic_digital_twin.spatial_types.spatial_types import (
    HomogeneousTransformationMatrix,
    Pose2D,
)
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.geometry import PlanarBoundingBox
from semantic_digital_twin.world_description.shape_collection import (
    BoundingBoxCollection,
)

# %% the running process


@dataclass
class LocalizationProcess:
    """
    A localization running on its own copy of a world that follows the changes of the
    world it was fetched from.
    """

    world: World
    """
    The copy of the world the robot is localized in.
    """

    world_synchronizer: WorldSynchronizer
    """
    Applies the changes of the world the copy was fetched from.
    """

    localization_node: LocalizationNode
    """
    Localizes the robot in :attr:`world`.
    """

    def close(self) -> None:
        """
        Stops localizing and following the changes of the fetched world.
        """
        self.localization_node.close()
        self.world_synchronizer.close()


# %% command line


class LocalizationCommandLineOption(StrEnum):
    """
    The options of ``semantic-digital-twin-localization``.
    """

    ROBOT = "--robot"
    """
    Sets :attr:`LocalizationCommandLine.robot_name`.
    """

    SEARCH_WHOLE_REGION = "--search-whole-region"
    """
    Sets :attr:`LocalizationCommandLine.search_whole_region`.
    """

    FETCH_TIMEOUT = "--fetch-timeout"
    """
    Sets :attr:`LocalizationCommandLine.fetch_timeout`.
    """

    MAP_FRAME = "--map-frame"
    """
    Sets :attr:`LocalizationCommandLine.map_frame`.
    """

    ODOM_FRAME = "--odom-frame"
    """
    Sets :attr:`LocalizationCommandLine.odom_frame`.
    """

    BASE_FRAME = "--base-frame"
    """
    Sets :attr:`LocalizationCommandLine.base_frame`.
    """


class LocalizationNodeName(StrEnum):
    """
    The names of the nodes a localization process creates.
    """

    LOCALIZATION = "semantic_digital_twin_localization"
    """
    The node the localization communicates over.
    """


@dataclass
class LocalizationCommandLine:
    """
    The settings of a localization started from the command line.
    """

    robot_name: str | None = None
    """
    The name of the robot to localize, or None if the world holds exactly one robot.
    """

    search_whole_region: bool = False
    """
    Whether to search for the robot in the whole environment instead of starting where
    the fetched world places it.
    """

    fetch_timeout: float = 10.0
    """
    How long to wait for a world to fetch, in seconds.
    """

    map_frame: str = CoordinateFrame.MAP
    """
    The tf frame of the world root.
    """

    odom_frame: str = CoordinateFrame.ODOM
    """
    The tf frame the robot's odometry is measured in.
    """

    base_frame: str = CoordinateFrame.BASE_LINK
    """
    The tf frame of the robot's base.
    """

    @classmethod
    def run(cls) -> None:
        """
        Read the settings from the command line and localize until the process is
        stopped.
        """
        defaults = cls()
        parser = argparse.ArgumentParser(description=LocalizationNode.__doc__)
        parser.add_argument(LocalizationCommandLineOption.ROBOT, default=None)
        parser.add_argument(
            LocalizationCommandLineOption.SEARCH_WHOLE_REGION, action="store_true"
        )
        parser.add_argument(
            LocalizationCommandLineOption.FETCH_TIMEOUT,
            type=float,
            default=defaults.fetch_timeout,
        )
        parser.add_argument(
            LocalizationCommandLineOption.MAP_FRAME, default=defaults.map_frame
        )
        parser.add_argument(
            LocalizationCommandLineOption.ODOM_FRAME, default=defaults.odom_frame
        )
        parser.add_argument(
            LocalizationCommandLineOption.BASE_FRAME, default=defaults.base_frame
        )
        arguments = parser.parse_args()
        settings = cls(
            robot_name=arguments.robot,
            search_whole_region=arguments.search_whole_region,
            fetch_timeout=arguments.fetch_timeout,
            map_frame=arguments.map_frame,
            odom_frame=arguments.odom_frame,
            base_frame=arguments.base_frame,
        )
        rclpy.init()
        node = rclpy.create_node(LocalizationNodeName.LOCALIZATION)
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        spinning = threading.Thread(target=executor.spin, daemon=True)
        spinning.start()
        settings.start(node)
        spinning.join()

    def start(self, node: Node) -> LocalizationProcess:
        """
        Fetches a world from a
        :class:`~semantic_digital_twin.adapters.ros.world_fetcher.FetchWorldServer`,
        follows its changes and starts localizing its robot.

        :param node: The node to communicate over, spun by the caller.
        :return: The started localization.
        :raises RobotToLocalizeNotFound: If the world holds no robot of
            :attr:`robot_name`, or not exactly one robot when no name is given.
        :raises RobotWithoutBaseLidar: If the robot has no mobile base carrying a lidar.
        """
        world = fetch_world_from_service(node, timeout_seconds=self.fetch_timeout)
        world_synchronizer = WorldSynchronizer(node=node, _world=world)
        robot = self._robot_in(world)
        if not isinstance(robot, HasMobileBase) or not isinstance(
            robot.mobile_base, HasLidar
        ):
            raise RobotWithoutBaseLidar(robot=robot)
        return LocalizationProcess(
            world=world,
            world_synchronizer=world_synchronizer,
            localization_node=LocalizationNode(
                node=node,
                drive=robot.drive,
                lidar=robot.mobile_base.lidar,
                region=self._environment_region(world, robot),
                map_frame=self.map_frame,
                odom_frame=self.odom_frame,
                base_frame=self.base_frame,
                start_pose=(
                    None
                    if self.search_whole_region
                    else Pose2D.from_pose(
                        world.compute_forward_kinematics(
                            world.root, robot.drive.child
                        ).pose
                    )
                ),
            ),
        )

    def _robot_in(self, world: World) -> AbstractRobot:
        """
        :param world: The fetched world.
        :return: The robot of :attr:`robot_name`, or the world's only robot.
        :raises RobotToLocalizeNotFound: If there is no such robot.
        """
        robots = world.get_semantic_annotations_by_type(AbstractRobot)
        matching = [
            robot
            for robot in robots
            if self.robot_name is None or str(robot.name) == self.robot_name
        ]
        if len(matching) != 1:
            raise RobotToLocalizeNotFound(
                robot_names=[str(robot.name) for robot in robots],
                requested_name=self.robot_name,
            )
        return matching[0]

    @staticmethod
    def _environment_region(world: World, robot: AbstractRobot) -> PlanarBoundingBox:
        """
        :param world: The fetched world.
        :param robot: The robot to localize.
        :return: The part of the world root's x-y plane the collision geometry of
            everything but the robot spans.
        """
        robot_bodies = set(robot.bodies)
        bounds = (
            BoundingBoxCollection.merge_all(
                [
                    body.collision.as_bounding_box_collection_in_frame(world.root)
                    for body in world.bodies_with_collision
                    if body not in robot_bodies
                ],
                world.root,
            )
            .bounding_box()
            .to_array_bounds()
        )
        return PlanarBoundingBox.from_array_bounds(
            bounds.lower[:2],
            bounds.upper[:2],
            HomogeneousTransformationMatrix(reference_frame=world.root),
        )
