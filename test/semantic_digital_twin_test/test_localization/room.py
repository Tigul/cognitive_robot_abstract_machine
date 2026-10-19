from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
import numpy.typing as npt
from typing_extensions import Self

from krrood.ormatic.utils import classproperty
from semantic_digital_twin.adapters.sensors.lidar import Lidar, LidarSource
from semantic_digital_twin.datastructures.lidar_reading import LidarReading
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.datastructures.scan_pattern import ScanPattern
from semantic_digital_twin.robots.input_source import RobotTopic
from semantic_digital_twin.spatial_types.spatial_types import (
    HomogeneousTransformationMatrix,
    Pose2D,
)
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import (
    Connection6DoF,
    FixedConnection,
)
from semantic_digital_twin.world_description.geometry import (
    Box,
    PlanarBoundingBox,
    Scale,
)
from semantic_digital_twin.world_description.shape_collection import ShapeCollection
from semantic_digital_twin.world_description.world_entity import (
    Body,
    KinematicStructureEntity,
)

# %% the room


@dataclass
class StandingBox:
    """
    A box standing upright on the floor of a room.
    """

    center_x: float
    """
    The x coordinate of the box's center, in meters.
    """

    center_y: float
    """
    The y coordinate of the box's center, in meters.
    """

    length: float
    """
    The extent of the box along the x axis, in meters.
    """

    width: float
    """
    The extent of the box along the y axis, in meters.
    """


@dataclass
class Room:
    """
    A walled rectangular room whose floor spans the origin's first quadrant, holding
    obstacles placed so that no rotation or mirroring maps the room onto itself.
    """

    length: float = 6.0
    """
    Inner extent of the room along the x axis, in meters.
    """

    width: float = 4.0
    """
    Inner extent of the room along the y axis, in meters.
    """

    wall_thickness: float = 0.2
    """
    Thickness of every wall, in meters.
    """

    wall_height: float = 2.0
    """
    Height of the walls and obstacles, in meters.
    """

    obstacles: list[StandingBox] = field(
        default_factory=lambda: [
            StandingBox(center_x=1.0, center_y=0.75, length=0.5, width=1.5),
            StandingBox(center_x=4.8, center_y=0.6, length=0.8, width=0.8),
        ]
    )
    """
    The boxes standing inside the walls.
    """

    @property
    def floor(self) -> PlanarBoundingBox:
        """
        :return: The floor inside the walls.
        """
        return PlanarBoundingBox(
            min_x=0.0,
            min_y=0.0,
            max_x=self.length,
            max_y=self.width,
            origin=HomogeneousTransformationMatrix(),
        )

    def build(self) -> World:
        """
        :return: A world holding the walls and obstacles of this room.
        """
        world = World.create_with_root_body("map")
        with world.modify_world():
            for index, standing_box in enumerate(self._walls() + self.obstacles):
                box = Body(
                    name=PrefixedName(f"box_{index}"),
                    collision=ShapeCollection(
                        [
                            Box(
                                scale=Scale(
                                    standing_box.length,
                                    standing_box.width,
                                    self.wall_height,
                                )
                            )
                        ]
                    ),
                )
                world.add_body(box)
                world.add_connection(
                    FixedConnection(
                        parent=world.root,
                        child=box,
                        parent_T_connection_expression=HomogeneousTransformationMatrix.from_xyz_rpy(
                            x=standing_box.center_x,
                            y=standing_box.center_y,
                            z=self.wall_height / 2,
                        ),
                    )
                )
        return world

    def _walls(self) -> list[StandingBox]:
        """
        :return: The four walls enclosing the floor.
        """
        half_thickness = self.wall_thickness / 2
        outer_length = self.length + 2 * self.wall_thickness
        outer_width = self.width + 2 * self.wall_thickness
        return [
            StandingBox(
                self.length / 2, -half_thickness, outer_length, self.wall_thickness
            ),
            StandingBox(
                self.length / 2,
                self.width + half_thickness,
                outer_length,
                self.wall_thickness,
            ),
            StandingBox(
                -half_thickness, self.width / 2, self.wall_thickness, outer_width
            ),
            StandingBox(
                self.length + half_thickness,
                self.width / 2,
                self.wall_thickness,
                outer_width,
            ),
        ]


# %% the robot driving through it


@dataclass(eq=False)
class FullCircleLidar(Lidar):
    """
    A lidar sweeping a full circle in one degree steps.
    """

    @classproperty
    def topic_name(cls) -> str:
        """
        The topic the sweeps of this lidar are published on.
        """
        return RobotTopic.SCAN

    @classmethod
    def with_source(
        cls, robot_root: KinematicStructureEntity, source: LidarSource
    ) -> Self:
        return cls(
            root=robot_root,
            scan_pattern=ScanPattern(
                minimum_angle=-np.pi,
                maximum_angle=np.pi - np.radians(1),
                angle_increment=np.radians(1),
                minimum_range=0.05,
                maximum_range=10.0,
            ),
            source=source,
        )


@dataclass
class SimulatedRobot:
    """
    A robot base carrying a lidar through a world, whose odometry drifts from where the
    base really drives.
    """

    world: World
    """
    The world the robot drives through.
    """

    start_pose: Pose2D
    """
    Where the base starts, in the world root's frame.
    """

    random_generator: np.random.Generator
    """
    The source of the odometry and range noise.
    """

    base_T_lidar: HomogeneousTransformationMatrix = field(
        default_factory=lambda: HomogeneousTransformationMatrix.from_xyz_rpy(
            x=0.2, z=0.3
        )
    )
    """
    Where the lidar is mounted on the base.
    """

    odometry_translation_noise: float = 0.05
    """
    Standard deviation of the odometry's translation error, relative to the distance
    driven.
    """

    odometry_rotation_noise: float = 0.02
    """
    Standard deviation of the odometry's rotation error per drive, in radians.
    """

    range_noise: float = 0.02
    """
    Standard deviation of every measured range, in meters.
    """

    lidar: FullCircleLidar = field(init=False)
    """
    The lidar the base carries.
    """

    _odom_T_base: npt.NDArray[np.float64] = field(init=False)
    """
    The pose of the base its odometry reports.
    """

    _world_C_base: Connection6DoF = field(init=False)
    """
    The connection placing the base where it really is.
    """

    def __post_init__(self):
        base = Body(name=PrefixedName("base"))
        lidar_body = Body(name=PrefixedName("lidar"))
        with self.world.modify_world():
            self._world_C_base = Connection6DoF.create_with_dofs(
                world=self.world, parent=self.world.root, child=base
            )
            self.world.add_connection(self._world_C_base)
            self.world.add_connection(
                FixedConnection(
                    parent=base,
                    child=lidar_body,
                    parent_T_connection_expression=self.base_T_lidar,
                )
            )
        self.place(self.start_pose)
        self.lidar = FullCircleLidar.with_simulated_source(lidar_body)
        self._odom_T_base = np.eye(4)

    @property
    def pose(self) -> Pose2D:
        """
        :return: Where the base really is, in the world root's frame.
        """
        return Pose2D.from_pose(self._world_C_base.origin.pose)

    def place(self, pose: Pose2D) -> None:
        """
        Puts the base somewhere without its odometry noticing.

        :param pose: The new pose of the base in the world root's frame.
        """
        self._world_C_base.origin = HomogeneousTransformationMatrix(
            pose.homogeneous_matrix.to_np(),
            reference_frame=self.world.root,
            child_frame=self._world_C_base.child,
        )

    def drive(self, forward: float, rotation: float) -> None:
        """
        Drives the base along an arc and lets its odometry measure that with noise.

        :param forward: How far the base drives along its heading, in meters.
        :param rotation: How far the base turns while driving, in radians.
        """
        x, y, yaw = self.pose.to_np()
        self.place(
            Pose2D(
                x=x + forward * np.cos(yaw + rotation / 2),
                y=y + forward * np.sin(yaw + rotation / 2),
                yaw=yaw + rotation,
            )
        )
        measured_forward = forward * (
            1 + self.random_generator.normal(0, self.odometry_translation_noise)
        )
        measured_rotation = rotation + self.random_generator.normal(
            0, self.odometry_rotation_noise
        )
        self._odom_T_base = (
            self._odom_T_base
            @ Pose2D(
                x=measured_forward * np.cos(measured_rotation / 2),
                y=measured_forward * np.sin(measured_rotation / 2),
                yaw=measured_rotation,
            ).homogeneous_matrix.to_np()
        )

    def read_scan(self) -> LidarReading:
        """
        :return: A sweep of the lidar with noisy ranges.
        """
        reading = self.lidar.get_lidar_reading()
        reading.ranges = reading.ranges + self.random_generator.normal(
            0, self.range_noise, len(reading.ranges)
        )
        return reading

    @property
    def odometry(self) -> HomogeneousTransformationMatrix:
        """
        :return: The pose of the base its odometry reports.
        """
        return HomogeneousTransformationMatrix(self._odom_T_base)


# %% comparing poses


def position_error(estimate: Pose2D, truth: Pose2D) -> float:
    """
    :return: The distance between the positions of two poses, in meters.
    """
    return float(np.linalg.norm(estimate.to_np()[:2] - truth.to_np()[:2]))


def yaw_error(estimate: Pose2D, truth: Pose2D) -> float:
    """
    :return: The absolute angle between the headings of two poses, in radians.
    """
    difference = estimate.to_np()[2] - truth.to_np()[2]
    return float(abs(np.arctan2(np.sin(difference), np.cos(difference))))
