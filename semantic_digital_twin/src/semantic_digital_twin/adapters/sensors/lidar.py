from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field

import numpy as np
from typing_extensions import List, Self, TYPE_CHECKING

from semantic_digital_twin.datastructures.joint_state import JointState
from semantic_digital_twin.datastructures.lidar_reading import LidarReading
from semantic_digital_twin.datastructures.scan_pattern import ScanPattern
from semantic_digital_twin.robots.exceptions import MissingInputSourceError
from semantic_digital_twin.robots.input_source import InputSource
from semantic_digital_twin.robots.robot_part_mixins import (
    HasInputSource,
    TGenericInputSource,
)
from semantic_digital_twin.robots.robot_parts import Sensor
from semantic_digital_twin.world_description.world_entity import (
    KinematicStructureEntity,
)

if TYPE_CHECKING:
    from rclpy.node import Node


# %% where readings come from


@dataclass
class LidarSource(InputSource, ABC):
    """
    Where the readings of a lidar come from, either the world or a real scanner.
    """

    @abstractmethod
    def get_lidar_reading(self, lidar: Lidar) -> LidarReading:
        """
        :param lidar: The lidar whose beams are read.
        :return: The most recent sweep of that lidar.
        """


@dataclass
class SimulatedLidarSource(LidarSource):
    """
    A source that measures the world's collision geometry by casting a ray along every
    beam of the lidar's scan pattern.
    """

    def get_lidar_reading(self, lidar: Lidar) -> LidarReading:
        world_T_lidar = lidar.root.global_transform.to_np()
        return LidarReading(
            reference_frame=lidar.root,
            scan_pattern=lidar.scan_pattern,
            ranges=lidar.root._world.ray_tracer.measure_distances(
                origin_points=np.tile(
                    world_T_lidar[:3, 3], (lidar.scan_pattern.beam_count, 1)
                ),
                directions=lidar.scan_pattern.beam_directions @ world_T_lidar[:3, :3].T,
                minimum_distance=lidar.scan_pattern.minimum_range,
                maximum_distance=lidar.scan_pattern.maximum_range,
            ),
        )


# %% the lidar a robot carries


@dataclass(eq=False)
class Lidar(Sensor, HasInputSource[LidarSource], ABC):
    """
    A lidar is a sensor that measures the distance to the surfaces around it along a fan
    of beams.

    Subclasses state where they are mounted, and a source decides whether the readings
    are measured in the world or received from a real scanner.
    """

    scan_pattern: ScanPattern = field(kw_only=True)
    """
    The directions this lidar sweeps and the distances it can measure.
    """

    source: TGenericInputSource = field(
        default_factory=SimulatedLidarSource, kw_only=True
    )
    """
    Where the readings of this lidar come from.
    """

    def get_lidar_reading(self) -> LidarReading:
        """
        :return: The most recent sweep of this lidar.
        :raises MissingInputSourceError: If nothing says where the readings come from.
        """
        if self.source is None:
            raise MissingInputSourceError(robot_part=self)
        return self.source.get_lidar_reading(self)

    def setup_hardware_interfaces(self):
        pass

    def setup_joint_states(self) -> List[JointState]:
        return []

    @classmethod
    def simulated_source(cls) -> LidarSource:
        return SimulatedLidarSource()

    def real_source(self, node: Node) -> LidarSource:
        from semantic_digital_twin.adapters.ros.lidar import SubscribedLidarSource

        return SubscribedLidarSource(node=node, topic_name=self.topic_name)

    @classmethod
    @abstractmethod
    def with_source(
        cls, robot_root: KinematicStructureEntity, source: LidarSource
    ) -> Self:
        """
        :param robot_root: The root of the robot carrying this lidar.
        :param source: Where the readings come from, such as a real scanner on a topic.
        :return: This lidar, mounted on the body and sweeping the pattern its robot
            description declares.
        """

    @classmethod
    def with_simulated_source(cls, robot_root: KinematicStructureEntity) -> Self:
        """
        :param robot_root: The root of the robot carrying this lidar.
        :return: This lidar, measuring the world it stands in.
        """
        return cls.with_source(robot_root, cls.simulated_source())

    @classmethod
    def setup_default_configuration_in_world_below_robot_root(
        cls, robot_root: KinematicStructureEntity
    ) -> Self:
        return cls.with_simulated_source(robot_root)
