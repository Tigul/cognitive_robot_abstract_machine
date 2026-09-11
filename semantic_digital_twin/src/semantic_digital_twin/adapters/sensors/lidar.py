from __future__ import annotations

import math
from abc import ABC
from dataclasses import dataclass

import numpy as np
from typing_extensions import List

from semantic_digital_twin.datastructures.joint_state import JointState
from semantic_digital_twin.datastructures.lidar_reading import LidarReading
from semantic_digital_twin.robots.robot_parts import Lidar


@dataclass(eq=False)
class SimulatedLidar(Lidar, ABC):
    """
    A lidar that measures the world's collision geometry by casting a ray along every
    beam of its scan pattern.
    """

    def setup_hardware_interfaces(self):
        pass

    def setup_joint_states(self) -> List[JointState]:
        return []

    def get_lidar_reading(self) -> LidarReading:
        world_T_lidar = self.root.global_transform.to_np()
        world_V_beams = self.scan_pattern.beam_directions @ world_T_lidar[:3, :3].T
        world_P_lidar = np.tile(world_T_lidar[:3, 3], (self.scan_pattern.beam_count, 1))

        points, index_ray, _ = self.root._world.ray_tracer.ray_test(
            world_P_lidar,
            world_P_lidar + world_V_beams * self.scan_pattern.maximum_range,
            multiple_hits=True,
            min_distance=self.scan_pattern.minimum_range,
            max_distance=self.scan_pattern.maximum_range,
        )

        return LidarReading(
            direction=self.beam_directions,
            distance=self._nearest_hit_per_beam(points, index_ray, world_P_lidar),
        )

    def _nearest_hit_per_beam(
        self, points: np.ndarray, index_ray: np.ndarray, world_P_lidar: np.ndarray
    ) -> List[float]:
        """
        Reduces the hits of a ray test to the one distance each beam measures.

        :param points: The positions where the beams met a surface.
        :param index_ray: The beam each of those positions belongs to.
        :param world_P_lidar: The origin of every beam.
        :return: The distance of the closest hit per beam, and ``math.inf`` for beams
            that hit nothing.

        ..note:: A beam can meet several surfaces, and the ray test does not order its
            hits, so the closest one is picked explicitly.
        """
        distances = np.full(self.scan_pattern.beam_count, math.inf)
        if len(index_ray) == 0:
            return distances.tolist()

        hit_distances = np.linalg.norm(points - world_P_lidar[index_ray], axis=1)
        farthest_first = np.argsort(hit_distances)[::-1]
        distances[index_ray[farthest_first]] = hit_distances[farthest_first]
        return distances.tolist()
