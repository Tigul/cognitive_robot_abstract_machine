from __future__ import annotations

import math
from abc import ABC, abstractmethod
from dataclasses import dataclass, field

import numpy as np
from typing_extensions import List, Optional

from semantic_digital_twin.datastructures.scan_pattern import ScanPattern
from semantic_digital_twin.spatial_types import Vector3
from semantic_digital_twin.world_description.world_entity import (
    KinematicStructureEntity,
)


@dataclass
class LaserReading:
    """
    One sweep of a laser scanner.

    Both lists hold one entry per beam and share their order, so ``direction[i]`` is the
    beam that measured ``distance[i]``.
    """

    direction: List[Vector3] = field(default_factory=list)
    """
    The direction of each beam, as a unit vector in the laser's frame.
    """

    distance: List[float] = field(default_factory=list)
    """
    The distance each beam travelled before it hit a surface, in meters.

    A beam that hit nothing within the scan pattern's range measures ``math.inf``.
    """


@dataclass
class Laser(ABC):
    """
    A source of laser scanner readings.

    Subclasses either receive their readings from a real scanner or produce them from
    the world, so code reading a laser does not have to know which one it holds.
    """

    root: Optional[KinematicStructureEntity] = field(default=None, kw_only=True)
    """
    The entity in the world the beams originate from.
    """

    @property
    @abstractmethod
    def scan_pattern(self) -> ScanPattern:
        """
        :return: The directions this laser sweeps and the distances it can measure.
        """

    @abstractmethod
    def get_laser_reading(self) -> LaserReading:
        """
        :return: The most recent sweep of this laser.
        """


@dataclass
class SimulatedLaser(Laser):
    """
    A laser that measures the world's collision geometry by casting a ray along every
    beam of its scan pattern.
    """

    pattern: ScanPattern
    """
    The directions this laser sweeps and the distances it can measure.
    """

    @property
    def scan_pattern(self) -> ScanPattern:
        return self.pattern

    def get_laser_reading(self) -> LaserReading:
        world_T_laser = self.root.global_transform.to_np()
        world_V_beams = self.pattern.beam_directions @ world_T_laser[:3, :3].T
        world_P_laser = np.tile(world_T_laser[:3, 3], (self.pattern.beam_count, 1))

        points, index_ray, _ = self.root._world.ray_tracer.ray_test(
            world_P_laser,
            world_P_laser + world_V_beams * self.pattern.maximum_range,
            multiple_hits=True,
            min_distance=self.pattern.minimum_range,
            max_distance=self.pattern.maximum_range,
        )

        return LaserReading(
            direction=self.pattern.beam_directions_in_frame(self.root),
            distance=self._nearest_hit_per_beam(points, index_ray, world_P_laser),
        )

    def _nearest_hit_per_beam(
        self, points: np.ndarray, index_ray: np.ndarray, world_P_laser: np.ndarray
    ) -> List[float]:
        """
        Reduces the hits of a ray test to the one distance each beam measures.

        :param points: The positions where the beams met a surface.
        :param index_ray: The beam each of those positions belongs to.
        :param world_P_laser: The origin of every beam.
        :return: The distance of the closest hit per beam, and ``math.inf`` for beams
            that hit nothing.

        ..note:: A beam can meet several surfaces, and the ray test does not order its
            hits, so the closest one is picked explicitly.
        """
        distances = np.full(self.pattern.beam_count, math.inf)
        if len(index_ray) == 0:
            return distances.tolist()

        hit_distances = np.linalg.norm(points - world_P_laser[index_ray], axis=1)
        farthest_first = np.argsort(hit_distances)[::-1]
        distances[index_ray[farthest_first]] = hit_distances[farthest_first]
        return distances.tolist()
