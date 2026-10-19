from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import numpy.typing as npt

from semantic_digital_twin.datastructures.lidar_reading import LidarReading
from semantic_digital_twin.spatial_computations.raytracer import RayTracer
from semantic_digital_twin.spatial_types.spatial_types import Pose2D

# %% scan alignment


@dataclass
class ScanAlignment:
    """
    Refines the pose of a robot base on the floor by fitting its lidar scan to the
    surfaces of the world with point-to-plane ICP.
    """

    iteration_count: int = 10
    """
    How often the scan points are paired with surface points and moved.
    """

    maximum_pairing_distance: float = 0.5
    """
    Scan points farther than this from every surface are left out, in meters.
    """

    def align(
        self,
        pose: Pose2D,
        reading: LidarReading,
        base_T_lidar: npt.NDArray[np.float64],
        ray_tracer: RayTracer,
    ) -> Pose2D:
        """
        :param pose: The pose of the base to start from, in the world root's frame.
        :param reading: The scan the lidar measured.
        :param base_T_lidar: The pose of the lidar on the base.
        :param ray_tracer: The ray tracer holding the surfaces to align with.
        :return: The pose of the base at which the scan fits the surfaces best.
        """
        lidar_P_points = np.column_stack((reading.points, np.ones(len(reading.points))))
        base_P_points = (lidar_P_points @ base_T_lidar.T)[:, :3]
        x, y, yaw = pose.to_np()
        for _ in range(self.iteration_count):
            cosine, sine = np.cos(yaw), np.sin(yaw)
            # Scan points relative to the base's position, so that a correction of the
            # yaw turns them about the base.
            relative_points = base_P_points @ np.array(
                [[cosine, sine, 0.0], [-sine, cosine, 0.0], [0.0, 0.0, 1.0]]
            )
            correction = self._find_correction(
                relative_points, relative_points + [x, y, 0.0], ray_tracer
            )
            x, y, yaw = np.array([x, y, yaw]) + correction
        return Pose2D(x=x, y=y, yaw=yaw)

    def _find_correction(
        self,
        relative_points: npt.NDArray[np.float64],
        world_P_points: npt.NDArray[np.float64],
        ray_tracer: RayTracer,
    ) -> npt.NDArray[np.float64]:
        """
        :param relative_points: The scan points relative to the base's position, one per
            row, oriented like the world root's frame.
        :param world_P_points: The same scan points in the world root's frame.
        :param ray_tracer: The ray tracer holding the surfaces to align with.
        :return: The change of the base's x, y and yaw that brings the scan points
            closest to the planes of the surfaces nearest to them, to first order.
        """
        surface = ray_tracer.find_closest_surface_points(world_P_points)
        offsets = world_P_points - surface.positions
        is_paired = np.linalg.norm(offsets, axis=1) <= self.maximum_pairing_distance
        normals = surface.normals[is_paired]
        relative_points = relative_points[is_paired]
        jacobian = np.column_stack(
            (
                normals[:, 0],
                normals[:, 1],
                normals[:, 1] * relative_points[:, 0]
                - normals[:, 0] * relative_points[:, 1],
            )
        )
        correction, *_ = np.linalg.lstsq(
            jacobian, -np.sum(normals * offsets[is_paired], axis=1), rcond=None
        )
        return correction
