from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
import numpy.typing as npt

from semantic_digital_twin.datastructures.lidar_reading import LidarReading
from semantic_digital_twin.localization.beam_measurement_model import (
    BeamMeasurementModel,
)
from semantic_digital_twin.localization.motion_models import (
    DifferentialDriveMotionModel,
    MotionModel,
)
from semantic_digital_twin.localization.particles import ParticleSet
from semantic_digital_twin.localization.resampling import (
    AdaptiveResampling,
    RandomParticleInjection,
)
from semantic_digital_twin.localization.scan_alignment import ScanAlignment
from semantic_digital_twin.spatial_computations.raytracer import RayTracer
from semantic_digital_twin.spatial_types.spatial_types import (
    HomogeneousTransformationMatrix,
    Pose2D,
)
from semantic_digital_twin.world_description.geometry import PlanarBoundingBox

# %% pose estimate


@dataclass
class PoseEstimate:
    """
    Where a robot base stands in the world, as estimated at one odometry reading.
    """

    pose: Pose2D
    """
    The estimated pose of the base in the world root's frame.
    """

    covariance: npt.NDArray[np.float64]
    """
    The covariance of the estimate's x, y and yaw.
    """

    odom_T_base: HomogeneousTransformationMatrix
    """
    The odometry reading the estimate belongs to.
    """

    @property
    def world_T_odom(self) -> HomogeneousTransformationMatrix:
        """
        :return: The pose of the odometry frame in the world root's frame that places
            the base at the estimated pose.
        """
        return self.pose.homogeneous_matrix @ self.odom_T_base.inverse()


# %% Monte Carlo localization


@dataclass
class MonteCarloLocalization:
    """
    Estimates where a robot base stands in a known world from its odometry and the scans
    of a lidar it carries, with a particle filter.

    A filter step runs only once the base moved far enough since the previous one.
    """

    ray_tracer: RayTracer
    """
    The ray tracer holding the surfaces the lidar can see, without the robot's own.
    """

    base_T_lidar: HomogeneousTransformationMatrix
    """
    The pose of the lidar on the base.
    """

    region: PlanarBoundingBox
    """
    The part of the world root's x-y plane random particles are placed in.
    """

    particles: ParticleSet
    """
    The current hypotheses of the base's pose.
    """

    motion_model: MotionModel = field(default_factory=DifferentialDriveMotionModel)
    """
    How uncertain the motion is that the odometry measures.
    """

    measurement_model: BeamMeasurementModel = field(
        default_factory=BeamMeasurementModel
    )
    """
    How well a pose explains a scan.
    """

    resampling: AdaptiveResampling = field(default_factory=AdaptiveResampling)
    """
    How the particles are drawn anew.
    """

    random_particle_injection: RandomParticleInjection = field(
        default_factory=RandomParticleInjection
    )
    """
    How many random particles are drawn to recover a lost estimate.
    """

    scan_alignment: ScanAlignment | None = None
    """
    How the estimate of converged particles is refined, or None to leave it unrefined.
    """

    minimum_update_translation: float = 0.2
    """
    How far the base drives before the next filter step, in meters (AMCL's
    update_min_d).
    """

    minimum_update_rotation: float = np.pi / 6
    """
    How far the base turns before the next filter step, in radians (AMCL's
    update_min_a).
    """

    converged_position_deviation: float = 0.1
    """
    The standard deviation of the particles' x and y below which they count as converged
    for the scan alignment, in meters.
    """

    converged_yaw_deviation: float = 0.1
    """
    The standard deviation of the particles' yaw below which they count as converged for
    the scan alignment, in radians.
    """

    random_generator: np.random.Generator = field(default_factory=np.random.default_rng)
    """
    The source of the filter's randomness.
    """

    estimate: PoseEstimate | None = field(init=False, default=None)
    """
    The estimate of the latest filter step, or None before the first.
    """

    def step(
        self, odom_T_base: HomogeneousTransformationMatrix, reading: LidarReading
    ) -> PoseEstimate:
        """
        Moves the particles by the odometry since the previous filter step, weighs them
        by the scan and draws them anew when their weights spread out or the estimate
        seems lost.

        :param odom_T_base: The odometry reading taken when the scan was measured.
        :param reading: The scan the lidar measured.
        :return: The estimate of the latest filter step.
        """
        if self.estimate is not None:
            motion = self._motion_since_estimate(odom_T_base)
            x, y, yaw = motion.to_np()
            if (
                np.hypot(x, y) < self.minimum_update_translation
                and abs(yaw) < self.minimum_update_rotation
            ):
                return self.estimate
            self.particles.move(
                self.motion_model.sample_displacements(
                    motion, len(self.particles), self.random_generator
                )
            )

        base_T_lidar = self.base_T_lidar.to_np()
        self._weigh_particles(reading, base_T_lidar)
        cluster = self.particles.heaviest_cluster(self.resampling.histogram)
        self.estimate = PoseEstimate(
            pose=self._estimate_pose(cluster, reading, base_T_lidar),
            covariance=cluster.covariance(),
            odom_T_base=odom_T_base,
        )

        if (
            self.particles.effective_sample_size < len(self.particles) / 2
            or self.random_particle_injection.probability > 0
        ):
            self.particles = self.resampling.resample(
                self.particles,
                self.random_particle_injection.probability,
                self.region,
                self.random_generator,
            )
        return self.estimate

    def _weigh_particles(
        self, reading: LidarReading, base_T_lidar: npt.NDArray[np.float64]
    ) -> None:
        """
        Multiplies the weight of every particle by the likelihood of the scan there and
        records how well the scan fits.

        :param reading: The scan the lidar measured.
        :param base_T_lidar: The pose of the lidar on the base.
        """
        likelihoods = self.measurement_model.weigh(
            self.particles, reading, base_T_lidar, self.ray_tracer
        )
        log_weights = np.log(self.particles.weights) + likelihoods.log_likelihoods
        weights = np.exp(log_weights - log_weights.max())
        self.particles.weights = weights / weights.sum()
        self.random_particle_injection.record(likelihoods.fit(self.particles.weights))

    def _motion_since_estimate(
        self, odom_T_base: HomogeneousTransformationMatrix
    ) -> Pose2D:
        """
        :param odom_T_base: The current odometry reading.
        :return: The pose of the base at the current odometry reading, in its frame at
            the reading of :attr:`estimate`.
        """
        motion = np.linalg.inv(self.estimate.odom_T_base.to_np()) @ odom_T_base.to_np()
        return Pose2D(
            x=motion[0, 3], y=motion[1, 3], yaw=np.arctan2(motion[1, 0], motion[0, 0])
        )

    def _estimate_pose(
        self,
        cluster: ParticleSet,
        reading: LidarReading,
        base_T_lidar: npt.NDArray[np.float64],
    ) -> Pose2D:
        """
        :param cluster: The particles the estimate is taken from.
        :param reading: The scan the particles were weighed by.
        :param base_T_lidar: The pose of the lidar on the base.
        :return: The mean of the cluster, aligned with the scan if the cluster converged
            and a scan alignment is set.
        """
        mean = cluster.mean()
        if self.scan_alignment is None:
            return mean
        deviations = np.sqrt(np.diag(cluster.covariance()))
        if (
            deviations[:2].max() > self.converged_position_deviation
            or deviations[2] > self.converged_yaw_deviation
        ):
            return mean
        return self.scan_alignment.align(mean, reading, base_T_lidar, self.ray_tracer)
