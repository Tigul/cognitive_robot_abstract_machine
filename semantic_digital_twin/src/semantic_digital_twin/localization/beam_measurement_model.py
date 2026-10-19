from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import numpy.typing as npt

from semantic_digital_twin.datastructures.lidar_reading import LidarReading
from semantic_digital_twin.localization.particles import ParticleSet
from semantic_digital_twin.spatial_computations.raytracer import RayTracer

# %% scan likelihoods


@dataclass
class ScanLikelihoods:
    """
    How likely one scan is at each particle.
    """

    log_likelihoods: npt.NDArray[np.float64]
    """
    The log-likelihood of the scan at every particle, the sum of the logs of the
    compared beams' probabilities.
    """

    beam_count: int
    """
    How many beams of the scan were compared.
    """

    def fit(self, weights: npt.NDArray[np.float64]) -> float:
        """
        :param weights: The weight of every particle, summing to one.
        :return: The geometric mean probability of a compared beam, averaged over the
            particles by their weights.
        """
        return float(np.exp(weights @ self.log_likelihoods / self.beam_count))


# %% beam measurement model


@dataclass
class BeamMeasurementModel:
    """
    How well a pose explains a lidar scan, judged by comparing every measured range with
    the range a ray cast from that pose predicts.

    Each beam's probability mixes a hit near the predicted surface, a reading cut short
    by something unmodelled, a reading at the maximum range and a random reading, and
    the beams of a scan count as independent. The mixture's defaults are those of AMCL's
    beam model.
    """

    hit_weight: float = 0.5
    """
    The weight of a hit near the predicted surface (AMCL's z_hit).
    """

    short_weight: float = 0.05
    """
    The weight of a reading shorter than the predicted surface (AMCL's z_short).
    """

    maximum_range_weight: float = 0.05
    """
    The weight of a reading at the maximum range (AMCL's z_max).
    """

    random_weight: float = 0.5
    """
    The weight of a reading anywhere within the range (AMCL's z_rand).
    """

    hit_standard_deviation: float = 0.2
    """
    The spread of a hit around the predicted surface, in meters (AMCL's sigma_hit).
    """

    short_decay_rate: float = 0.1
    """
    How fast a short reading becomes less likely with its range, per meter (AMCL's
    lambda_short).
    """

    used_beam_count: int = 60
    """
    How many evenly spread beams of a scan are compared (AMCL's max_beams).
    """

    def weigh(
        self,
        particles: ParticleSet,
        reading: LidarReading,
        base_T_lidar: npt.NDArray[np.float64],
        ray_tracer: RayTracer,
    ) -> ScanLikelihoods:
        """
        :param particles: The poses of the base to judge.
        :param reading: The scan the lidar measured.
        :param base_T_lidar: The pose of the lidar on the base.
        :param ray_tracer: The ray tracer predicting the ranges.
        :return: The likelihood of the scan at every particle.
        """
        pattern = reading.scan_pattern
        used_beams = np.unique(
            np.linspace(0, pattern.beam_count - 1, self.used_beam_count).round()
        ).astype(int)
        used_beams = used_beams[~np.isnan(reading.ranges[used_beams])]
        probabilities = self.beam_probabilities(
            reading.ranges[used_beams],
            self._predict_ranges(
                particles, reading, used_beams, base_T_lidar, ray_tracer
            ),
            pattern.maximum_range,
        )
        return ScanLikelihoods(
            log_likelihoods=np.sum(np.log(probabilities), axis=-1),
            beam_count=len(used_beams),
        )

    @staticmethod
    def _predict_ranges(
        particles: ParticleSet,
        reading: LidarReading,
        used_beams: npt.NDArray[np.int_],
        base_T_lidar: npt.NDArray[np.float64],
        ray_tracer: RayTracer,
    ) -> npt.NDArray[np.float64]:
        """
        :param particles: The poses of the base to predict the ranges at.
        :param reading: The scan whose beams are predicted.
        :param used_beams: The indices of the predicted beams.
        :param base_T_lidar: The pose of the lidar on the base.
        :param ray_tracer: The ray tracer predicting the ranges.
        :return: The range every used beam measures at every particle, one particle
            per row, and ``numpy.inf`` where a beam meets no surface within range.
        """
        pattern = reading.scan_pattern
        world_T_lidars = particles.to_transformation_matrices() @ base_T_lidar
        directions = np.einsum(
            "pij,bj->pbi",
            world_T_lidars[:, :3, :3],
            pattern.beam_directions[used_beams],
        )
        return ray_tracer.measure_distances(
            origin_points=np.repeat(world_T_lidars[:, :3, 3], len(used_beams), axis=0),
            directions=directions.reshape((-1, 3)),
            minimum_distance=pattern.minimum_range,
            maximum_distance=pattern.maximum_range,
        ).reshape((len(particles), len(used_beams)))

    def beam_probabilities(
        self,
        measured_ranges: npt.NDArray[np.float64],
        expected_ranges: npt.NDArray[np.float64],
        maximum_range: float,
    ) -> npt.NDArray[np.float64]:
        """
        :param measured_ranges: The range each beam measured, in meters.
        :param expected_ranges: The range each beam is predicted to measure, in meters,
            broadcastable against the measured ranges.
        :param maximum_range: The farthest range the lidar measures; longer ranges count
            as this range.
        :return: The probability of every measured range given its expected range.
        """
        measured_ranges = np.minimum(measured_ranges, maximum_range)
        expected_ranges = np.minimum(expected_ranges, maximum_range)
        reaches_maximum = measured_ranges >= maximum_range
        hit = self.hit_weight * np.exp(
            -((measured_ranges - expected_ranges) ** 2)
            / (2 * self.hit_standard_deviation**2)
        )
        short = np.where(
            measured_ranges < expected_ranges,
            self.short_weight
            * self.short_decay_rate
            * np.exp(-self.short_decay_rate * measured_ranges),
            0.0,
        )
        at_maximum = np.where(reaches_maximum, self.maximum_range_weight, 0.0)
        random = np.where(reaches_maximum, 0.0, self.random_weight / maximum_range)
        return hit + short + at_maximum + random
