from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
import numpy.typing as npt
from scipy.stats import norm

from semantic_digital_twin.localization.particles import ParticleSet, PoseHistogram
from semantic_digital_twin.world_description.geometry import PlanarBoundingBox

# %% recovering a lost estimate


@dataclass
class RandomParticleInjection:
    """
    Decides how many particles to replace by random ones, from how well recent scans fit
    the particles compared to how well they fitted over a longer time (augmented Monte
    Carlo localization).
    """

    slow_rate: float = 0.001
    """
    How fast the long-term average of the scan fit follows a new scan (AMCL's
    recovery_alpha_slow).
    """

    fast_rate: float = 0.1
    """
    How fast the short-term average of the scan fit follows a new scan (AMCL's
    recovery_alpha_fast).
    """

    _slow_average: float = field(init=False, default=0.0)
    """
    The long-term average of the scan fit, zero before the first scan.
    """

    _fast_average: float = field(init=False, default=0.0)
    """
    The short-term average of the scan fit, zero before the first scan.
    """

    def record(self, fit: float) -> None:
        """
        :param fit: How well the latest scan fits the particles, positive and larger for
            a better fit.
        """
        if self._slow_average == 0.0:
            self._slow_average = self._fast_average = fit
            return
        self._slow_average += self.slow_rate * (fit - self._slow_average)
        self._fast_average += self.fast_rate * (fit - self._fast_average)

    @property
    def probability(self) -> float:
        """
        :return: The probability that a resampled particle is replaced by a random one.
        """
        if self._slow_average == 0.0:
            return 0.0
        return max(0.0, 1.0 - self._fast_average / self._slow_average)


# %% resampling


@dataclass
class AdaptiveResampling:
    """
    Draws equally weighted particles from weighted ones, as many as needed to bound the
    error between the drawn and the weighted distribution (KLD sampling).

    The error is measured over a histogram of poses: the more cells the drawn particles
    occupy, the more particles are drawn.
    """

    minimum_particle_count: int = 100
    """
    The fewest particles drawn (AMCL's min_particles).
    """

    maximum_particle_count: int = 5000
    """
    The most particles drawn (AMCL's max_particles).
    """

    error_bound: float = 0.01
    """
    The Kullback-Leibler divergence the drawn distribution may have from the weighted
    one (AMCL's kld_err).
    """

    error_quantile: float = 0.99
    """
    The probability with which the divergence stays within :attr:`error_bound` (AMCL's
    kld_z).
    """

    histogram: PoseHistogram = field(default_factory=PoseHistogram)
    """
    The cells the error is measured over (AMCL's histogram of 0.5 m and 10 degrees).
    """

    def resample(
        self,
        particles: ParticleSet,
        random_particle_probability: float,
        region: PlanarBoundingBox,
        random_generator: np.random.Generator,
    ) -> ParticleSet:
        """
        :param particles: The weighted particles to draw from.
        :param random_particle_probability: The probability that a drawn particle is
            replaced by one at a random pose in the region.
        :param region: Where random particles are placed.
        :param random_generator: The source of the randomness.
        :return: The drawn particles.
        """
        drawn = random_generator.choice(
            len(particles), size=self.maximum_particle_count, p=particles.weights
        )
        poses = particles.poses[drawn]
        is_random = random_generator.random(len(poses)) < random_particle_probability
        if is_random.any():
            poses[is_random] = ParticleSet.uniform(
                region, int(is_random.sum()), random_generator
            ).poses

        cells = self.histogram.find_cells(poses)
        _, first_in_cell = np.unique(cells, axis=0, return_index=True)
        opens_cell = np.zeros(len(poses), dtype=bool)
        opens_cell[first_in_cell] = True
        drawn_counts = np.arange(1, len(poses) + 1)
        is_enough = drawn_counts >= self.required_particle_counts(np.cumsum(opens_cell))
        count = int(drawn_counts[is_enough][0]) if is_enough.any() else len(poses)
        return ParticleSet(poses=poses[:count], weights=np.full(count, 1 / count))

    def required_particle_counts(
        self, occupied_cell_counts: npt.NDArray[np.int_]
    ) -> npt.NDArray[np.int_]:
        """
        :param occupied_cell_counts: Numbers of histogram cells holding particles.
        :return: For each, how many particles keep the error within its bound, limited
            to the minimum and maximum particle count.
        """
        degrees_of_freedom = np.maximum(occupied_cell_counts - 1, 1)
        spread = 2 / (9 * degrees_of_freedom)
        quantile = norm.ppf(self.error_quantile)
        counts = (
            degrees_of_freedom
            / (2 * self.error_bound)
            * (1 - spread + np.sqrt(spread) * quantile) ** 3
        )
        counts = np.where(occupied_cell_counts > 1, np.ceil(counts), 0)
        return np.clip(
            counts, self.minimum_particle_count, self.maximum_particle_count
        ).astype(int)
