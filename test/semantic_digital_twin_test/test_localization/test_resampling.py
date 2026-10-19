import numpy as np
import pytest

from semantic_digital_twin.localization.particles import ParticleSet
from semantic_digital_twin.localization.resampling import (
    AdaptiveResampling,
    RandomParticleInjection,
)

from .room import Room

# %% random particle injection


def test_no_random_particles_are_drawn_while_the_scans_fit_equally_well():
    injection = RandomParticleInjection()

    for _ in range(20):
        injection.record(0.5)

    assert injection.probability == 0.0


def test_random_particles_are_drawn_once_the_scans_fit_worse_than_before():
    injection = RandomParticleInjection()
    for _ in range(20):
        injection.record(0.5)

    injection.record(0.1)

    assert injection.probability > 0.0


# %% adaptive resampling


def test_particles_crowded_into_one_cell_are_resampled_to_the_fewest_allowed():
    resampling = AdaptiveResampling()
    particles = ParticleSet(poses=np.zeros((1000, 3)), weights=np.full(1000, 1e-3))

    resampled = resampling.resample(
        particles, 0.0, Room().floor, np.random.default_rng(0)
    )

    assert len(resampled) == resampling.minimum_particle_count


def test_particles_spread_over_the_room_are_resampled_to_the_most_allowed():
    resampling = AdaptiveResampling()
    floor = Room().floor
    particles = ParticleSet.uniform(floor, 5000, np.random.default_rng(0))

    resampled = resampling.resample(particles, 0.0, floor, np.random.default_rng(0))

    assert len(resampled) == resampling.maximum_particle_count


def test_resampling_draws_poses_by_their_weights():
    particles = ParticleSet(
        poses=np.array([[1.0, 1.0, 0.0], [3.0, 2.0, 1.0]]),
        weights=np.array([0.0, 1.0]),
    )

    resampled = AdaptiveResampling().resample(
        particles, 0.0, Room().floor, np.random.default_rng(0)
    )

    np.testing.assert_array_equal(
        np.unique(resampled.poses, axis=0), particles.poses[[1]]
    )
    np.testing.assert_allclose(resampled.weights, 1 / len(resampled))


def test_resampling_with_certain_injection_places_every_particle_at_random():
    floor = Room().floor
    particles = ParticleSet(poses=np.zeros((10, 3)), weights=np.full(10, 0.1))

    resampled = AdaptiveResampling().resample(
        particles, 1.0, floor, np.random.default_rng(0)
    )

    assert not np.any(np.all(resampled.poses == 0.0, axis=1))
