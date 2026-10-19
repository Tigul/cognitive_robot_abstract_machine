import numpy as np
import pytest

from semantic_digital_twin.localization.particles import ParticleSet, PoseHistogram
from semantic_digital_twin.spatial_types.spatial_types import Pose2D

from .room import Room, yaw_error

# %% building particle sets


def test_uniform_particles_cover_the_region_with_equal_weights():
    floor = Room().floor

    particles = ParticleSet.uniform(floor, 1000, np.random.default_rng(0))

    assert len(particles) == 1000
    assert particles.poses[:, 0].min() >= floor.min_x
    assert particles.poses[:, 0].max() <= floor.max_x
    assert particles.poses[:, 1].min() >= floor.min_y
    assert particles.poses[:, 1].max() <= floor.max_y
    np.testing.assert_allclose(particles.weights, 1 / 1000)


def test_particles_around_a_pose_spread_by_the_given_covariance():
    covariance = np.diag([0.04, 0.01, 0.0025])

    particles = ParticleSet.around(
        Pose2D(x=1.0, y=2.0, yaw=0.5), covariance, 20000, np.random.default_rng(0)
    )

    np.testing.assert_allclose(particles.covariance(), covariance, atol=2e-3)


# %% summarizing particle sets


def test_mean_heading_of_particles_on_both_sides_of_pi_points_backwards():
    particles = ParticleSet(
        poses=np.array([[0.0, 0.0, np.pi - 0.1], [0.0, 0.0, -np.pi + 0.1]]),
        weights=np.array([0.5, 0.5]),
    )

    assert yaw_error(particles.mean(), Pose2D(yaw=np.pi)) == pytest.approx(0.0)


def test_mean_position_weighs_every_particle_by_its_weight():
    particles = ParticleSet(
        poses=np.array([[0.0, 0.0, 0.0], [4.0, 0.0, 0.0]]),
        weights=np.array([0.75, 0.25]),
    )

    np.testing.assert_allclose(particles.mean().to_np(), [1.0, 0.0, 0.0])


def test_equal_weights_count_every_particle_as_effective():
    particles = ParticleSet(poses=np.zeros((4, 3)), weights=np.full(4, 0.25))

    assert particles.effective_sample_size == pytest.approx(4.0)


def test_one_particle_holding_all_weight_is_the_only_effective_one():
    particles = ParticleSet(poses=np.zeros((4, 3)), weights=np.array([1.0, 0, 0, 0]))

    assert particles.effective_sample_size == pytest.approx(1.0)


# %% moving particles


def test_a_displacement_moves_each_particle_in_its_own_frame():
    particles = ParticleSet(
        poses=np.array([[0.0, 0.0, 0.0], [1.0, 1.0, np.pi / 2]]),
        weights=np.full(2, 0.5),
    )

    particles.move(np.array([[1.0, 0.0, 0.1], [1.0, 0.0, 0.1]]))

    np.testing.assert_allclose(
        particles.poses, [[1.0, 0.0, 0.1], [1.0, 2.0, np.pi / 2 + 0.1]], atol=1e-12
    )


def test_moved_headings_stay_between_minus_pi_and_pi():
    particles = ParticleSet(poses=np.array([[0.0, 0.0, 3.0]]), weights=np.ones(1))

    particles.move(np.array([[0.0, 0.0, 1.0]]))

    assert particles.poses[0, 2] == pytest.approx(4.0 - 2 * np.pi)


# %% clustering particles


def test_the_heaviest_cluster_holds_the_group_with_the_most_weight():
    histogram = PoseHistogram()
    light_group = np.tile([1.0, 1.0, 0.0], (3, 1))
    heavy_group = np.tile([4.0, 3.0, 0.0], (2, 1))
    particles = ParticleSet(
        poses=np.vstack((light_group, heavy_group)),
        weights=np.array([0.1, 0.1, 0.1, 0.35, 0.35]),
    )

    cluster = particles.heaviest_cluster(histogram)

    np.testing.assert_array_equal(cluster.poses, heavy_group)
    np.testing.assert_allclose(cluster.weights, [0.5, 0.5])


def test_a_cluster_extends_across_the_turn_from_pi_to_minus_pi():
    histogram = PoseHistogram()
    particles = ParticleSet(
        poses=np.array(
            [[1.0, 1.0, np.pi - 0.01], [1.0, 1.0, -np.pi + 0.01], [4.0, 3.0, 0.0]]
        ),
        weights=np.array([0.3, 0.3, 0.4]),
    )

    cluster = particles.heaviest_cluster(histogram)

    np.testing.assert_array_equal(cluster.poses, particles.poses[:2])
