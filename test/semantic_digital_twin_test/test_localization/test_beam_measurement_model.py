import numpy as np
import pytest

from semantic_digital_twin.localization.beam_measurement_model import (
    BeamMeasurementModel,
    ScanLikelihoods,
)
from semantic_digital_twin.localization.particles import ParticleSet
from semantic_digital_twin.spatial_types.spatial_types import Pose2D

from .room import Room, SimulatedRobot

# %% single beams


def test_a_beam_measuring_the_expected_range_scores_a_hit_and_a_random_reading():
    model = BeamMeasurementModel()
    maximum_range = 10.0

    [probability] = model.beam_probabilities(
        np.array([2.0]), np.array([2.0]), maximum_range
    )

    assert probability == pytest.approx(
        model.hit_weight + model.random_weight / maximum_range
    )


def test_a_beam_ending_short_of_the_expected_surface_also_scores_a_short_reading():
    model = BeamMeasurementModel()
    maximum_range = 10.0
    measured, expected = 1.0, 3.0

    [probability] = model.beam_probabilities(
        np.array([measured]), np.array([expected]), maximum_range
    )

    assert probability == pytest.approx(
        model.hit_weight
        * np.exp(-((measured - expected) ** 2) / (2 * model.hit_standard_deviation**2))
        + model.short_weight
        * model.short_decay_rate
        * np.exp(-model.short_decay_rate * measured)
        + model.random_weight / maximum_range
    )


def test_a_beam_reaching_the_maximum_range_scores_a_maximum_range_reading():
    model = BeamMeasurementModel()
    maximum_range = 10.0

    [probability] = model.beam_probabilities(
        np.array([np.inf]), np.array([np.inf]), maximum_range
    )

    assert probability == pytest.approx(model.hit_weight + model.maximum_range_weight)


# %% whole scans


def scan_likelihoods_at_shifted_poses(model: BeamMeasurementModel) -> ScanLikelihoods:
    """
    :return: The likelihoods a model gives a scan at the true pose and two poses near it.
    """
    world = Room().build()
    robot = SimulatedRobot(
        world, Pose2D(x=3.0, y=1.0, yaw=0.3), np.random.default_rng(0)
    )
    x, y, yaw = robot.pose.to_np()
    particles = ParticleSet(
        poses=np.array([[x, y, yaw], [x + 0.3, y, yaw], [x, y, yaw + 0.2]]),
        weights=np.full(3, 1 / 3),
    )
    return model.weigh(
        particles, robot.read_scan(), robot.base_T_lidar.to_np(), world.ray_tracer
    )


def test_the_true_pose_explains_a_scan_better_than_a_shifted_pose():
    likelihoods = scan_likelihoods_at_shifted_poses(BeamMeasurementModel())

    assert likelihoods.log_likelihoods.argmax() == 0


# %% tempering


def test_the_likelihood_exponent_scales_the_log_likelihoods():
    untempered = scan_likelihoods_at_shifted_poses(
        BeamMeasurementModel(likelihood_exponent=1.0)
    )
    model = BeamMeasurementModel()

    tempered = scan_likelihoods_at_shifted_poses(model)

    np.testing.assert_allclose(
        tempered.log_likelihoods,
        model.likelihood_exponent * untempered.log_likelihoods,
    )


def test_the_fit_of_a_scan_does_not_depend_on_the_likelihood_exponent():
    untempered = scan_likelihoods_at_shifted_poses(
        BeamMeasurementModel(likelihood_exponent=1.0)
    )
    tempered = scan_likelihoods_at_shifted_poses(BeamMeasurementModel())
    weights = np.array([0.5, 0.3, 0.2])

    assert tempered.fit(weights) == pytest.approx(untempered.fit(weights))
