import numpy as np
import pytest

from semantic_digital_twin.localization.monte_carlo_localization import (
    MonteCarloLocalization,
)
from semantic_digital_twin.localization.particles import ParticleSet
from semantic_digital_twin.localization.scan_alignment import ScanAlignment
from semantic_digital_twin.spatial_types.spatial_types import Pose2D

from .room import Room, SimulatedRobot, position_error, yaw_error

# %% helpers


def localization_in_room(
    room: Room, robot: SimulatedRobot, particles: ParticleSet, seed: int = 1
) -> MonteCarloLocalization:
    """
    :return: A localization of the robot in the room, starting from the given particles.
    """
    return MonteCarloLocalization(
        ray_tracer=robot.world.ray_tracer,
        base_T_lidar=robot.base_T_lidar,
        region=room.floor,
        particles=particles,
        random_generator=np.random.default_rng(seed),
    )


def drive_circle(
    robot: SimulatedRobot,
    localization: MonteCarloLocalization,
    step_count: int,
    arc_length: float = 0.2,
    arc_rotation: float = 0.2,
) -> None:
    """
    Drives the robot along a circle and runs one filter step after every arc.

    :param step_count: How many arcs the robot drives.
    :param arc_length: How far the robot drives between two scans, in meters.
    :param arc_rotation: How far the robot turns between two scans, in radians.
    """
    for _ in range(step_count):
        robot.drive(arc_length, arc_rotation)
        localization.step(robot.odometry, robot.read_scan())


def estimated_pose(
    localization: MonteCarloLocalization, robot: SimulatedRobot
) -> Pose2D:
    """
    :return: Where the localization places the robot's base at its current odometry
        reading.
    """
    return Pose2D.from_pose((localization.estimate.world_T_odom @ robot.odometry).pose)


def particles_near(
    pose: Pose2D, standard_deviation: float = 0.2, count: int = 500
) -> ParticleSet:
    """
    :param standard_deviation: The spread of the particles' positions in meters, and of
        their headings in radians.
    :return: Particles spread around a pose, as after an initial pose estimate.
    """
    return ParticleSet.around(
        pose,
        np.diag([standard_deviation**2] * 2 + [(standard_deviation / 2) ** 2]),
        count,
        np.random.default_rng(2),
    )


# %% tracking


def test_tracking_follows_the_robot_around_a_circle():
    room = Room()
    robot = SimulatedRobot(
        room.build(), Pose2D(x=3.0, y=1.0, yaw=0.0), np.random.default_rng(0)
    )
    localization = localization_in_room(
        room, robot, particles_near(Pose2D(x=3.15, y=0.9, yaw=0.1))
    )
    localization.step(robot.odometry, robot.read_scan())

    drive_circle(robot, localization, 31)

    assert position_error(estimated_pose(localization, robot), robot.pose) < 0.1
    assert yaw_error(estimated_pose(localization, robot), robot.pose) < 0.05


def test_odom_pose_places_the_odometry_reading_at_the_estimated_pose():
    room = Room()
    robot = SimulatedRobot(
        room.build(), Pose2D(x=3.0, y=1.0, yaw=0.0), np.random.default_rng(0)
    )
    localization = localization_in_room(room, robot, particles_near(robot.pose))
    localization.step(robot.odometry, robot.read_scan())
    drive_circle(robot, localization, 5)

    estimate = localization.estimate

    np.testing.assert_allclose(
        (estimate.world_T_odom @ estimate.odom_T_base).to_np(),
        estimate.pose.homogeneous_matrix.to_np(),
        atol=1e-9,
    )


def test_a_standing_robot_leaves_the_particles_where_they_are():
    room = Room()
    robot = SimulatedRobot(
        room.build(), Pose2D(x=3.0, y=1.0, yaw=0.0), np.random.default_rng(0)
    )
    localization = localization_in_room(room, robot, particles_near(robot.pose))
    localization.step(robot.odometry, robot.read_scan())
    poses_before = localization.particles.poses.copy()

    localization.step(robot.odometry, robot.read_scan())

    np.testing.assert_array_equal(localization.particles.poses, poses_before)


# %% global localization and recovery


def test_global_localization_finds_the_robot_from_particles_spread_over_the_room():
    room = Room()
    robot = SimulatedRobot(
        room.build(), Pose2D(x=3.0, y=1.0, yaw=0.0), np.random.default_rng(0)
    )
    localization = localization_in_room(
        room, robot, ParticleSet.uniform(room.floor, 5000, np.random.default_rng(2))
    )
    localization.step(robot.odometry, robot.read_scan())

    drive_circle(robot, localization, 31)

    assert position_error(estimated_pose(localization, robot), robot.pose) < 0.1
    assert yaw_error(estimated_pose(localization, robot), robot.pose) < 0.05


def test_a_kidnapped_robot_is_found_again():
    room = Room()
    robot = SimulatedRobot(
        room.build(), Pose2D(x=3.0, y=1.0, yaw=0.0), np.random.default_rng(0)
    )
    localization = localization_in_room(room, robot, particles_near(robot.pose))
    localization.step(robot.odometry, robot.read_scan())
    drive_circle(robot, localization, 10)

    robot.place(Pose2D(x=2.5, y=2.8, yaw=np.pi))
    drive_circle(robot, localization, 62)

    assert position_error(estimated_pose(localization, robot), robot.pose) < 0.1
    assert yaw_error(estimated_pose(localization, robot), robot.pose) < 0.05


# %% refinement


def test_scan_alignment_refines_the_estimate_of_converged_particles():
    room = Room()
    robot = SimulatedRobot(
        room.build(),
        Pose2D(x=3.0, y=1.0, yaw=0.0),
        np.random.default_rng(0),
        range_noise=0.0,
    )
    localization = localization_in_room(
        room,
        robot,
        particles_near(Pose2D(x=3.05, y=0.95, yaw=0.03), standard_deviation=0.02),
    )
    localization.scan_alignment = ScanAlignment()

    localization.step(robot.odometry, robot.read_scan())

    assert position_error(localization.estimate.pose, robot.pose) == pytest.approx(
        0.0, abs=5e-3
    )
