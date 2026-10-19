import numpy as np
import pytest

from semantic_digital_twin.localization.scan_alignment import ScanAlignment
from semantic_digital_twin.spatial_types.spatial_types import Pose2D

from .room import Room, SimulatedRobot, position_error, yaw_error

# %% aligning a scan with the room


def test_aligning_a_scan_corrects_an_offset_estimate():
    world = Room().build()
    robot = SimulatedRobot(
        world,
        Pose2D(x=3.0, y=1.5, yaw=0.4),
        np.random.default_rng(0),
        range_noise=0.0,
    )
    x, y, yaw = robot.pose.to_np()

    aligned = ScanAlignment().align(
        Pose2D(x=x + 0.1, y=y - 0.08, yaw=yaw + 0.04),
        robot.read_scan(),
        robot.base_T_lidar.to_np(),
        world.ray_tracer,
    )

    assert position_error(aligned, robot.pose) == pytest.approx(0.0, abs=5e-3)
    assert yaw_error(aligned, robot.pose) == pytest.approx(0.0, abs=2e-3)
