from __future__ import annotations

import numpy as np
import pytest
from sensor_msgs.msg import LaserScan

from semantic_digital_twin.adapters.ros.laser import SubscribedLaser
from semantic_digital_twin.adapters.ros.msg_converter import (
    LaserScanBeamCountMismatch,
    Ros2ToSemDTConverter,
)
from semantic_digital_twin.adapters.ros.ros2_to_semdt_converters import (
    LaserScanToSemDTConverter,
)
from semantic_digital_twin.exceptions import NoLaserScanReceived
from semantic_digital_twin.world import World

# %% scan messages under test

LASER_FRAME_NAME = "base_laser_link"
"""
Name of the body the scans below are expressed in.
"""

RANGES = [1.0, 2.0, 3.0]
"""
Measured distances of the beams of :func:`laser_scan`.
"""


def laser_scan(ranges: list[float] = None) -> LaserScan:
    """
    :param ranges: The measured distances, defaulting to :data:`RANGES`.
    :return: A scan of three beams spanning a right angle around the forward axis.
    """
    scan = LaserScan()
    scan.header.frame_id = LASER_FRAME_NAME
    scan.angle_min = -np.pi / 4
    scan.angle_max = np.pi / 4
    scan.angle_increment = np.pi / 4
    scan.range_min = 0.1
    scan.range_max = 10.0
    scan.ranges = list(RANGES if ranges is None else ranges)
    return scan


@pytest.fixture
def world_with_laser_body() -> World:
    """
    A world whose root body carries the name the scan messages refer to.
    """
    return World.create_with_root_body(LASER_FRAME_NAME)


# %% message conversion


def test_converted_scan_keeps_one_direction_and_one_distance_per_range(
    world_with_laser_body,
):
    reading = LaserScanToSemDTConverter.convert(laser_scan(), world_with_laser_body)

    assert reading.distance == RANGES
    assert len(reading.direction) == len(RANGES)


def test_converted_scan_spans_the_angles_the_message_declares(world_with_laser_body):
    scan = laser_scan()

    reading = LaserScanToSemDTConverter.convert(scan, world_with_laser_body)

    assert np.allclose(
        reading.direction[0].to_np(),
        [np.cos(scan.angle_min), np.sin(scan.angle_min), 0.0, 0.0],
    )
    assert np.allclose(
        reading.direction[-1].to_np(),
        [np.cos(scan.angle_max), np.sin(scan.angle_max), 0.0, 0.0],
    )


def test_converted_scan_is_expressed_in_the_body_named_by_its_header(
    world_with_laser_body,
):
    reading = LaserScanToSemDTConverter.convert(laser_scan(), world_with_laser_body)

    assert reading.direction[0].reference_frame is world_with_laser_body.root


def test_scan_whose_range_count_disagrees_with_its_angles_is_rejected(
    world_with_laser_body,
):
    with pytest.raises(LaserScanBeamCountMismatch):
        LaserScanToSemDTConverter.convert(
            laser_scan(ranges=[1.0, 2.0]), world_with_laser_body
        )


def test_converter_is_found_by_the_registry(world_with_laser_body):
    scan = laser_scan()

    assert Ros2ToSemDTConverter.get_to_converter(scan) is LaserScanToSemDTConverter


# %% subscribed laser


def test_subscribed_laser_reports_the_reading_of_its_latest_scan(
    rclpy_node, world_with_laser_body
):
    scan = laser_scan()
    laser = SubscribedLaser(
        node=rclpy_node, topic_name="/scan", root=world_with_laser_body.root
    )
    laser.latest_scan = scan

    expected = LaserScanToSemDTConverter.convert(scan, world_with_laser_body)
    reading = laser.get_laser_reading()

    assert reading.distance == expected.distance
    assert [direction.to_np().tolist() for direction in reading.direction] == [
        direction.to_np().tolist() for direction in expected.direction
    ]


def test_subscribed_laser_takes_its_scan_pattern_from_its_latest_scan(
    rclpy_node, world_with_laser_body
):
    scan = laser_scan()
    laser = SubscribedLaser(
        node=rclpy_node, topic_name="/scan", root=world_with_laser_body.root
    )
    laser.latest_scan = scan

    assert laser.scan_pattern.minimum_angle == scan.angle_min
    assert laser.scan_pattern.maximum_angle == scan.angle_max
    assert laser.scan_pattern.angle_increment == scan.angle_increment
    assert laser.scan_pattern.minimum_range == scan.range_min
    assert laser.scan_pattern.maximum_range == scan.range_max


def test_subscribed_laser_without_a_scan_cannot_be_read(
    rclpy_node, world_with_laser_body
):
    laser = SubscribedLaser(
        node=rclpy_node, topic_name="/scan", root=world_with_laser_body.root
    )

    with pytest.raises(NoLaserScanReceived):
        laser.get_laser_reading()
