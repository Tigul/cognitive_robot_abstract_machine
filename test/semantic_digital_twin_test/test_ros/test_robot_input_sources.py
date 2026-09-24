"""
Tests for reading the parts of a robot from what the real robot publishes.
"""

from __future__ import annotations

import pytest

from semantic_digital_twin.adapters.ros.input_synchronization import (
    PendingJointPositionSource,
    SubscribedBasePoseSource,
)
from semantic_digital_twin.adapters.sensors.lidar import SimulatedLidarSource
from semantic_digital_twin.robots.input_source import RobotTopic
from semantic_digital_twin.robots.pr2 import PR2, PR2Topic
from semantic_digital_twin.robots.robot_parts import KinematicChain, MobileBase
from semantic_digital_twin.world import World


@pytest.fixture
def annotated_pr2(pr2_world_copy: World) -> PR2:
    """
    The PR2 annotated in a world of its own, which the tests below switch around.
    """
    return pr2_world_copy.get_semantic_annotations_by_type(PR2)[0]


def test_switching_a_robot_reads_every_part_declaring_a_topic_from_the_robot(
    annotated_pr2, rclpy_node
):
    annotated_pr2.use_real_sources(rclpy_node)

    assert isinstance(annotated_pr2.left_arm.source, PendingJointPositionSource)
    assert isinstance(annotated_pr2.mobile_base.source, SubscribedBasePoseSource)


def test_a_part_declaring_no_topic_keeps_reading_the_world_of_a_switched_robot(
    annotated_pr2, rclpy_node
):
    annotated_pr2.use_real_sources(rclpy_node)

    assert isinstance(annotated_pr2.mobile_base.lidar.source, SimulatedLidarSource)


def test_a_switched_robot_reads_the_topics_its_parts_declare(annotated_pr2, rclpy_node):
    annotated_pr2.use_real_sources(rclpy_node)

    assert annotated_pr2.left_arm.source.topic_name == f"/{RobotTopic.JOINT_STATES}"
    assert annotated_pr2.mobile_base.source.topic_name == f"/{PR2Topic.ODOMETRY}"


def test_a_switched_chain_writes_only_its_own_joints(annotated_pr2, rclpy_node):
    annotated_pr2.use_real_sources(rclpy_node)

    assert (
        annotated_pr2.left_arm.source.connections
        == annotated_pr2.left_arm.active_connections
    )


def test_every_part_read_from_the_robot_is_an_input_to_apply(annotated_pr2, rclpy_node):
    switched_parts = [
        part
        for part in annotated_pr2._robot_parts
        if isinstance(part, (KinematicChain, MobileBase))
    ]
    annotated_pr2.use_real_sources(rclpy_node)

    synchronizers = annotated_pr2.get_input_synchronizers()

    assert len(synchronizers) == len(switched_parts)
    assert all(
        any(synchronizer is part.source for synchronizer in synchronizers)
        for part in switched_parts
    )
