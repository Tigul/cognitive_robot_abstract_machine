import logging

import pytest

from coraplex.datastructures.dataclasses import Context

from ...conftest import SAMPLING_SEED

# %% debug validation


def test_debug_requires_a_ros_node(pr2_apartment_context):
    """
    Debug output is visualized over ROS, so a context constructed in debug mode without
    a node is rejected at construction rather than failing later during execution.
    """
    world, robot, _ = pr2_apartment_context

    with pytest.raises(ValueError):
        Context(world, robot, _debug=True)


def test_debug_raises_the_coraplex_log_level(pr2_apartment_context, rclpy_node):
    """
    Constructing a context in debug mode lowers the package's log level, so debug
    messages are emitted without the caller touching logging.
    """
    world, robot, _ = pr2_apartment_context
    coraplex_logger = logging.getLogger("coraplex")
    previous_level = coraplex_logger.level

    try:
        Context(world, robot, ros_node=rclpy_node, _debug=True)
        assert coraplex_logger.level == logging.DEBUG
    finally:
        coraplex_logger.setLevel(previous_level)


def test_default_context_logs_at_info(pr2_apartment_context):
    """
    Without debug mode the package logs at info level.
    """
    world, robot, _ = pr2_apartment_context
    coraplex_logger = logging.getLogger("coraplex")
    previous_level = coraplex_logger.level

    try:
        context = Context(world, robot)
        assert not context.debug
        assert coraplex_logger.level == logging.INFO
    finally:
        coraplex_logger.setLevel(previous_level)


# %% the root Cartesian goals are expressed in


def test_controlled_root_is_the_robot_when_the_base_stands_still(pr2_apartment_context):
    """
    A robot that keeps its base still can only move what hangs below its own root, so
    that is what a Cartesian goal is expressed relative to.
    """
    world, robot, _ = pr2_apartment_context
    robot.mobile_base.full_body_controlled = False

    assert Context(world, robot).controlled_root is robot.root


def test_controlled_root_is_the_world_when_the_base_may_drive(pr2_apartment_context):
    """
    A robot that drives its base while it manipulates moves relative to the world, so
    the world root is what a Cartesian goal is expressed relative to.
    """
    world, robot, _ = pr2_apartment_context
    previous = robot.mobile_base.full_body_controlled
    robot.mobile_base.full_body_controlled = True

    try:
        assert Context(world, robot).controlled_root is world.root
    finally:
        robot.mobile_base.full_body_controlled = previous


# %% repeatable location samples

WORLD_FIXTURES_WITH_A_CONTEXT = [
    "pr2_apartment_context",
    "simple_pr2_context",
    "stretch_apartment_context",
    "apartment_world_pr2_copy_with_context",
]
"""
The shared fixtures that hand a test a plan context to run its actions in.
"""


@pytest.mark.parametrize("world_fixture", WORLD_FIXTURES_WITH_A_CONTEXT)
def test_a_shared_fixture_fixes_the_samples_its_context_makes(world_fixture, request):
    """
    A location samples its candidates from a costmap rather than ranking it, so a test
    handed an unseeded context would stand somewhere else every run.
    """
    _, _, context = request.getfixturevalue(world_fixture)

    assert context.sampling_seed == SAMPLING_SEED
