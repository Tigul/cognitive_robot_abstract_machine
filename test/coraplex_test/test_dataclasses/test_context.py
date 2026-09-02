import logging
from copy import deepcopy

import pytest

from coraplex.datastructures.dataclasses import Context
from krrood.utils import detach_memoization_cache

# %% debug validation


def test_debug_requires_a_ros_node(immutable_model_world):
    """
    Debug output is visualized over ROS, so a context constructed in debug mode without
    a node is rejected at construction rather than failing later during execution.
    """
    world, robot, _ = immutable_model_world

    with pytest.raises(ValueError):
        Context(world, robot, _debug=True)


def test_debug_raises_the_coraplex_log_level(immutable_model_world, rclpy_node):
    """
    Constructing a context in debug mode lowers the package's log level, so debug
    messages are emitted without the caller touching logging.
    """
    world, robot, _ = immutable_model_world
    coraplex_logger = logging.getLogger("coraplex")
    previous_level = coraplex_logger.level

    try:
        Context(world, robot, ros_node=rclpy_node, _debug=True)
        assert coraplex_logger.level == logging.DEBUG
    finally:
        coraplex_logger.setLevel(previous_level)


def test_default_context_logs_at_info(immutable_model_world):
    """
    Without debug mode the package logs at info level.
    """
    world, robot, _ = immutable_model_world
    coraplex_logger = logging.getLogger("coraplex")
    previous_level = coraplex_logger.level

    try:
        context = Context(world, robot)
        assert not context.debug
        assert coraplex_logger.level == logging.INFO
    finally:
        coraplex_logger.setLevel(previous_level)


# %% copying onto another world


def test_copy_for_other_world_resolves_the_robot_in_that_world(immutable_model_world):
    """
    A copy of the world holds its own robot, carrying the same id, so the copied context
    must reach that one rather than the robot it was built with.
    """
    world, robot, context = immutable_model_world
    other_world = deepcopy(world)

    copied_context = context.copy_for_other_world(other_world)

    assert copied_context.world is other_world
    assert copied_context.robot is not robot
    assert copied_context.robot.id == robot.id
    assert copied_context.robot is other_world.get_semantic_annotation_by_id(robot.id)


def test_copy_for_other_world_carries_the_remaining_settings_over(
    immutable_model_world,
):
    """
    Only the world and the robot differ; everything that configures how the context
    executes stays as it was.
    """
    world, robot, context = immutable_model_world
    context.evaluate_conditions = False

    copied_context = context.copy_for_other_world(deepcopy(world))

    assert copied_context.evaluate_conditions == context.evaluate_conditions
    assert copied_context.query_backend is context.query_backend
    assert copied_context.motion_tolerances is context.motion_tolerances


def test_copy_for_other_world_gets_its_own_memoization_cache(immutable_model_world):
    """
    The copy memoizes values built for its own world, and the giskard wrapper among them
    holds that world.

    A shared cache would keep it reachable from this context for as long as the context
    lives.
    """
    world, robot, context = immutable_model_world
    detach_memoization_cache(context)

    copied_context = context.copy_for_other_world(deepcopy(world))

    assert copied_context.__memo__ is not context.__memo__
