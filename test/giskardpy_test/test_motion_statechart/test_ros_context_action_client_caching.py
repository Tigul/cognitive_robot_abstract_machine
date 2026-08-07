from __future__ import annotations

from dataclasses import dataclass, field

import pytest
from typing_extensions import Any, Type

import giskardpy.motion_statechart.ros_context as ros_context_module
from giskardpy.motion_statechart.context import MotionStatechartContext
from giskardpy.motion_statechart.exceptions import ActionClientTypeMismatchError
from giskardpy.motion_statechart.ros2_nodes.ros_tasks import ActionServerTask
from giskardpy.motion_statechart.ros_context import RosContextExtension
from semantic_digital_twin.world import World

# %% test doubles


class FakeActionType:
    """A distinct action message type, standing in for a real generated ROS action type."""


class OtherFakeActionType:
    """A second, distinct action message type, used to exercise the mismatch guard."""


class FakeFuture:
    """
    Stand-in for ``rclpy.task.Future`` whose result is already known and whose
    done-callback fires synchronously, avoiding a real executor/event loop.
    """

    def __init__(self, result):
        self._result = result

    def result(self):
        return self._result

    def add_done_callback(self, callback):
        callback(self)


class FakeGoalHandle:
    """Stand-in for the goal handle rclpy hands back once a goal is accepted."""

    def __init__(self, accepted: bool, result):
        self.accepted = accepted
        self._result = result

    def get_result_async(self) -> FakeFuture:
        return FakeFuture(self._result)


class FakeActionClient:
    """
    Stand-in for ``rclpy.action.ActionClient`` that avoids real ROS network I/O.

    Mirrors the constructor signature and the ``_action_type``/``wait_for_server``/
    ``send_goal_async`` surface that :class:`RosContextExtension` and
    :class:`ActionServerTask` rely on. Each call to :meth:`send_goal_async` tracks its
    own goal independently (mirroring rclpy's per-goal futures), which is what allows
    several tasks to safely share one cached client.
    """

    construction_count: int = 0

    def __init__(self, node, action_type, action_name):
        FakeActionClient.construction_count += 1
        self.node = node
        self._action_type = action_type
        self.action_name = action_name

    def wait_for_server(self) -> bool:
        return True

    def send_goal_async(self, goal) -> FakeFuture:
        # Echoes the goal back as the eventual result, so a test can tell which
        # task's goal actually produced a given result.
        return FakeFuture(FakeGoalHandle(accepted=True, result=goal))


@dataclass(eq=False, repr=False)
class RecordingActionServerTask(ActionServerTask):
    """An ``ActionServerTask`` whose goal message is a settable payload, used to
    exercise ``build()``/``on_start()``."""

    goal_payload: Any = None
    """The value ``build_msg`` copies into ``self._msg``, settable per test."""

    def build_msg(self, context: MotionStatechartContext):
        self._msg = self.goal_payload


@pytest.fixture(autouse=True)
def fake_action_client(monkeypatch):
    FakeActionClient.construction_count = 0
    monkeypatch.setattr(ros_context_module, "ActionClient", FakeActionClient)


@pytest.fixture
def context() -> MotionStatechartContext:
    built_context = MotionStatechartContext(world=World())
    built_context.add_extension(RosContextExtension(ros_node=object()))
    return built_context


# %% RosContextExtension.get_or_create_action_client


def test_get_or_create_action_client_reuses_client_for_same_topic(context):
    ros_context_extension = context.require_extension(RosContextExtension)

    first = ros_context_extension.get_or_create_action_client(
        FakeActionType, "my_action"
    )
    second = ros_context_extension.get_or_create_action_client(
        FakeActionType, "my_action"
    )

    assert first is second
    assert FakeActionClient.construction_count == 1


def test_get_or_create_action_client_creates_separate_clients_for_different_topics(
    context,
):
    ros_context_extension = context.require_extension(RosContextExtension)

    first = ros_context_extension.get_or_create_action_client(
        FakeActionType, "action_a"
    )
    second = ros_context_extension.get_or_create_action_client(
        FakeActionType, "action_b"
    )

    assert first is not second
    assert FakeActionClient.construction_count == 2


def test_get_or_create_action_client_raises_on_message_type_mismatch(context):
    ros_context_extension = context.require_extension(RosContextExtension)
    ros_context_extension.get_or_create_action_client(FakeActionType, "my_action")

    with pytest.raises(ActionClientTypeMismatchError):
        ros_context_extension.get_or_create_action_client(
            OtherFakeActionType, "my_action"
        )


# %% ActionServerTask.build


def test_action_server_task_build_reuses_cached_action_client(context):
    task_a = RecordingActionServerTask(
        action_topic="my_action", message_type=FakeActionType
    )
    task_b = RecordingActionServerTask(
        action_topic="my_action", message_type=FakeActionType
    )

    task_a.build(context)
    task_b.build(context)

    assert task_a._action_client is task_b._action_client
    assert FakeActionClient.construction_count == 1


def test_two_tasks_sharing_a_cached_action_client_track_independent_goals(context):
    """
    Two tasks that target the same action topic share one cached action client, but
    each ``send_goal_async``/``get_result_async`` call returns its own future (mirroring
    real rclpy), so each task's own goal and result stay isolated even when both are
    active at once.
    """
    task_a = RecordingActionServerTask(
        action_topic="my_action", message_type=FakeActionType, goal_payload="goal_a"
    )
    task_b = RecordingActionServerTask(
        action_topic="my_action", message_type=FakeActionType, goal_payload="goal_b"
    )
    task_a.build(context)
    task_b.build(context)
    assert task_a._action_client is task_b._action_client

    # Interleaved, as if both tasks were running in the same control cycle.
    task_a.on_start(context)
    task_b.on_start(context)

    assert task_a._result == "goal_a"
    assert task_b._result == "goal_b"
