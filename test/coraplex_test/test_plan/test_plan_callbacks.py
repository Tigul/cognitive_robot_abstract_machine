"""
Plan callbacks follow the start and the end of every node of a plan, read off the
snapshots its statechart records.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import StrEnum

import pytest

from coraplex.exceptions import MotionDidNotFinish
from coraplex.execution_environment import simulated_robot
from coraplex.plans.plan_callbacks import PlanCallback, PlanCallbackDispatcher
from coraplex.plans.plan_execution import PlanExecutor
from coraplex.robot_plans.actions.core.robot_body import MoveTorsoAction
from cramph.composites import Sequence
from cramph.context import StatechartContext
from cramph.data_types import LifeCycleValues, ObservationStateValues
from cramph.node import StatechartNode
from cramph.nodes_for_testing import ConstTrueNode, NodeFailingOnObservingFalse
from cramph.statechart import StateHistoryItem, Statechart
from giskardpy.motion_statechart.tasks.joint_tasks import JointPositionList
from semantic_digital_twin.datastructures.definitions import TorsoState
from semantic_digital_twin.world import World

# %% observer records


class ExecutionEvent(StrEnum):
    """
    What a callback was told about a node.
    """

    START = "start"
    END = "end"


@dataclass
class NodeEvent:
    """
    One report a callback received.
    """

    kind: ExecutionEvent
    """
    Whether the node started or ended.
    """

    node: StatechartNode
    """
    The node reported.
    """

    life_cycle_state: LifeCycleValues
    """
    The life cycle state the node was in when it was reported.
    """


@dataclass
class ExecutionRecorder(PlanCallback):
    """
    Records every report, in order.
    """

    compiled: list[StatechartNode] = field(default_factory=list)
    """
    The plans reported as compiled.
    """

    events: list[NodeEvent] = field(default_factory=list)
    """
    The starts and ends reported.
    """

    def on_compile(self, plan: StatechartNode, statechart: Statechart) -> None:
        self.compiled.append(plan)

    def on_start(self, node: StatechartNode) -> None:
        self.events.append(NodeEvent(ExecutionEvent.START, node, node.life_cycle_state))

    def on_end(self, node: StatechartNode) -> None:
        self.events.append(NodeEvent(ExecutionEvent.END, node, node.life_cycle_state))

    def events_of(self, node: StatechartNode) -> list[NodeEvent]:
        """
        :return: The reports about `node`, in order.
        """
        return [event for event in self.events if event.node is node]


@dataclass
class RecordedNode:
    """
    A node of a plan whose life cycle state a test sets snapshot by snapshot.
    """

    statechart: Statechart
    """
    The statechart holding :attr:`node`.
    """

    node: StatechartNode
    """
    The node whose state is set.
    """

    def record(self, state: LifeCycleValues) -> None:
        """
        Put :attr:`node` into `state` and record a snapshot.

        :param state: The life cycle state of the snapshot.
        """
        self.statechart.life_cycle_state[self.node] = state
        self.statechart.history.append(
            StateHistoryItem(
                tick_count=len(self.statechart.history),
                life_cycle_state=self.statechart.life_cycle_state,
                observation_state=self.statechart.observation_state,
            )
        )


@pytest.fixture
def tracked_node() -> tuple[RecordedNode, ExecutionRecorder]:
    """
    A plan of one node, observed by a recorder through a dispatcher.
    """
    statechart = Statechart(context=StatechartContext(world=World()))
    node = ConstTrueNode()
    statechart.add_node(node)
    recorder = ExecutionRecorder()
    statechart.history.add_observer(
        PlanCallbackDispatcher(plan=node, callbacks=[recorder])
    )
    return RecordedNode(statechart=statechart, node=node), recorder


# %% node life cycle


@pytest.mark.parametrize("terminal", list(LifeCycleValues.terminal_states()))
def test_a_terminal_node_reports_its_start_and_exact_outcome(
    terminal, tracked_node
) -> None:
    """
    A node that ends in the tick it starts in still starts before it ends.
    """
    recorded, recorder = tracked_node

    recorded.record(LifeCycleValues.NOT_STARTED)
    assert recorder.events == []
    recorded.record(terminal)
    recorded.record(terminal)

    assert recorder.events == [
        NodeEvent(ExecutionEvent.START, recorded.node, terminal),
        NodeEvent(ExecutionEvent.END, recorded.node, terminal),
    ]


def test_pausing_and_resuming_does_not_repeat_the_start(tracked_node) -> None:
    recorded, recorder = tracked_node

    for state in (
        LifeCycleValues.RUNNING,
        LifeCycleValues.PAUSED,
        LifeCycleValues.RUNNING,
    ):
        recorded.record(state)

    assert recorder.events == [
        NodeEvent(ExecutionEvent.START, recorded.node, LifeCycleValues.RUNNING)
    ]


def test_a_reset_starts_a_new_execution(tracked_node) -> None:
    recorded, recorder = tracked_node

    for state in (
        LifeCycleValues.RUNNING,
        LifeCycleValues.SUCCEEDED,
        LifeCycleValues.NOT_STARTED,
        LifeCycleValues.RUNNING,
        LifeCycleValues.INTERRUPTED,
    ):
        recorded.record(state)

    assert [event.kind for event in recorder.events] == [
        ExecutionEvent.START,
        ExecutionEvent.END,
        ExecutionEvent.START,
        ExecutionEvent.END,
    ]
    assert recorder.events[-1].life_cycle_state is LifeCycleValues.INTERRUPTED


def test_the_default_callback_accepts_every_event(tracked_node) -> None:
    """
    A callback reacts only to the events it cares about.
    """
    recorded, _ = tracked_node
    recorded.statechart.history.add_observer(
        PlanCallbackDispatcher(plan=recorded.node, callbacks=[PlanCallback()])
    )

    recorded.record(LifeCycleValues.RUNNING)
    recorded.record(LifeCycleValues.SUCCEEDED)


# %% executed plans


def test_an_executed_plan_reports_its_compile_and_every_node_once(
    pr2_apartment_context,
) -> None:
    world, robot, context = pr2_apartment_context
    torso = MoveTorsoAction(TorsoState.HIGH)
    plan = Sequence([torso])
    recorder = ExecutionRecorder()
    executor = PlanExecutor(context, callbacks=[recorder])

    with simulated_robot:
        executor.compile(plan)
        executor.execute()

    assert recorder.compiled == [plan]
    [joint_goal] = plan.statechart.get_nodes_by_type(JointPositionList)
    for node in (plan, torso, joint_goal):
        assert [event.kind for event in recorder.events_of(node)] == [
            ExecutionEvent.START,
            ExecutionEvent.END,
        ]
    assert recorder.events_of(plan)[-1].life_cycle_state is LifeCycleValues.SUCCEEDED


def test_a_failed_plan_reports_its_failure(pr2_apartment_context) -> None:
    world, robot, context = pr2_apartment_context
    plan = NodeFailingOnObservingFalse(observation=ObservationStateValues.FALSE)
    recorder = ExecutionRecorder()
    executor = PlanExecutor(context, callbacks=[recorder])

    with simulated_robot, pytest.raises(MotionDidNotFinish):
        executor.compile(plan)
        executor.execute()

    assert recorder.events_of(plan)[-1] == NodeEvent(
        ExecutionEvent.END, plan, LifeCycleValues.FAILED
    )
