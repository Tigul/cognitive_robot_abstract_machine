from typing_extensions import Callable, List, Optional

import pytest

from giskardpy.executor import Executor
from giskardpy.motion_statechart.context import MotionStatechartContext
from giskardpy.motion_statechart.data_types import (
    LifeCycleValues,
    ObservationStateValues,
)
from giskardpy.motion_statechart.goals.templates import Parallel, Sequence
from giskardpy.motion_statechart.graph_node import (
    CancelMotion,
    EndMotion,
    MotionStatechartNode,
)
from giskardpy.motion_statechart.monitors.payload_monitors import Pulse
from giskardpy.motion_statechart.exceptions import ControlCycleDoesNotSettleError
from giskardpy.motion_statechart.motion_statechart import (
    CompiledControlCycle,
    MotionStatechart,
)
from giskardpy.motion_statechart.nodes_for_testing.nodes_for_testing import (
    CompositeStatechartNodeObservingItsCancelMotionRun,
    CompositeStatechartNodeObservingItsSecondChildRun,
    CompositeStatechartNodeWithARecordingChild,
    ConstFalseNode,
    ConstTrueNode,
    LifeCycleCallback,
    NodeObservingAPredicate,
    NodeObservingAWrittenVariable,
    NodeObservingTheOppositeOfAnObservationPredicate,
    NodeObservingTrueOnlyOnTick,
    NodeRecordingItsCallbacks,
    NodeWritingAVariableOnStart,
    TestNodeAssertionError,
)
from krrood.symbolic_math.symbolic_math import Scalar
from semantic_digital_twin.world import World

CYCLES_TO_WATCH = 8
"""
How many control cycles a test ticks through, enough for every chart here to settle.
"""

DEEP_NESTING = 30
"""
How many sequences are nested around one task in the deepest statechart here, more than
a fixed number of passes per control cycle would settle.
"""


def _compile(motion_statechart: MotionStatechart) -> Executor:
    """
    :param motion_statechart: The statechart to run.
    :return: An executor that compiled `motion_statechart`, which already ticked once.
    """
    executor = Executor(MotionStatechartContext(world=World()))
    executor.compile(motion_statechart=motion_statechart)
    return executor


def _tick_until(executor: Executor, happened: Callable[[], bool]) -> None:
    """
    Ticks `executor` until `happened` is true, which it may already be.

    :param executor: The executor to tick.
    :param happened: Whether the awaited event has happened by now.
    """
    for _ in range(CYCLES_TO_WATCH):
        if happened():
            return
        executor.tick()
    assert happened()


def _first_cycles(
    executor: Executor, events: List[Callable[[], bool]]
) -> List[Optional[int]]:
    """
    :param executor: The executor to tick, whose compile tick counts as cycle 0.
    :param events: Whether each awaited event has happened by now.
    :return: Per event, the first control cycle after which it had happened, or None if
        it did not within :data:`CYCLES_TO_WATCH`.
    """
    first_cycles: List[Optional[int]] = [None] * len(events)
    for control_cycle in range(CYCLES_TO_WATCH + 1):
        if control_cycle > 0:
            executor.tick()
        for index, happened in enumerate(events):
            if first_cycles[index] is None and happened():
                first_cycles[index] = control_cycle
    return first_cycles


def _nested_sequence_chart() -> MotionStatechart:
    """
    :return: A statechart whose only step finishes two composite levels above its task,
        so finishing it takes more than one pass through the statechart.
    """
    motion_statechart = MotionStatechart()
    motion_statechart.add_node(Sequence(nodes=[Sequence(nodes=[ConstTrueNode()])]))
    return motion_statechart


# %% reaction time does not depend on nesting


def _bare(task: ConstTrueNode) -> MotionStatechartNode:
    task.success_condition = task.observes_true
    return task


def _in_a_sequence(task: ConstTrueNode) -> MotionStatechartNode:
    return Sequence(nodes=[task])


def _in_nested_sequences(task: ConstTrueNode) -> MotionStatechartNode:
    return Sequence(nodes=[Sequence(nodes=[Sequence(nodes=[task])])])


class TestReactionTimeAcrossNesting:
    """
    A node waiting for a step reacts on the cycle the step's task reaches its goal,
    however many composite levels lie between the two.
    """

    @pytest.mark.parametrize(
        "make_step",
        [_bare, _in_a_sequence, _in_nested_sequences],
        ids=["bare task", "task in a sequence", "task in nested sequences"],
    )
    def test_a_node_waiting_on_a_step_starts_on_the_cycle_its_task_reaches_its_goal(
        self, make_step: Callable[[ConstTrueNode], MotionStatechartNode]
    ):
        motion_statechart = MotionStatechart()
        task = ConstTrueNode()
        step = make_step(task)
        waiting = ConstFalseNode()
        motion_statechart.add_nodes([step, waiting])
        waiting.start_condition = step.is_succeeded
        executor = _compile(motion_statechart)

        goal_reached_cycle, started_cycle = _first_cycles(
            executor,
            [
                lambda: motion_statechart.observation_state[task]
                == ObservationStateValues.TRUE,
                lambda: waiting.life_cycle_state == LifeCycleValues.RUNNING,
            ],
        )

        assert started_cycle == goal_reached_cycle

    @pytest.mark.parametrize(
        "make_step",
        [_bare, _in_a_sequence, _in_nested_sequences],
        ids=["bare task", "task in a sequence", "task in nested sequences"],
    )
    def test_the_motion_ends_on_the_cycle_after_a_step_reaches_its_goal(
        self, make_step: Callable[[ConstTrueNode], MotionStatechartNode]
    ):
        """
        The end motion node starts on the cycle the step's task reaches its goal and,
        like any node started during a cycle, first observes on the next one.
        """
        motion_statechart = MotionStatechart()
        task = ConstTrueNode()
        step = make_step(task)
        motion_statechart.add_nodes([step, EndMotion.when_true(step)])
        executor = _compile(motion_statechart)

        goal_reached_cycle, end_cycle = _first_cycles(
            executor,
            [
                lambda: motion_statechart.observation_state[task]
                == ObservationStateValues.TRUE,
                motion_statechart.is_end_motion,
            ],
        )

        assert end_cycle == goal_reached_cycle + 1

    def test_a_parent_reads_the_outcome_its_child_reaches_on_the_same_cycle(self):
        motion_statechart = MotionStatechart()
        child = ConstTrueNode()
        parallel = Parallel([child])
        motion_statechart.add_node(parallel)
        child.success_condition = child.observes_true
        parallel.success_condition = child.is_succeeded
        executor = _compile(motion_statechart)

        _tick_until(
            executor, lambda: child.life_cycle_state == LifeCycleValues.SUCCEEDED
        )

        assert parallel.life_cycle_state == LifeCycleValues.SUCCEEDED

    def test_a_parent_reads_what_its_child_observes_on_the_same_cycle(self):
        motion_statechart = MotionStatechart()
        child = ConstTrueNode()
        parallel = Parallel([child])
        motion_statechart.add_node(parallel)
        parallel.success_condition = child.observes_true
        executor = _compile(motion_statechart)

        _tick_until(
            executor,
            lambda: motion_statechart.observation_state[child]
            == ObservationStateValues.TRUE,
        )

        assert parallel.life_cycle_state == LifeCycleValues.SUCCEEDED

    def test_an_observation_reads_the_outcome_another_node_reaches_on_the_same_cycle(
        self,
    ):
        motion_statechart = MotionStatechart()
        watched = ConstTrueNode()
        observer = NodeObservingAPredicate(watched_node=watched)
        motion_statechart.add_nodes([watched, observer])
        watched.success_condition = watched.observes_true
        executor = _compile(motion_statechart)

        _tick_until(
            executor, lambda: watched.life_cycle_state == LifeCycleValues.SUCCEEDED
        )

        assert (
            motion_statechart.observation_state[observer] == ObservationStateValues.TRUE
        )


# %% what is decided once per control cycle


class TestOncePerControlCycle:
    """
    Python code on a node runs once per control cycle, however often the statechart has
    to be evaluated before that cycle settles.
    """

    def test_on_tick_is_called_once_per_control_cycle(self):
        motion_statechart = _nested_sequence_chart()
        motion_statechart.add_node(ticked := NodeObservingTrueOnlyOnTick())
        executor = _compile(motion_statechart)

        for _ in range(CYCLES_TO_WATCH):
            executor.tick()

        assert ticked.on_tick_calls == CYCLES_TO_WATCH

    def test_what_on_tick_returns_is_the_observation_a_settled_cycle_ends_with(self):
        motion_statechart = _nested_sequence_chart()
        motion_statechart.add_node(ticked := NodeObservingTrueOnlyOnTick())
        executor = _compile(motion_statechart)

        observations = []
        for _ in range(CYCLES_TO_WATCH):
            executor.tick()
            observations.append(motion_statechart.observation_state[ticked])

        assert observations == [ObservationStateValues.TRUE] * CYCLES_TO_WATCH

    def test_a_node_started_this_cycle_first_observes_on_the_next_one(self):
        motion_statechart = MotionStatechart()
        motion_statechart.add_nodes(
            [trigger := ConstTrueNode(), started_late := ConstTrueNode()]
        )
        started_late.start_condition = trigger.observes_true
        executor = _compile(motion_statechart)

        executor.tick()
        assert started_late.life_cycle_state == LifeCycleValues.RUNNING
        assert (
            motion_statechart.observation_state[started_late]
            == ObservationStateValues.UNKNOWN
        )

        executor.tick()
        assert (
            motion_statechart.observation_state[started_late]
            == ObservationStateValues.TRUE
        )

    def test_what_a_start_callback_writes_is_observed_from_the_next_cycle(self):
        motion_statechart = MotionStatechart()
        writer = NodeWritingAVariableOnStart()
        reader = NodeObservingAWrittenVariable(writer=writer)
        motion_statechart.add_nodes([trigger := ConstTrueNode(), writer, reader])
        writer.start_condition = trigger.observes_true
        executor = _compile(motion_statechart)

        executor.tick()
        assert writer.life_cycle_state == LifeCycleValues.RUNNING
        assert (
            motion_statechart.observation_state[reader] == ObservationStateValues.FALSE
        )

        executor.tick()
        assert (
            motion_statechart.observation_state[reader] == ObservationStateValues.TRUE
        )


# %% control cycles that never settle


class TestUnsettledControlCycle:
    """
    A control cycle whose passes keep changing the statechart is stopped instead of
    blocking the control loop.
    """

    def test_observations_contradicting_each_other_stop_the_control_cycle(self):
        motion_statechart = MotionStatechart()
        first = NodeObservingTheOppositeOfAnObservationPredicate()
        second = NodeObservingTheOppositeOfAnObservationPredicate(watched_node=first)
        first.watched_node = second
        motion_statechart.add_nodes([first, second])
        executor = _compile(motion_statechart)

        with pytest.raises(ControlCycleDoesNotSettleError) as error:
            executor.tick()

        assert error.value.pass_limit == CompiledControlCycle.pass_limit
        assert error.value.unsettled_nodes == [first, second]

    def test_observations_contradicting_each_other_are_stopped_once_they_repeat(self):
        """
        A control cycle that returns to a state it already had can never settle, so it
        is stopped right away instead of using up the passes a control cycle may take.
        """
        motion_statechart = MotionStatechart()
        first = NodeObservingTheOppositeOfAnObservationPredicate()
        second = NodeObservingTheOppositeOfAnObservationPredicate(watched_node=first)
        first.watched_node = second
        motion_statechart.add_nodes([first, second])
        executor = _compile(motion_statechart)

        with pytest.raises(ControlCycleDoesNotSettleError) as error:
            executor.tick()

        assert error.value.passes_taken < error.value.pass_limit

    def test_deeply_nested_steps_settle_within_one_control_cycle(self):
        """
        Every nesting level may need its own passes to pass an outcome on, which is no
        reason to stop a control cycle that does settle.
        """
        motion_statechart = MotionStatechart()
        plan = ConstTrueNode()
        for _ in range(DEEP_NESTING):
            plan = Sequence(nodes=[plan])
        motion_statechart.add_node(plan)
        executor = _compile(motion_statechart)

        _tick_until(
            executor, lambda: plan.life_cycle_state == LifeCycleValues.SUCCEEDED
        )


# %% life cycle callbacks


def _callbacks_per_cycle(
    executor: Executor, node: NodeRecordingItsCallbacks
) -> List[List[LifeCycleCallback]]:
    """
    :param executor: The executor to tick, which has compiled but not ticked since.
    :param node: The node whose callbacks to record.
    :return: The callbacks run on `node` on every control cycle, starting with the
        compile tick.
    """
    callbacks = [node.take_callbacks()]
    for _ in range(CYCLES_TO_WATCH):
        executor.tick()
        callbacks.append(node.take_callbacks())
    return callbacks


class TestLifeCycleCallbacks:
    """
    Tests which callbacks run on a node whose life cycle changes more than once around
    the same control cycle.
    """

    def test_a_node_cut_off_right_after_starting_runs_its_start_then_its_end(self):
        motion_statechart = MotionStatechart()
        motion_statechart.add_node(
            composite := CompositeStatechartNodeObservingItsSecondChildRun()
        )
        composite.success_condition = composite.observes_true
        executor = _compile(motion_statechart)

        for _ in range(CYCLES_TO_WATCH):
            executor.tick()

        assert composite.second.life_cycle_state == LifeCycleValues.INTERRUPTED
        assert composite.second.take_callbacks() == [
            LifeCycleCallback.START,
            LifeCycleCallback.END,
        ]

    def test_a_cancel_motion_cut_off_right_after_starting_still_cancels_the_motion(
        self,
    ):
        motion_statechart = MotionStatechart()
        motion_statechart.add_node(
            composite := CompositeStatechartNodeObservingItsCancelMotionRun()
        )
        composite.success_condition = composite.observes_true
        executor = _compile(motion_statechart)

        with pytest.raises(TestNodeAssertionError) as error:
            for _ in range(CYCLES_TO_WATCH):
                executor.tick()

        assert error.value is composite.cancel.exception

    def test_a_cancel_motion_lets_the_control_cycle_it_starts_in_complete(self):
        """
        Cancelling the motion ends it after the control cycle, so every callback of that
        cycle still runs and the cycle is still recorded.
        """
        motion_statechart = MotionStatechart()
        trigger = ConstTrueNode()
        cancel = CancelMotion(
            exception=TestNodeAssertionError(reason="cancelled on the first goal")
        )
        ended = NodeRecordingItsCallbacks()
        motion_statechart.add_nodes([trigger, cancel, ended])
        cancel.start_condition = trigger.observes_true
        ended.interrupt_condition = trigger.observes_true
        executor = _compile(motion_statechart)

        with pytest.raises(TestNodeAssertionError) as error:
            executor.tick()

        assert error.value is cancel.exception
        assert ended.take_callbacks() == [
            LifeCycleCallback.START,
            LifeCycleCallback.END,
        ]
        last_snapshot = motion_statechart.history.history[-1]
        assert last_snapshot.control_cycle == executor.control_cycles
        assert last_snapshot.life_cycle_state[cancel] == LifeCycleValues.RUNNING

    def test_a_node_starting_while_its_pause_condition_holds_starts_paused(self):
        """
        A node that would be paused right away never runs, not even for the control
        cycle it starts in, and still gets both callbacks in order.
        """
        motion_statechart = MotionStatechart()
        motion_statechart.add_node(paused := NodeRecordingItsCallbacks())
        paused.pause_condition = Scalar.const_true()
        executor = _compile(motion_statechart)

        assert paused.life_cycle_state == LifeCycleValues.PAUSED
        assert motion_statechart.history.get_life_cycle_history_of_node(paused) == [
            LifeCycleValues.NOT_STARTED,
            LifeCycleValues.PAUSED,
        ]
        assert (
            _callbacks_per_cycle(executor, paused)
            == [[LifeCycleCallback.START, LifeCycleCallback.PAUSE]]
            + [[]] * CYCLES_TO_WATCH
        )

    def test_a_node_that_restarts_after_failing_takes_one_step_per_cycle(self):
        motion_statechart = MotionStatechart()
        motion_statechart.add_nodes(
            [trigger := ConstTrueNode(), restarting := NodeRecordingItsCallbacks()]
        )
        restarting.fail_condition = trigger.observes_true
        restarting.reset_condition = restarting.is_failed
        executor = _compile(motion_statechart)

        callbacks = _callbacks_per_cycle(executor, restarting)

        restart_loop = [
            [LifeCycleCallback.START],
            [LifeCycleCallback.END],
            [LifeCycleCallback.RESET],
        ]
        assert callbacks == (restart_loop * CYCLES_TO_WATCH)[: CYCLES_TO_WATCH + 1]

    def test_a_child_reset_by_its_parent_starts_again_only_on_the_next_cycle(self):
        motion_statechart = MotionStatechart()
        motion_statechart.add_nodes(
            [
                pulse := Pulse(),
                composite := CompositeStatechartNodeWithARecordingChild(),
            ]
        )
        composite.reset_condition = pulse.observes_true
        executor = _compile(motion_statechart)

        callbacks = _callbacks_per_cycle(executor, composite.child)

        assert callbacks == [
            [LifeCycleCallback.START],
            [LifeCycleCallback.RESET],
            [LifeCycleCallback.START],
        ] + [[]] * (CYCLES_TO_WATCH - 2)

    def test_a_child_forced_through_pause_end_and_reset_runs_each_callback_once(self):
        """
        Each ancestor of the child takes one transition of its own, triggered by what
        the level below it did on the previous pass, so the child is paused, ended and
        reset within one control cycle and only starts again on the next one.
        """
        motion_statechart = MotionStatechart()
        child = NodeRecordingItsCallbacks()
        trigger = ConstTrueNode()
        inner = Parallel([trigger, child])
        middle = Parallel([inner])
        outer = Parallel([middle])
        motion_statechart.add_node(outer)
        inner.pause_condition = trigger.observes_true
        middle.success_condition = inner.is_paused
        outer.reset_condition = middle.is_succeeded
        executor = _compile(motion_statechart)

        callbacks = _callbacks_per_cycle(executor, child)

        forced_through_and_restarted = [
            [LifeCycleCallback.PAUSE, LifeCycleCallback.END, LifeCycleCallback.RESET],
            [LifeCycleCallback.START],
        ]
        assert (
            callbacks
            == [[LifeCycleCallback.START]]
            + (forced_through_and_restarted * CYCLES_TO_WATCH)[:CYCLES_TO_WATCH]
        )

    def test_nodes_pausing_each_other_alternate_once_per_cycle(self):
        """
        Each node pauses while the other runs, which no single consistent state
        satisfies, so each node takes one step per cycle rather than the chart being
        rejected.
        """
        motion_statechart = MotionStatechart()
        motion_statechart.add_nodes(
            [
                first := NodeRecordingItsCallbacks(),
                second := NodeRecordingItsCallbacks(),
            ]
        )
        first.pause_condition = second.is_running
        second.pause_condition = first.is_running
        executor = _compile(motion_statechart)

        callbacks = _callbacks_per_cycle(executor, first)

        alternating = [[LifeCycleCallback.PAUSE], [LifeCycleCallback.UNPAUSE]]
        assert (
            callbacks
            == [[LifeCycleCallback.START]]
            + (alternating * CYCLES_TO_WATCH)[:CYCLES_TO_WATCH]
        )
