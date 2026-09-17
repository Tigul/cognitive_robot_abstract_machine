from __future__ import annotations

import pytest

from cramph.composites import Sequence
from cramph.context import StatechartContext
from cramph.data_types import LifeCycleValues, ObservationStateValues
from cramph.exceptions import TickDurationUnknownError
from cramph.executor import SimulationPacer, StatechartExecutor
from cramph.monitors import CountSimulationTimeSeconds, CountTicks
from cramph.node import CancelStatechart, EndStatechart, NodeArtifacts
from cramph.nodes_for_testing import ConstTrueNode, NodeAssertionError
from cramph.statechart import Statechart

# %% ticking generic nodes


def test_generic_nodes_tick_until_the_statechart_ends(
    statechart_executor: StatechartExecutor,
):
    first_step = CountTicks(ticks=2)
    second_step = CountTicks(ticks=3)
    sequence = Sequence(nodes=[first_step, second_step])
    statechart = Statechart()
    statechart.add_node(sequence)
    statechart.add_node(EndStatechart.when_true(sequence))

    statechart_executor.compile(statechart)
    statechart_executor.tick_until_end()

    assert statechart.is_ended()
    assert sequence.life_cycle_state == LifeCycleValues.SUCCEEDED
    # +1 for EndStatechart to observe True
    assert statechart_executor.tick_count == first_step.ticks + second_step.ticks + 1


def test_end_statechart_ends_the_tick_after_it_starts(
    statechart_executor: StatechartExecutor,
):
    node = ConstTrueNode()
    end = EndStatechart.when_true(node)
    statechart = Statechart()
    statechart.add_nodes([node, end])
    statechart_executor.compile(statechart)

    statechart_executor.tick()
    assert end.life_cycle_state == LifeCycleValues.RUNNING
    assert not statechart.is_ended()

    statechart_executor.tick()
    assert statechart.is_ended()


def test_cancel_statechart_raises_its_exception(
    statechart_executor: StatechartExecutor,
):
    node = ConstTrueNode()
    exception = NodeAssertionError(reason="cancelled")
    statechart = Statechart()
    statechart.add_nodes([node, CancelStatechart.when_true(node, exception)])
    statechart_executor.compile(statechart)

    with pytest.raises(NodeAssertionError) as raised:
        statechart_executor.tick()

    assert raised.value is exception


def test_generic_nodes_build_plain_node_artifacts(
    statechart_executor: StatechartExecutor,
):
    node = ConstTrueNode()
    statechart = Statechart()
    statechart.add_nodes([node, EndStatechart.when_true(node)])

    artifacts = node.build(statechart_executor.context)

    assert type(artifacts) is NodeArtifacts


# %% tick duration


def test_simulation_time_is_counted_in_tick_durations(
    statechart_executor: StatechartExecutor,
):
    ticks = 3
    tick_duration = statechart_executor.context.tick_duration
    counter = CountSimulationTimeSeconds(seconds=ticks * tick_duration)
    statechart = Statechart()
    statechart.add_nodes([counter, EndStatechart.when_true(counter)])
    statechart_executor.compile(statechart)

    for _ in range(ticks - 1):
        statechart_executor.tick()
    assert counter.observation_state == ObservationStateValues.FALSE

    statechart_executor.tick()
    assert counter.observation_state == ObservationStateValues.TRUE
    assert statechart_executor.time == ticks * tick_duration


def test_a_context_does_not_know_its_tick_duration_by_default(
    statechart_context_without_tick_duration: StatechartContext,
):
    assert statechart_context_without_tick_duration.tick_duration is None


def test_a_statechart_ticks_without_knowing_its_tick_duration(
    statechart_context_without_tick_duration: StatechartContext,
):
    executor = StatechartExecutor(context=statechart_context_without_tick_duration)
    counter = CountTicks(ticks=2)
    statechart = Statechart()
    statechart.add_nodes([counter, EndStatechart.when_true(counter)])
    executor.compile(statechart)

    executor.tick_until_end()

    assert statechart.is_ended()


def test_time_is_unknown_without_a_tick_duration(
    statechart_context_without_tick_duration: StatechartContext,
):
    executor = StatechartExecutor(context=statechart_context_without_tick_duration)

    with pytest.raises(TickDurationUnknownError):
        executor.time


def test_simulation_time_cannot_be_counted_without_a_tick_duration(
    statechart_context_without_tick_duration: StatechartContext,
):
    executor = StatechartExecutor(context=statechart_context_without_tick_duration)
    counter = CountSimulationTimeSeconds(seconds=1.0)
    statechart = Statechart()
    statechart.add_nodes([counter, EndStatechart.when_true(counter)])
    executor.compile(statechart)

    with pytest.raises(TickDurationUnknownError):
        executor.tick()


def test_a_simulation_cannot_be_paced_without_a_tick_duration(
    statechart_context_without_tick_duration: StatechartContext,
):
    with pytest.raises(TickDurationUnknownError):
        StatechartExecutor(
            context=statechart_context_without_tick_duration,
            pacer=SimulationPacer(),
        )
