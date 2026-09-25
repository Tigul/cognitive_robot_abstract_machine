from __future__ import annotations

import pytest

from cramph.composites import Sequence
from cramph.data_types import LifeCycleValues, ObservationStateValues
from cramph.exceptions import (
    StatechartAlreadyCompiledError,
    StatechartNotCompiledError,
)
from cramph.executor import StatechartExecutor
from cramph.nodes_for_testing import NodeSucceedingOnObservingTrue
from cramph.statechart import Statechart

# %% helpers


def _node_arriving_at_once(name: str) -> NodeSucceedingOnObservingTrue:
    """
    :return: A node that succeeds on the first tick it runs.
    """
    return NodeSucceedingOnObservingTrue(
        name=name, observation=ObservationStateValues.TRUE
    )


def _compile_and_run_one_node(
    statechart_executor: StatechartExecutor,
) -> NodeSucceedingOnObservingTrue:
    """
    Compile a statechart holding one node and tick it until that node succeeded.

    :return: The node, succeeded.
    """
    statechart = Statechart(context=statechart_executor.context)
    statechart.add_node(first := _node_arriving_at_once("first"))
    statechart_executor.compile(statechart)
    statechart_executor.tick()
    assert first.life_cycle_state == LifeCycleValues.SUCCEEDED
    return first


# %% growing


def test_an_added_node_runs_once_the_node_it_waits_for_succeeded(
    statechart_executor: StatechartExecutor,
):
    first = _compile_and_run_one_node(statechart_executor)
    second = _node_arriving_at_once("second")
    second.start_condition = first.is_succeeded

    statechart_executor.extend([second])
    statechart_executor.tick()
    assert second.life_cycle_state == LifeCycleValues.RUNNING
    statechart_executor.tick()

    assert second.life_cycle_state == LifeCycleValues.SUCCEEDED


def test_extending_keeps_the_outcome_of_the_nodes_already_there(
    statechart_executor: StatechartExecutor,
):
    first = _compile_and_run_one_node(statechart_executor)

    statechart_executor.extend([_node_arriving_at_once("second")])
    statechart_executor.tick()

    assert first.life_cycle_state == LifeCycleValues.SUCCEEDED


def test_extending_does_not_restart_the_tick_count(
    statechart_executor: StatechartExecutor,
):
    _compile_and_run_one_node(statechart_executor)
    ticks_before = statechart_executor.tick_count

    statechart_executor.extend([_node_arriving_at_once("second")])

    assert statechart_executor.tick_count == ticks_before


def test_the_history_of_an_added_node_starts_when_it_joined(
    statechart_executor: StatechartExecutor,
):
    _compile_and_run_one_node(statechart_executor)
    history = statechart_executor.statechart.history
    recorded_before_joining = len(history)
    second = _node_arriving_at_once("second")

    statechart_executor.extend([second])
    statechart_executor.tick()

    recorded_since_joining = len(history) - recorded_before_joining
    assert len(history.get_life_cycle_history_of_node(second)) == recorded_since_joining
    assert second.start_time is not None


def test_an_added_composite_node_runs_its_children(
    statechart_executor: StatechartExecutor,
):
    _compile_and_run_one_node(statechart_executor)
    steps = [_node_arriving_at_once("step one"), _node_arriving_at_once("step two")]

    statechart_executor.extend([Sequence(nodes=steps)])
    for _ in steps:
        statechart_executor.tick()
        statechart_executor.tick()

    assert [step.life_cycle_state for step in steps] == [
        LifeCycleValues.SUCCEEDED,
        LifeCycleValues.SUCCEEDED,
    ]


def test_a_compiled_composite_node_still_rejects_new_children(
    statechart_executor: StatechartExecutor,
):
    statechart = Statechart(context=statechart_executor.context)
    statechart.add_node(sequence := Sequence(nodes=[_node_arriving_at_once("first")]))
    statechart_executor.compile(statechart)

    with pytest.raises(StatechartAlreadyCompiledError):
        sequence.add_node(_node_arriving_at_once("second"))


def test_extending_a_statechart_that_is_not_compiled_is_rejected(
    statechart_executor: StatechartExecutor,
):
    statechart = Statechart(context=statechart_executor.context)
    statechart.add_node(_node_arriving_at_once("first"))

    with pytest.raises(StatechartNotCompiledError):
        statechart.extend([_node_arriving_at_once("second")])
