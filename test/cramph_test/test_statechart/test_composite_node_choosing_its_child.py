from __future__ import annotations

from dataclasses import dataclass, field

from typing_extensions import List

from cramph.composites import (
    ChildChoice,
    ChildChooser,
    ChildChooserAccess,
    ChoicePending,
    ChosenChild,
    CompositeNodeChoosingItsChild,
    NoChildLeft,
)
from cramph.data_types import LifeCycleValues, ObservationStateValues
from cramph.executor import ExecutorExtension, StatechartExecutor
from cramph.node import EndStatechart, StatechartNode
from cramph.nodes_for_testing import (
    ConstFalseNode,
    NodeFailingOnObservingFalse,
    NodeSucceedingOnObservingTrue,
)
from cramph.statechart import Statechart

# %% mimics


@dataclass
class ChooserAnsweringInTurn(ChildChooser):
    """
    Gives the prepared answers one after another, then says no child is left.
    """

    answers: List[ChildChoice]
    """
    The answers still to give, in order.
    """

    asked_nodes: List[CompositeNodeChoosingItsChild] = field(default_factory=list)
    """
    Every node that asked, once per question.
    """

    def choose_child(self, node: CompositeNodeChoosingItsChild, context) -> ChildChoice:
        self.asked_nodes.append(node)
        if not self.answers:
            return NoChildLeft()
        return self.answers.pop(0)


@dataclass
class ExtensionCountingCompiles(ExecutorExtension):
    """
    An executor extension that counts how often it was told the statechart compiled.
    """

    compile_count: int = 0
    """
    How often :meth:`after_compile` ran.
    """

    def after_compile(self, executor: StatechartExecutor) -> None:
        self.compile_count += 1


# %% helpers


def _succeeding_child(name: str) -> NodeSucceedingOnObservingTrue:
    """
    :return: A node that succeeds on the tick after it starts.
    """
    return NodeSucceedingOnObservingTrue(
        name=name, observation=ObservationStateValues.TRUE
    )


def _failing_child(name: str) -> NodeFailingOnObservingFalse:
    """
    :return: A node that fails on the tick after it starts.
    """
    return NodeFailingOnObservingFalse(
        name=name, observation=ObservationStateValues.FALSE
    )


def _run_choosing_node(
    executor: StatechartExecutor,
    chooser: ChildChooser,
    end_node_factory=EndStatechart.when_true,
) -> CompositeNodeChoosingItsChild:
    """
    Run a statechart holding one choosing node until it ends the statechart.

    :return: The choosing node.
    """
    executor.context.add_extension(ChildChooserAccess(chooser=chooser))
    statechart = Statechart(context=executor.context)
    choosing_node = CompositeNodeChoosingItsChild(name="choosing")
    statechart.add_node(choosing_node)
    statechart.add_node(end_node_factory(choosing_node))
    executor.compile(statechart)
    executor.tick_until_end(timeout=30)
    return choosing_node


# %% choosing


def test_the_node_succeeds_with_the_child_it_chose(
    statechart_executor: StatechartExecutor,
):
    child = _succeeding_child("child")

    choosing_node = _run_choosing_node(
        statechart_executor, ChooserAnsweringInTurn([ChosenChild(node=child)])
    )

    assert choosing_node.children == [child]
    assert child.life_cycle_state == LifeCycleValues.SUCCEEDED
    assert choosing_node.life_cycle_state == LifeCycleValues.SUCCEEDED


def test_a_failed_child_makes_the_node_choose_again(
    statechart_executor: StatechartExecutor,
):
    failing = _failing_child("failing")
    succeeding = _succeeding_child("succeeding")

    choosing_node = _run_choosing_node(
        statechart_executor,
        ChooserAnsweringInTurn(
            [ChosenChild(node=failing), ChosenChild(node=succeeding)]
        ),
    )

    assert choosing_node.children == [failing, succeeding]
    assert failing.life_cycle_state == LifeCycleValues.FAILED
    assert choosing_node.life_cycle_state == LifeCycleValues.SUCCEEDED


def test_no_child_left_fails_the_node(statechart_executor: StatechartExecutor):
    choosing_node = _run_choosing_node(
        statechart_executor,
        ChooserAnsweringInTurn([]),
        end_node_factory=EndStatechart.when_failed,
    )

    assert choosing_node.children == []
    assert choosing_node.life_cycle_state == LifeCycleValues.FAILED


def test_a_pending_choice_is_asked_again_on_the_next_tick(
    statechart_executor: StatechartExecutor,
):
    child = _succeeding_child("child")
    chooser = ChooserAnsweringInTurn(
        [ChoicePending(), ChoicePending(), ChosenChild(node=child)]
    )

    choosing_node = _run_choosing_node(statechart_executor, chooser)

    assert chooser.asked_nodes == [choosing_node] * 3
    assert choosing_node.life_cycle_state == LifeCycleValues.SUCCEEDED


def test_a_node_that_is_not_running_is_not_asked(
    statechart_executor: StatechartExecutor,
):
    chooser = ChooserAnsweringInTurn([ChosenChild(node=_succeeding_child("child"))])
    statechart_executor.context.add_extension(ChildChooserAccess(chooser=chooser))
    statechart = Statechart(context=statechart_executor.context)
    never_true = ConstFalseNode(name="never true")
    statechart.add_node(never_true)
    choosing_node = CompositeNodeChoosingItsChild(name="choosing")
    choosing_node.start_condition = never_true.observes_true
    statechart.add_node(choosing_node)

    statechart_executor.compile(statechart)
    statechart_executor.tick()

    assert chooser.asked_nodes == []
    assert choosing_node.life_cycle_state == LifeCycleValues.NOT_STARTED


def test_choosing_keeps_the_state_and_history_of_the_nodes_already_there(
    statechart_executor: StatechartExecutor,
):
    chooser = ChooserAnsweringInTurn([ChosenChild(node=_succeeding_child("child"))])
    statechart_executor.context.add_extension(ChildChooserAccess(chooser=chooser))
    statechart = Statechart(context=statechart_executor.context)
    earlier = _succeeding_child("earlier")
    statechart.add_node(earlier)
    choosing_node = CompositeNodeChoosingItsChild(name="choosing")
    choosing_node.start_condition = earlier.is_succeeded
    statechart.add_node(choosing_node)
    statechart.add_node(EndStatechart.when_true(choosing_node))
    statechart_executor.compile(statechart)
    statechart_executor.tick()
    assert earlier.life_cycle_state == LifeCycleValues.SUCCEEDED
    recorded_before_choosing = len(statechart.history)

    statechart_executor.tick_until_end(timeout=30)

    assert earlier.life_cycle_state == LifeCycleValues.SUCCEEDED
    assert len(statechart.history) == recorded_before_choosing + (
        statechart_executor.tick_count - 1
    )


def test_a_choice_compiles_the_statechart_once(
    statechart_context,
):
    counting = ExtensionCountingCompiles()
    executor = StatechartExecutor(context=statechart_context, extensions=[counting])
    child = _succeeding_child("child")
    executor.context.add_extension(
        ChildChooserAccess(chooser=ChooserAnsweringInTurn([ChosenChild(node=child)]))
    )
    statechart = Statechart(context=executor.context)
    choosing_node = CompositeNodeChoosingItsChild(name="choosing")
    statechart.add_node(choosing_node)

    executor.compile(statechart)

    assert choosing_node.children == [child]
    assert counting.compile_count == 2


def test_the_node_observes_what_its_latest_child_observed(
    statechart_executor: StatechartExecutor,
):
    child: StatechartNode = _succeeding_child("child")

    choosing_node = _run_choosing_node(
        statechart_executor, ChooserAnsweringInTurn([ChosenChild(node=child)])
    )

    assert choosing_node.last_observation_state == ObservationStateValues.TRUE
