"""
Tests for the monitored subtree templates (see
``giskardpy/motion_statechart/monitors/templates.py``).

``build_artifacts`` reads nothing but the observations of the monitor and the monitored
node and the life cycle of the monitored node, all of which exist as variables from
construction, so each observation expression is evaluated by substituting values into it
rather than by ticking an executor. How the templates end and pause their monitored node
is tested by ticking an executor.
"""

import pytest

from giskardpy.executor import Executor
from giskardpy.motion_statechart.context import MotionStatechartContext
from giskardpy.motion_statechart.data_types import (
    LifeCycleValues,
    ObservationStateValues,
)
from giskardpy.motion_statechart.goals.templates import Attempt
from giskardpy.motion_statechart.graph_node import (
    DerivedConditionVariable,
    MotionStatechartNode,
)
from giskardpy.motion_statechart.monitors.templates import (
    MonitoredCompositeStatechartNode,
    PausedUntilTrue,
    PausedWhileTrue,
    StoppedWhenTrue,
)
from giskardpy.motion_statechart.monitors.payload_monitors import CountControlCycles
from giskardpy.motion_statechart.motion_statechart import MotionStatechart
from giskardpy.motion_statechart.nodes_for_testing.nodes_for_testing import (
    ConstFalseNode,
    ConstTrueNode,
    SelfFailingMaintenanceNode,
)
from krrood.symbolic_math.symbolic_math import Scalar
from semantic_digital_twin.world import World

SETTLE_CYCLES = 6
"""
Control cycles after which the templates ticked below have settled on an outcome.
"""

UNFINISHED_OBSERVATIONS = [ObservationStateValues.FALSE, ObservationStateValues.UNKNOWN]
"""
The observations of a monitored node that has not reached its goal.
"""

# %% evaluating a template's observation


def create_goal(
    goal_type: type[MonitoredCompositeStatechartNode],
) -> MonitoredCompositeStatechartNode:
    """
    :param goal_type: The template to instantiate.
    :return: A goal whose monitor and monitored node contribute nothing but their
        observation variables.
    """
    return goal_type(
        monitor=MotionStatechartNode(name="monitor"),
        monitored_node=MotionStatechartNode(name="monitored"),
    )


def observation_for(
    goal: MonitoredCompositeStatechartNode,
    monitored_observation: ObservationStateValues,
    monitor_observation: ObservationStateValues,
    monitored_life_cycle: LifeCycleValues = LifeCycleValues.RUNNING,
) -> ObservationStateValues:
    """
    Evaluate the observation a goal builds for one pair of input observations.

    An observation expression reads the observation a node took on the previous control
    cycle, which is also its last observation, so the same value stands for both of a
    node's observation variables. The predicates over those variables and the life cycle
    state are replaced by what they stand for first, as compiling the observation does.

    :param goal: The goal whose observation expression is evaluated.
    :param monitored_observation: What the monitored node observed.
    :param monitor_observation: What the monitor observed.
    :param monitored_life_cycle: The life cycle state the monitored node is in.
    :return: What the goal observes.
    """
    artifacts = goal.build_artifacts(MotionStatechartContext(world=World()))
    # A template may hand back a node's observation variable unwrapped, which cannot be
    # copied and therefore not substituted into.
    substituted = DerivedConditionVariable.substitute_in(
        Scalar(artifacts.observation)
    ).substitute(
        [
            goal.monitored_node.observation_variable,
            goal.monitored_node.last_observation,
            goal.monitored_node.life_cycle_variable,
            goal.monitor.observation_variable,
            goal.monitor.last_observation,
        ],
        [
            monitored_observation,
            monitored_observation,
            monitored_life_cycle,
            monitor_observation,
            monitor_observation,
        ],
    )
    return ObservationStateValues(float(substituted))


# %% pausing leaves the outcome to the monitored node


@pytest.mark.parametrize("goal_type", [PausedWhileTrue, PausedUntilTrue])
@pytest.mark.parametrize("monitored_observation", list(ObservationStateValues))
@pytest.mark.parametrize("monitor_observation", list(ObservationStateValues))
def test_pausing_templates_observe_the_monitored_node(
    goal_type: type[MonitoredCompositeStatechartNode],
    monitored_observation: ObservationStateValues,
    monitor_observation: ObservationStateValues,
) -> None:
    """
    A monitor that only pauses never changes the outcome, so the goal reports exactly
    what the monitored node observes.
    """
    goal = create_goal(goal_type)

    assert (
        observation_for(goal, monitored_observation, monitor_observation)
        is monitored_observation
    )


# %% stopping a monitored node


@pytest.mark.parametrize("monitor_observation", list(ObservationStateValues))
def test_stopped_when_true_succeeds_once_the_monitored_node_succeeded(
    monitor_observation: ObservationStateValues,
) -> None:
    """
    A monitored node that reached its goal makes the template succeed, whatever the
    monitor observes.
    """
    goal = create_goal(StoppedWhenTrue)

    assert (
        observation_for(goal, ObservationStateValues.TRUE, monitor_observation)
        is ObservationStateValues.TRUE
    )


@pytest.mark.parametrize("monitored_observation", UNFINISHED_OBSERVATIONS)
def test_stopped_when_true_fails_when_it_stopped_an_unfinished_node(
    monitored_observation: ObservationStateValues,
) -> None:
    """
    A monitor that fires before the monitored node reached its goal ends it as a
    failure.
    """
    goal = create_goal(StoppedWhenTrue)

    assert (
        observation_for(goal, monitored_observation, ObservationStateValues.TRUE)
        is ObservationStateValues.FALSE
    )


@pytest.mark.parametrize("monitor_observation", list(ObservationStateValues))
def test_stopped_when_true_keeps_succeeding_once_the_monitored_node_succeeded(
    monitor_observation: ObservationStateValues,
) -> None:
    """
    A monitored node that succeeded observes nothing any more, so its outcome is what
    makes the template succeed.
    """
    goal = create_goal(StoppedWhenTrue)

    assert (
        observation_for(
            goal,
            ObservationStateValues.UNKNOWN,
            monitor_observation,
            monitored_life_cycle=LifeCycleValues.SUCCEEDED,
        )
        is ObservationStateValues.TRUE
    )


def test_stopped_when_true_fails_when_it_stopped_a_node_observing_true() -> None:
    """
    The observation read on the cycle after the monitor stopped the monitored node is
    still the True it took while running, which must not count as a success.
    """
    goal = create_goal(StoppedWhenTrue)

    assert (
        observation_for(
            goal,
            ObservationStateValues.TRUE,
            ObservationStateValues.TRUE,
            monitored_life_cycle=LifeCycleValues.INTERRUPTED,
        )
        is ObservationStateValues.FALSE
    )


@pytest.mark.parametrize("monitored_observation", UNFINISHED_OBSERVATIONS)
@pytest.mark.parametrize("monitor_observation", UNFINISHED_OBSERVATIONS)
def test_stopped_when_true_stays_unknown_while_the_monitored_node_runs(
    monitored_observation: ObservationStateValues,
    monitor_observation: ObservationStateValues,
) -> None:
    """
    Neither a monitored node that has not succeeded yet nor a monitor that has not fired
    decides anything, so the template observes Unknown.
    """
    goal = create_goal(StoppedWhenTrue)

    assert (
        observation_for(goal, monitored_observation, monitor_observation)
        is ObservationStateValues.UNKNOWN
    )


# %% a monitored node that ends on its own


def compile_chart(nodes: list[MotionStatechartNode]) -> Executor:
    """
    :param nodes: The top level nodes of a fresh statechart.
    :return: The executor, after compiling the statechart.
    """
    motion_statechart = MotionStatechart()
    motion_statechart.add_nodes(nodes)
    executor = Executor(MotionStatechartContext(world=World()))
    executor.compile(motion_statechart=motion_statechart)
    return executor


def tick_compiled(node: MotionStatechartNode) -> Executor:
    """
    :param node: The node to run as the only top level node of a fresh statechart.
    :return: The executor, after compiling the statechart and ticking it for
        :data:`SETTLE_CYCLES` control cycles.
    """
    executor = compile_chart([node])
    for _ in range(SETTLE_CYCLES):
        executor.tick()
    return executor


@pytest.mark.parametrize(
    "goal_type, monitor_type",
    [
        (PausedWhileTrue, ConstFalseNode),
        (PausedUntilTrue, ConstTrueNode),
        (StoppedWhenTrue, ConstFalseNode),
    ],
)
def test_a_template_fails_once_its_monitored_node_failed_on_its_own(
    goal_type: type[MonitoredCompositeStatechartNode],
    monitor_type: type[MotionStatechartNode],
) -> None:
    """
    A monitored node that failed never arrives, so the template must not keep its owner
    waiting for it.

    The monitor lets the monitored node run and never stops it.
    """
    monitored_node = SelfFailingMaintenanceNode(
        name="monitored", observation=ObservationStateValues.FALSE
    )
    goal = goal_type(
        monitor=monitor_type(name="monitor"), monitored_node=monitored_node
    )

    tick_compiled(goal)

    assert monitored_node.life_cycle_state == LifeCycleValues.FAILED
    assert goal.life_cycle_state == LifeCycleValues.FAILED


@pytest.mark.parametrize("goal_type", [PausedWhileTrue, StoppedWhenTrue])
def test_a_template_whose_monitored_node_failed_observing_true_fails_its_attempt(
    goal_type: type[MonitoredCompositeStatechartNode],
) -> None:
    """
    What a monitored node observed on the cycle it failed is no arrival, so an attempt
    running the template must not succeed on it.
    """
    monitored_node = ConstTrueNode(name="monitored")
    monitored_node.fail_condition = monitored_node.observes_true
    goal = goal_type(
        monitor=ConstFalseNode(name="monitor"), monitored_node=monitored_node
    )
    attempt = Attempt(task=goal, failure_monitors=[])

    tick_compiled(attempt)

    assert goal.life_cycle_state == LifeCycleValues.FAILED
    assert attempt.life_cycle_state == LifeCycleValues.FAILED


# %% pausing from the start


def test_paused_until_true_never_runs_its_node_before_the_monitor_observed_true() -> (
    None
):
    """
    A monitor that has not observed True yet holds the monitored node from the control
    cycle the template starts in, so the node's constraints never act before that.
    """
    delay = CountControlCycles(name="delay", control_cycles=SETTLE_CYCLES // 2)
    monitored_node = ConstTrueNode(name="monitored")
    goal = PausedUntilTrue(
        monitor=ConstFalseNode(name="monitor"), monitored_node=monitored_node
    )
    goal.start_condition = delay.observes_true
    executor = compile_chart([delay, goal])

    life_cycles = []
    for _ in range(SETTLE_CYCLES):
        executor.tick()
        life_cycles.append(monitored_node.life_cycle_state)

    assert LifeCycleValues.PAUSED in life_cycles
    assert LifeCycleValues.RUNNING not in life_cycles
