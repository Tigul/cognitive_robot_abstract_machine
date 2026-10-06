"""
Tests for compiling and executing a plan with a :class:`PlanExecutor`.
"""

import pytest

from coraplex.exceptions import MotionDidNotFinish, PlanNotCompiled
from coraplex.execution_environment import no_execution, simulated_robot
from coraplex.plans.plan_execution import PlanExecutor
from coraplex.robot_plans.actions.core.robot_body import MoveTorsoAction
from cramph.data_types import LifeCycleValues, ObservationStateValues
from cramph.nodes_for_testing import NodeFailingOnObservingFalse
from giskardpy.motion_statechart.graph_node import EndMotion
from semantic_digital_twin.datastructures.definitions import TorsoState
from semantic_digital_twin.robots.pr2 import PR2Joint

# %% compiling


def test_compiling_puts_the_plan_and_its_end_in_one_statechart(
    immutable_model_world,
):
    world, robot_view, context = immutable_model_world
    plan = MoveTorsoAction(TorsoState.HIGH)
    executor = PlanExecutor(context)

    with simulated_robot:
        executor.compile(plan)

    statechart = plan.statechart
    assert plan in statechart.top_level_nodes
    assert len(statechart.get_nodes_by_type(EndMotion)) == 1


def test_executing_before_compiling_is_refused(immutable_model_world):
    world, robot_view, context = immutable_model_world

    with pytest.raises(PlanNotCompiled):
        PlanExecutor(context).execute()


# %% executing


def test_executing_runs_the_plan_until_it_succeeded(immutable_model_world):
    world, robot_view, context = immutable_model_world
    plan = MoveTorsoAction(TorsoState.HIGH)
    executor = PlanExecutor(context)

    with simulated_robot:
        executor.compile(plan)
        executor.execute()

    assert plan.life_cycle_state == LifeCycleValues.SUCCEEDED


def test_a_plan_that_fails_raises_motion_did_not_finish(immutable_model_world):
    world, robot_view, context = immutable_model_world
    plan = NodeFailingOnObservingFalse(observation=ObservationStateValues.FALSE)
    executor = PlanExecutor(context)

    with simulated_robot, pytest.raises(MotionDidNotFinish):
        executor.compile(plan)
        executor.execute()


def test_without_execution_the_robot_does_not_move(immutable_model_world):
    world, robot_view, context = immutable_model_world
    torso = world.get_degree_of_freedom_by_name(PR2Joint.TORSO_LIFT).id
    position_before = world.state[torso].position
    executor = PlanExecutor(context)

    with no_execution:
        executor.compile(MoveTorsoAction(TorsoState.HIGH))
        executor.execute()

    assert world.state[torso].position == position_before
