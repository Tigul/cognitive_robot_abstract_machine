"""
Tests for compiling and executing a plan with a :class:`PlanExecutor`.
"""

from datetime import timedelta

import pytest

from coraplex.exceptions import MotionDidNotFinish, PlanNotCompiled
from coraplex.execution_environment import no_execution, simulated_robot
from coraplex.plans.failures import (
    MotionExceededSimulationTimeLimit,
    MotionMadeNoProgress,
    MotionViolatedCollisionAvoidance,
)
from coraplex.plans.plan_execution import PlanExecutor, SimulatedPlanRun
from coraplex.robot_plans.actions.core.pick_up import ReachAction
from coraplex.robot_plans.actions.core.robot_body import MoveTorsoAction
from cramph.data_types import LifeCycleValues, ObservationStateValues
from cramph.executor import StatechartExecutor
from cramph.node import CancelStatechart
from cramph.nodes_for_testing import NodeFailingOnObservingFalse
from giskardpy.motion_statechart.exceptions import CollisionViolatedError
from giskardpy.motion_statechart.graph_node import EndMotion
from giskardpy.motion_statechart.monitors.progress_monitors import StillProgressing
from semantic_digital_twin.datastructures.definitions import TorsoState
from semantic_digital_twin.grasping.grasp_candidates import GraspCandidate
from semantic_digital_twin.robots.pr2 import PR2Joint
from semantic_digital_twin.semantic_annotations.semantic_annotations import Milk
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix

# %% compiling


def test_compiling_puts_the_plan_and_its_end_in_one_statechart(
    pr2_apartment_context,
):
    world, robot_view, context = pr2_apartment_context
    plan = MoveTorsoAction(TorsoState.HIGH)
    executor = PlanExecutor(context)

    with simulated_robot:
        executor.compile(plan)

    statechart = plan.statechart
    assert plan in statechart.top_level_nodes
    assert len(statechart.get_nodes_by_type(EndMotion)) == 1


def test_executing_before_compiling_is_refused(pr2_apartment_context):
    world, robot_view, context = pr2_apartment_context

    with pytest.raises(PlanNotCompiled):
        PlanExecutor(context).execute()


# %% executing


def test_executing_runs_the_plan_until_it_succeeded(pr2_apartment_context):
    world, robot_view, context = pr2_apartment_context
    plan = MoveTorsoAction(TorsoState.HIGH)
    executor = PlanExecutor(context)

    with simulated_robot:
        executor.compile(plan)
        executor.execute()

    assert plan.life_cycle_state == LifeCycleValues.SUCCEEDED


def test_a_plan_that_fails_raises_motion_did_not_finish(pr2_apartment_context):
    world, robot_view, context = pr2_apartment_context
    plan = NodeFailingOnObservingFalse(observation=ObservationStateValues.FALSE)
    executor = PlanExecutor(context)

    with simulated_robot, pytest.raises(MotionDidNotFinish):
        executor.compile(plan)
        executor.execute()


def test_without_execution_the_robot_does_not_move(pr2_apartment_context):
    world, robot_view, context = pr2_apartment_context
    torso = world.get_degree_of_freedom_by_name(PR2Joint.TORSO_LIFT).id
    position_before = world.state[torso].position
    executor = PlanExecutor(context)

    with no_execution:
        executor.compile(MoveTorsoAction(TorsoState.HIGH))
        executor.execute()

    assert world.state[torso].position == position_before


# %% how long a plan may take


def test_compiling_watches_the_plan_for_progress(pr2_apartment_context):
    """
    A stalled run has to end by itself, so the statechart carries a monitor watching the
    plan and an abort path wired to it.
    """
    world, robot_view, context = pr2_apartment_context
    plan = MoveTorsoAction(TorsoState.HIGH)
    executor = PlanExecutor(context)

    with simulated_robot:
        executor.compile(plan)

    [progress_monitor] = plan.statechart.get_nodes_by_type(StillProgressing)
    assert progress_monitor.monitored_node is plan
    assert len(plan.statechart.get_nodes_by_type(CancelStatechart)) == 1


def test_a_plan_that_stops_approaching_its_goal_is_given_up_on(
    pr2_apartment_context,
):
    """
    Nothing bounds the tick loop but the monitor, so a reach the arm cannot close on has
    to end the run rather than tick forever.
    """
    world, robot_view, context = pr2_apartment_context
    milk = world.get_semantic_annotations_by_type(Milk)[0]
    milk.root.parent_connection.origin = HomogeneousTransformationMatrix.from_xyz_rpy(
        2, 1.5, 50, reference_frame=milk.root.parent_connection.parent
    )
    plan = ReachAction(
        grasp=GraspCandidate.from_body_origin(milk), arm=context.robot.right_arm
    )
    executor = PlanExecutor(context)

    with simulated_robot, pytest.raises(MotionMadeNoProgress):
        executor.compile(plan)
        executor.execute()


def test_a_plan_that_outlasts_the_simulation_time_limit_is_given_up_on(
    pr2_apartment_context, monkeypatch
):
    """
    A simulated plan is given up on once it exceeds the simulation time limit.
    """
    world, robot_view, context = pr2_apartment_context
    monkeypatch.setattr(SimulatedPlanRun, "simulation_time_limit", timedelta(0))
    executor = PlanExecutor(context)

    with simulated_robot, pytest.raises(MotionExceededSimulationTimeLimit):
        executor.compile(MoveTorsoAction(TorsoState.HIGH))
        executor.execute()


def test_a_plan_that_violates_collision_avoidance_fails_as_a_plan_failure(
    pr2_apartment_context, monkeypatch
):
    """
    A plan that brings the robot closer to something than collision avoidance allows did
    not work from where it started, so a surrounding plan can try another candidate
    instead of stopping.
    """
    world, robot_view, context = pr2_apartment_context
    violation = CollisionViolatedError(violated_collisions=[], thresholds=[])

    def violate_collision_avoidance(executor: StatechartExecutor) -> None:
        raise violation

    executor = PlanExecutor(context)
    with simulated_robot:
        executor.compile(MoveTorsoAction(TorsoState.HIGH))
        monkeypatch.setattr(StatechartExecutor, "tick", violate_collision_avoidance)
        with pytest.raises(MotionViolatedCollisionAvoidance) as failure:
            executor.execute()

    assert failure.value.violation is violation
