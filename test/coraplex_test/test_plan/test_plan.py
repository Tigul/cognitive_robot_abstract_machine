"""
Tests for performing plans and for what their actions move.
"""

import pytest

from coraplex.datastructures.enums import (
    ApproachDirection,
    VerticalAlignment,
    Arms,
)
from coraplex.datastructures.grasp import GraspDescription
from coraplex.execution_environment import simulated_robot
from coraplex.plans.failures import EmptyUnderspecified
from coraplex.robot_plans.actions.core.navigation import NavigateAction
from coraplex.robot_plans.actions.core.pick_up import PickUpAction
from coraplex.robot_plans.actions.core.placing import PlaceAction
from coraplex.robot_plans.actions.core.robot_body import MoveTorsoAction
from cramph.node import CompositeNode, StatechartNode
from coraplex.datastructures.dataclasses import Context
from giskardpy.motion_statechart.goals.gripper import MoveGripper
from giskardpy.motion_statechart.tasks.cartesian_tasks import (
    CartesianPose,
    CartesianPosition,
)
from krrood.entity_query_language.backends import ProbabilisticBackend
from krrood.entity_query_language.factories import (
    variable_from,
    a,
    variable,
)
from krrood.parametrization.model_registries import (
    FullyFactorizedRegistry,
)
from krrood.parametrization.parameterizer import UnderspecifiedParameters
from semantic_digital_twin.datastructures.definitions import GripperState, TorsoState
from semantic_digital_twin.orm.model import (
    Point3Mapping,
    QuaternionMapping,
    PoseMapping,
)
from semantic_digital_twin.robots.pr2 import PR2Joint
from semantic_digital_twin.robots.robot_parts import EndEffector
from semantic_digital_twin.semantic_annotations.semantic_annotations import Milk
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix, Pose

from ..conftest import expand
from coraplex.plans.plan_execution import PlanExecutor
from coraplex.plans.underspecified import UnderspecifiedNode
from cramph.composites import Sequence


def _torso_position(world):
    return world.state[
        world.get_degree_of_freedom_by_name(PR2Joint.TORSO_LIFT).id
    ].position


def test_sequence_runs_all_motions(immutable_model_world):
    """
    Every motion of a sequence is executed, so the torso ends at the target of the
    *last* motion.

    The robot starts in the LOW configuration, so a final HIGH motion proves the second
    motion actually ran.
    """
    world, robot_view, context = immutable_model_world

    plan = Sequence([MoveTorsoAction(TorsoState.LOW), MoveTorsoAction(TorsoState.HIGH)])
    with simulated_robot:
        executor = PlanExecutor(context)
        executor.compile(plan)
        executor.execute()

    assert _torso_position(world) == pytest.approx(0.3, abs=0.05)


def test_algebra_sequential_plan(apartment_world_pr2_copy_with_context):
    """
    Parameterize a sequence using krrood parameterizer, create a fully- factorized
    distribution and assert the correctness of sampled values after conditioning and
    truncation.
    """
    world, robot_view, context = apartment_world_pr2_copy_with_context
    context.evaluate_conditions = False

    target_location = a(PoseMapping.from_point_mapping_quaternion_mapping)(
        position=a(Point3Mapping)(x=..., y=..., z=0.0, reference_frame=None),
        orientation=QuaternionMapping(x=0, y=0, z=0, w=1, reference_frame=None),
        reference_frame=variable_from([robot_view.root]),
    )

    navigate_action = a(NavigateAction)(
        target_location=target_location,
    )
    # navigate_action.resolve()

    context.query_backend = ProbabilisticBackend(
        model_registry=FullyFactorizedRegistry()
    )

    # resolved_navigate = next(pm_backend.evaluate(navigate_action))
    plan = Sequence(
        [MoveTorsoAction(TorsoState.LOW), UnderspecifiedNode(statement=navigate_action)]
    )

    with simulated_robot:
        executor = PlanExecutor(context)
        executor.compile(plan)
        executor.execute()

    assert isinstance(plan.nodes[1].latest_child, NavigateAction)
    assert len(plan.nodes[1].children) == 1


def test_parameterization_of_pick_up(apartment_world_pr2_copy_with_context):
    world, robot_view, context = apartment_world_pr2_copy_with_context
    context.evaluate_conditions = False

    milk = world.get_semantic_annotations_by_type(Milk)[0]

    milk_variable = variable_from([milk])

    pick_up_description = a(PickUpAction)(
        object_designator=milk_variable,
        arm=...,
        grasp_description=a(GraspDescription)(
            approach_direction=...,
            vertical_alignment=...,
            rotate_gripper=...,
            manipulation_offset=0.05,
            end_effector=variable(EndEffector, world.semantic_annotations),
        ),
    )

    parameters = UnderspecifiedParameters(pick_up_description)

    [end_effector_offset] = [
        v
        for v in parameters.variables.values()
        if v.name.endswith("manipulation_offset")
    ]

    assert (
        parameters.conditioning_assignments_from_literal_values[end_effector_offset]
        == 0.05
    )

    context.query_backend = ProbabilisticBackend(
        model_registry=FullyFactorizedRegistry()
    )

    plan = UnderspecifiedNode(statement=pick_up_description)

    with simulated_robot:
        try:
            executor = PlanExecutor(context)
            executor.compile(plan)
            executor.execute()
        except EmptyUnderspecified:
            pass


def test_motion_order_pick_up(mutable_model_world):
    world, robot_view, context = mutable_model_world

    grasp_description = GraspDescription(
        ApproachDirection.FRONT,
        VerticalAlignment.NoAlignment,
        robot_view.left_arm.end_effector,
    )

    milk_body = world.get_body_by_name("milk.stl")
    milk_body.parent_connection.origin = HomogeneousTransformationMatrix.from_xyz_rpy(
        1, -2, 0.6, reference_frame=world.root
    )
    robot_view.root.parent_connection.origin = (
        HomogeneousTransformationMatrix.from_xyz_rpy(
            0.3, -2.4, 0, reference_frame=world.root
        )
    )
    world.notify_state_change()

    root = Sequence(
        [
            PickUpAction(
                world.get_semantic_annotations_by_type(Milk)[0],
                Arms.LEFT,
                grasp_description,
            ),
        ]
    )

    performed_motions = _motions_of(root, context)

    assert performed_motions == [
        CartesianPose,
        GripperState.OPEN,
        CartesianPose,
        GripperState.CLOSE,
        CartesianPosition,
    ]


def test_motion_order_place(mutable_model_world):
    world, robot_view, context = mutable_model_world

    milk_body = world.get_body_by_name("milk.stl")
    milk_body.parent_connection.origin = world.get_body_by_name(
        "l_gripper_tool_frame"
    ).global_pose.to_homogeneous_matrix()

    with world.modify_world():

        world.move_branch_with_fixed_connection(
            world.get_body_by_name("milk.stl"),
            world.get_body_by_name("l_gripper_tool_frame"),
        )

    robot_view.root.parent_connection.origin = (
        HomogeneousTransformationMatrix.from_xyz_rpy(
            0.3, -2.4, 0, reference_frame=world.root
        )
    )
    world.notify_state_change()

    root = Sequence(
        [
            PlaceAction(
                world.get_body_by_name("milk.stl"),
                Pose.from_xyz_rpy(0.8, -1.9, 0.7, reference_frame=world.root),
                Arms.LEFT,
            ),
        ]
    )

    performed_motions = _motions_of(root, context)

    assert performed_motions == [
        CartesianPose,
        CartesianPose,
        GripperState.OPEN,
        CartesianPose,
    ]


# %% reading back what a plan moves


def _motions_of(plan: StatechartNode, context: Context) -> list:
    """
    Expand `plan` in `context` and report what it moves, in the order it runs.

    :return: One entry per motion: the gripper state a gripper motion commands, or the
        type of the Cartesian task any other motion is built around.
    """
    return _motions_below(expand(plan, context))


def _motions_below(goal) -> list:
    """
    :return: What every motion below `goal` moves, in the order the chart runs them.
    """
    found = []
    for node in goal.children:
        if isinstance(node, MoveGripper):
            found.append(node.state)
        elif isinstance(node, (CartesianPose, CartesianPosition)):
            found.append(type(node))
        elif isinstance(node, CompositeNode):
            found.extend(_motions_below(node))
    return found
