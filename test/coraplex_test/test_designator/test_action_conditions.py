from copy import deepcopy

import pytest

from krrood.entity_query_language.factories import (
    get_false_statements,
    evaluate_condition,
    ConditionType,
)
from coraplex.datastructures.enums import Arms, ApproachDirection, VerticalAlignment
from coraplex.datastructures.grasp import GraspDescription
from coraplex.exceptions import ConditionNotSatisfied, MotionDidNotFinish
from coraplex.execution_environment import simulated_robot
from coraplex.plans.factories import execute_single, sequential
from coraplex.robot_plans.actions.core.pick_up import PickUpAction
from coraplex.robot_plans.actions.core.robot_body import MoveTorsoAction
from semantic_digital_twin.datastructures.definitions import TorsoState
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix
from semantic_digital_twin.semantic_annotations.semantic_annotations import Milk


def _construct_and_evaluate_condition(action, action_condition):

    condition = action_condition(
        action.bound_variables,
        action.context,
        action.designator_parameter,
    )
    evaluation = evaluate_condition(condition)
    if evaluation:
        return True
    raise ConditionNotSatisfied(
        pre_condition=True, action=action.__class__, condition=condition
    )


def test_get_bound_variables(immutable_model_world):
    world, view, context = immutable_model_world

    pick_action = PickUpAction(
        world.get_semantic_annotations_by_type(Milk)[0],
        Arms.LEFT,
        GraspDescription(
            ApproachDirection.FRONT,
            VerticalAlignment.NoAlignment,
            view.left_arm.end_effector,
        ),
    )

    bound_variables = pick_action._create_variables()

    assert len(bound_variables) == 12
    assert list(bound_variables.keys()) == [
        "grasp_detection_threshold",
        "pre_approach_linear_velocity",
        "final_approach_linear_velocity",
        "grasp_closing_velocity",
        "lift_linear_velocity",
        "grasp_stall_minimum_time",
        "object_friction",
        "object_designator",
        "arm",
        "grasp_description",
        "tolerate_grasp_stall",
        "perceive_before_grasp",
    ]
    assert list(bound_variables["arm"]._domain_) == [Arms.LEFT]
    assert bound_variables["arm"]._type_ == Arms
    assert list(bound_variables["object_designator"]._domain_) == [
        world.get_semantic_annotations_by_type(Milk)[0]
    ]
    assert bound_variables["object_designator"]._type_ == Milk


def test_pick_up_pre_conditions(mutable_model_world):
    world, view, context = mutable_model_world

    pick_action = PickUpAction(
        world.get_semantic_annotations_by_type(Milk)[0],
        Arms.LEFT,
        GraspDescription(
            ApproachDirection.FRONT,
            VerticalAlignment.NoAlignment,
            view.left_arm.end_effector,
        ),
    )

    plan = sequential([pick_action], context)

    with pytest.raises(ConditionNotSatisfied):
        _construct_and_evaluate_condition(
            pick_action,
            pick_action.pre_condition,
        )

    pre_condition = pick_action.pre_condition(
        pick_action.bound_variables, context, pick_action.designator_parameter
    )

    false_statements = get_false_statements(pre_condition)

    assert len(false_statements) == 1
    assert false_statements[0]._name_ == "IsObjectReachableBy"

    with pytest.raises(ConditionNotSatisfied):
        _construct_and_evaluate_condition(pick_action, pick_action.pre_condition)

    view.root.parent_connection.origin = HomogeneousTransformationMatrix.from_xyz_rpy(
        1.9, 1.4, 0
    )

    pre_condition = pick_action.pre_condition(
        pick_action.bound_variables, context, pick_action.designator_parameter
    )

    assert evaluate_condition(pre_condition) == True

    with simulated_robot:
        plan.perform()

    assert evaluate_condition(pre_condition) == False
    _construct_and_evaluate_condition(pick_action, pick_action.post_condition)
    assert _construct_and_evaluate_condition(pick_action, pick_action.post_condition)


# %% conditions follow the plan's world


def test_built_conditions_read_the_plans_current_world(immutable_model_world):
    """
    A condition binds the world entities it reads at the moment it is built, so building
    it again after the plan is switched onto a copied world reads that world instead of
    the one the action was expanded against.
    """
    world, view, context = immutable_model_world

    action = MoveTorsoAction(TorsoState.HIGH)
    action_node = execute_single(action, context=context)
    action_node.notify()

    other_context = context.copy_for_other_world(deepcopy(world))
    other_context.robot.get_torso().get_joint_state_by_type(TorsoState.HIGH).apply_to(
        other_context.world
    )
    action_node.plan.context = other_context

    # The condition built while the action was expanded still reads the original world,
    # where the torso never moved.
    assert not evaluate_condition(action_node.children[-1].condition)
    assert evaluate_condition(action.build_post_condition())


def test_expansion_builds_the_condition_nodes_from_the_built_conditions(
    immutable_model_world,
):
    """
    Expanding an action stores the same conditions the build methods produce, so there
    is one definition of what an action's conditions are.
    """
    world, view, context = immutable_model_world

    action = MoveTorsoAction(TorsoState.HIGH)
    action_node = execute_single(action, context=context)
    action_node.notify()

    pre_condition_node, post_condition_node = (
        action_node.children[0],
        action_node.children[-1],
    )

    assert pre_condition_node.pre_condition
    assert not post_condition_node.pre_condition
    assert evaluate_condition(post_condition_node.condition) == evaluate_condition(
        action.build_post_condition()
    )


def test_pick_up_post_condition(mutable_model_world):
    world, view, context = mutable_model_world
    pick_action = PickUpAction(
        world.get_semantic_annotations_by_type(Milk)[0],
        Arms.LEFT,
        GraspDescription(
            ApproachDirection.FRONT,
            VerticalAlignment.NoAlignment,
            view.left_arm.end_effector,
        ),
    )
    # The standing pose test_pick_up_pre_condition establishes as reaching the milk.
    view.root.parent_connection.origin = HomogeneousTransformationMatrix.from_xyz_rpy(
        1.9, 1.4, 0
    )

    plan = sequential([pick_action], context)

    assert _construct_and_evaluate_condition(pick_action, pick_action.pre_condition)

    with simulated_robot:
        plan.perform()

    assert world.get_body_by_name(
        "milk.stl"
    ) in world.get_kinematic_structure_entities_of_branch(
        view.left_arm.end_effector.tool_frame
    )

    assert _construct_and_evaluate_condition(pick_action, pick_action.post_condition)
