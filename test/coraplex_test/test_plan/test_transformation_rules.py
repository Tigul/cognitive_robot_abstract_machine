from dataclasses import dataclass

import pytest
from typing_extensions import List

from coraplex.datastructures.enums import Arms, InsertionPosition
from coraplex.exceptions import PerceptionTargetMissing
from coraplex.orm.ormatic_interface import *  # type: ignore
from coraplex.language import SequentialNode
from coraplex.plans.factories import execute_single
from coraplex.plans.plan_node import ActionLike, ActionNode, MotionNode, PlanNode
from coraplex.plans.transformation_rules import (
    ActionTransformationRule,
    InsertionRule,
)
from coraplex.robot_plans.actions.core.misc import DetectAction
from coraplex.robot_plans.actions.core.navigation import LookAtAction
from coraplex.robot_plans.actions.core.pick_up import PickUpAction, ReachAction
from coraplex.robot_plans.actions.core.robot_body import MoveTorsoAction, ParkArmsAction
from coraplex.robot_plans.motions.gripper import (
    MoveGripperMotion,
    MoveToolCenterPointMotion,
)
from coraplex.robot_plans.motions.robot_body import MoveJointsMotion
from coraplex.robot_plans.transformation_rules import DetectBeforeGrasp
from semantic_digital_twin.datastructures.definitions import GripperState, TorsoState
from semantic_digital_twin.semantic_annotations.semantic_annotations import Milk

from .test_graph_parsing import detect_actions_of, reach_action

# %% rules under test


def motion_of(plan_node: ActionNode) -> MotionNode:
    """
    :param plan_node: The node of an action that expands into a single motion.
    :return: That motion's node.
    """
    [motion] = [node for node in plan_node.descendants if isinstance(node, MotionNode)]
    return motion


@dataclass
class MoveGrippersBesideTorsoMotion(
    InsertionRule, ActionTransformationRule[MoveTorsoAction]
):
    """
    Puts two distinguishable gripper motions next to the motion a torso move expands
    into.
    """

    def anchor(self, plan_node: ActionNode) -> PlanNode:
        return motion_of(plan_node)

    def nodes_to_insert(self, plan_node: ActionNode) -> List[ActionLike]:
        return [
            MoveGripperMotion(GripperState.OPEN, Arms.LEFT),
            MoveGripperMotion(GripperState.CLOSE, Arms.RIGHT),
        ]


@dataclass
class ParkArmsBesideTorsoMotion(
    InsertionRule, ActionTransformationRule[MoveTorsoAction]
):
    """
    Puts an action, which has a plan of its own, next to the motion a torso move expands
    into.
    """

    def anchor(self, plan_node: ActionNode) -> PlanNode:
        return motion_of(plan_node)

    def nodes_to_insert(self, plan_node: ActionNode) -> List[ActionLike]:
        return [ParkArmsAction(Arms.BOTH)]


@dataclass
class MoveGripperBelowTheReachBody(
    InsertionRule, ActionTransformationRule[ReachAction]
):
    """
    Puts a gripper motion below the sequence a reach expands into.
    """

    def anchor(self, plan_node: ActionNode) -> PlanNode:
        [body] = [
            node for node in plan_node.children if isinstance(node, SequentialNode)
        ]
        return body

    def nodes_to_insert(self, plan_node: ActionNode) -> List[ActionLike]:
        return [MoveGripperMotion(GripperState.CLOSE, Arms.RIGHT)]


def motions_of(plan_node: PlanNode) -> List[MotionNode]:
    """
    :param plan_node: The node whose children to look at.
    :return: The motions directly below the given node, in their plan order.
    """
    return [node for node in plan_node.children if isinstance(node, MotionNode)]


# %% inserting


def test_a_rule_inserts_its_nodes_before_the_anchor(immutable_model_world):
    """
    The nodes are placed in front of the anchor, keeping the order the rule gives them.
    """
    world, view, context = immutable_model_world
    context.transformation_rules.append(MoveGrippersBesideTorsoMotion())

    plan = execute_single(MoveTorsoAction(TorsoState.HIGH), context=context)
    plan.notify()

    motions = motions_of(plan)
    assert [type(motion.designator) for motion in motions] == [
        MoveGripperMotion,
        MoveGripperMotion,
        MoveJointsMotion,
    ]
    assert [motion.designator.gripper for motion in motions[:2]] == [
        Arms.LEFT,
        Arms.RIGHT,
    ]


def test_a_rule_inserts_its_nodes_after_the_anchor(immutable_model_world):
    """
    Inserting after the anchor keeps the given order too, rather than reversing it by
    pushing every node into the same place behind the anchor.
    """
    world, view, context = immutable_model_world
    context.transformation_rules.append(
        MoveGrippersBesideTorsoMotion(position=InsertionPosition.AFTER)
    )

    plan = execute_single(MoveTorsoAction(TorsoState.HIGH), context=context)
    plan.notify()

    motions = motions_of(plan)
    assert [type(motion.designator) for motion in motions] == [
        MoveJointsMotion,
        MoveGripperMotion,
        MoveGripperMotion,
    ]
    assert [motion.designator.gripper for motion in motions[1:]] == [
        Arms.LEFT,
        Arms.RIGHT,
    ]


def test_a_rule_inserts_its_nodes_below_the_anchor(immutable_model_world):
    """
    Inserting below the anchor makes the node its last child instead of its sibling.
    """
    world, view, context = immutable_model_world
    milk = world.get_semantic_annotations_by_type(Milk)[0]
    context.transformation_rules.append(
        MoveGripperBelowTheReachBody(position=InsertionPosition.BELOW)
    )

    plan = execute_single(reach_action(milk, view), context=context)
    plan.notify()

    [reach_body] = [node for node in plan.children if isinstance(node, SequentialNode)]
    assert [type(node.designator) for node in reach_body.children] == [
        MoveToolCenterPointMotion,
        MoveToolCenterPointMotion,
        MoveGripperMotion,
    ]


def test_a_rule_leaves_actions_of_another_type_alone(immutable_model_world):
    """
    A rule bound to one action type must not rewrite the plan of another one.
    """
    world, view, context = immutable_model_world
    context.transformation_rules.append(MoveGrippersBesideTorsoMotion())

    plan = execute_single(ParkArmsAction(Arms.BOTH), context=context)
    plan.notify()

    assert [
        node
        for node in plan.descendants
        if isinstance(node, MotionNode)
        and isinstance(node.designator, MoveGripperMotion)
    ] == []


def test_an_inserted_action_is_expanded(immutable_model_world):
    """
    Rules run while the plan is expanded, so an inserted action still gets a plan of its
    own instead of staying an unexpanded leaf.
    """
    world, view, context = immutable_model_world
    context.transformation_rules.append(ParkArmsBesideTorsoMotion())

    plan = execute_single(MoveTorsoAction(TorsoState.HIGH), context=context)
    plan.notify()

    [park] = [
        node
        for node in plan.descendants
        if isinstance(node, ActionNode) and isinstance(node.designator, ParkArmsAction)
    ]
    assert [type(motion.designator) for motion in motions_of(park)] == [
        MoveJointsMotion
    ]


# %% detecting before a grasp


def test_the_detection_asks_for_the_object_being_reached_for(immutable_model_world):
    """
    The detection has to ask for the object the reach was given, so that a plan grasping
    something else does not query for the wrong thing.
    """
    world, view, context = immutable_model_world
    milk = world.get_semantic_annotations_by_type(Milk)[0]
    context.transformation_rules.append(DetectBeforeGrasp())

    plan = execute_single(reach_action(milk, view), context=context)
    plan.notify()

    [detection] = detect_actions_of(plan)
    assert detection.object_sem_annotation is type(milk)


def test_the_perception_precedes_the_final_approach(immutable_model_world):
    """
    Perceiving is only worth anything before the approach it corrects, so the look and
    the detection go in front of the reach's last motion.
    """
    world, view, context = immutable_model_world
    milk = world.get_semantic_annotations_by_type(Milk)[0]
    context.transformation_rules.append(DetectBeforeGrasp())

    plan = execute_single(reach_action(milk, view), context=context)
    plan.notify()

    [reach_body] = [node for node in plan.children if isinstance(node, SequentialNode)]
    assert [type(node.designator) for node in reach_body.children] == [
        MoveToolCenterPointMotion,
        LookAtAction,
        DetectAction,
        MoveToolCenterPointMotion,
    ]


def test_a_rule_on_reaches_also_fires_inside_a_pick_up(immutable_model_world):
    """
    The reach a pick-up builds is expanded like any other, so a rule on reaches reaches
    it without the pick-up having to pass anything down.
    """
    world, view, context = immutable_model_world
    milk = world.get_semantic_annotations_by_type(Milk)[0]
    context.transformation_rules.append(DetectBeforeGrasp())

    plan = execute_single(
        PickUpAction(milk, Arms.RIGHT, reach_action(milk, view).grasp_description),
        context=context,
    )
    plan.notify()

    [detection] = detect_actions_of(plan)
    assert detection.object_sem_annotation is type(milk)


def test_perceiving_without_an_object_to_detect_is_rejected(immutable_model_world):
    """
    A reach may be given a pose without an object, but then there is nothing to build
    the detection query from, so the contradiction is reported instead of guessed away.
    """
    world, view, context = immutable_model_world
    milk = world.get_semantic_annotations_by_type(Milk)[0]
    context.transformation_rules.append(DetectBeforeGrasp())

    reach = reach_action(milk, view)
    reach.object_designator = None

    with pytest.raises(PerceptionTargetMissing):
        execute_single(reach, context=context).notify()
