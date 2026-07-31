import pytest
from semantic_digital_twin.semantic_annotations.semantic_annotations import Milk
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix

from coraplex.datastructures.enums import Arms, ApproachDirection, VerticalAlignment
from coraplex.datastructures.grasp import GraspDescription
from coraplex.exceptions import MotionDidNotFinish
from coraplex.execution_environment import simulated_robot
from coraplex.plans.factories import sequential
from coraplex.plans.plan_node import ActionNode, MotionNode
from coraplex.robot_plans.actions.core.pick_up import PickUpAction

# %% helpers


def unreachable_pick_up(world, view, context) -> PickUpAction:
    """
    :return: A pick-up whose motions cannot succeed, because the robot is placed out of
        reach of the object it grasps.
    """
    pick_up = PickUpAction(
        target_object=world.get_semantic_annotations_by_type(Milk)[0],
        arm=Arms.LEFT,
        grasp_description=GraspDescription(
            ApproachDirection.FRONT,
            VerticalAlignment.NoAlignment,
            view.left_arm.end_effector,
        ),
    )
    view.root.parent_connection.origin = HomogeneousTransformationMatrix.from_xyz_rpy(
        1.0, 2, 0
    )
    context.evaluate_conditions = False
    return pick_up


# %% attribution of a motion that does not reach its goal


def test_a_failing_motion_is_attributed_to_a_motion_node(mutable_model_world):
    world, view, context = mutable_model_world
    root = sequential([unreachable_pick_up(world, view, context)], context)

    with pytest.raises(MotionDidNotFinish) as raised:
        with simulated_robot:
            root.perform()

    assert isinstance(raised.value.node, MotionNode)


def test_the_attributed_motion_node_belongs_to_the_failing_plan(mutable_model_world):
    world, view, context = mutable_model_world
    root = sequential([unreachable_pick_up(world, view, context)], context)

    with pytest.raises(MotionDidNotFinish) as raised:
        with simulated_robot:
            root.perform()

    assert raised.value.node in root.plan.nodes


def test_an_attributed_failure_resolves_the_action_owning_the_motion(
    mutable_model_world,
):
    """
    The resolved action is the innermost one, so a failure inside a composite action
    names the sub-action that owns the motion rather than the whole composite.
    """
    world, view, context = mutable_model_world
    root = sequential([unreachable_pick_up(world, view, context)], context)

    with pytest.raises(MotionDidNotFinish) as raised:
        with simulated_robot:
            root.perform()

    action_node = raised.value.action_node
    assert isinstance(action_node, ActionNode)
    assert action_node in raised.value.node.path
    assert any(
        isinstance(ancestor, ActionNode) and isinstance(ancestor.action, PickUpAction)
        for ancestor in raised.value.node.path
    )


def test_an_attributed_failure_resolves_its_context(mutable_model_world):
    world, view, context = mutable_model_world
    root = sequential([unreachable_pick_up(world, view, context)], context)

    with pytest.raises(MotionDidNotFinish) as raised:
        with simulated_robot:
            root.perform()

    assert raised.value.context is context


def test_the_failed_motions_are_kept_for_diagnostics(mutable_model_world):
    world, view, context = mutable_model_world
    root = sequential([unreachable_pick_up(world, view, context)], context)

    with pytest.raises(MotionDidNotFinish) as raised:
        with simulated_robot:
            root.perform()

    assert raised.value.failed_motions
