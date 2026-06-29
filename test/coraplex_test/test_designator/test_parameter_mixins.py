from coraplex.datastructures.enums import (
    ApproachDirection,
    Arms,
    VerticalAlignment,
)
from coraplex.datastructures.grasp import GraspDescription
from coraplex.robot_plans.actions.core.container import OpenAction
from coraplex.robot_plans.actions.core.navigation import NavigateAction
from coraplex.robot_plans.actions.core.pick_up import PickUpAction
from coraplex.robot_plans.motions.gripper import MoveGripperMotion
from coraplex.robot_plans.parameter_mixins import (
    GraspParameters,
    GripperActuationParameters,
    GripperStateSet,
    HandleOperatedOn,
    HandleOperationParameters,
    NavigationParameters,
    ObjectActedOn,
    ObjectManipulationParameters,
    UsedArm,
    UsedGraspDescription,
)
from semantic_digital_twin.datastructures.definitions import GripperState
from semantic_digital_twin.semantic_annotations.semantic_annotations import Handle


def test_action_inherits_parameter_mixins():
    assert issubclass(PickUpAction, UsedArm)
    assert issubclass(PickUpAction, ObjectActedOn)
    assert issubclass(PickUpAction, UsedGraspDescription)


def test_bundle_mixins_compose_leaf_mixins():
    # bundles inherit their constituent leaf mixins ...
    assert issubclass(GraspParameters, ObjectManipulationParameters)
    assert issubclass(GraspParameters, UsedGraspDescription)
    assert issubclass(ObjectManipulationParameters, ObjectActedOn)
    assert issubclass(ObjectManipulationParameters, UsedArm)
    assert issubclass(GripperActuationParameters, GripperStateSet)
    assert issubclass(HandleOperationParameters, HandleOperatedOn)


def test_classes_inherit_bundle_mixins():
    # ... and concrete classes inherit the bundles while still exposing the leaf interface.
    assert issubclass(PickUpAction, GraspParameters)
    assert issubclass(PickUpAction, ObjectManipulationParameters)
    assert issubclass(PickUpAction, UsedArm)
    assert issubclass(NavigateAction, NavigationParameters)
    assert issubclass(OpenAction, HandleOperationParameters)
    assert issubclass(MoveGripperMotion, GripperActuationParameters)


def test_pick_up_action_exposes_unified_parameters(immutable_model_world):
    world, view, context = immutable_model_world
    grasp_description = GraspDescription(
        ApproachDirection.FRONT,
        VerticalAlignment.NoAlignment,
        view.left_arm.end_effector,
    )
    body = world.get_body_by_name("milk.stl")
    action = PickUpAction(
        object_designator=body, arm=Arms.LEFT, grasp_description=grasp_description
    )

    assert action.arm is Arms.LEFT
    assert action.object_designator is body
    assert action.grasp_description is grasp_description

    parameters = action.designator_parameter
    assert parameters["arm"] is Arms.LEFT
    assert parameters["object_designator"] is body
    assert parameters["grasp_description"] is grasp_description


def test_combined_mixins_instantiate_without_ordering_error(immutable_model_world):
    world, view, context = immutable_model_world
    grasp_description = GraspDescription(
        ApproachDirection.FRONT,
        VerticalAlignment.NoAlignment,
        view.left_arm.end_effector,
    )
    action = PickUpAction(
        object_designator=world.get_body_by_name("milk.stl"),
        arm=Arms.LEFT,
        grasp_description=grasp_description,
    )
    assert {"arm", "object_designator", "grasp_description"} <= set(
        action.designator_parameter
    )


def test_move_gripper_motion_exposes_unified_arm():
    motion = MoveGripperMotion(motion=GripperState.OPEN, arm=Arms.LEFT)

    assert motion.arm is Arms.LEFT
    assert motion.motion is GripperState.OPEN
    assert issubclass(MoveGripperMotion, UsedArm)
    assert issubclass(MoveGripperMotion, GripperStateSet)


def test_open_action_operates_on_handle(immutable_model_world):
    world, view, context = immutable_model_world
    handle = Handle(root=world.get_body_by_name("handle_cab10_m"))
    action = OpenAction(handle=handle, arm=Arms.LEFT)

    assert action.handle is handle
    assert action.handle.root is world.get_body_by_name("handle_cab10_m")
    assert issubclass(OpenAction, HandleOperatedOn)
    assert issubclass(OpenAction, UsedArm)
