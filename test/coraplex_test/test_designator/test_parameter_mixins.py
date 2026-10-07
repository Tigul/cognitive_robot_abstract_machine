import dataclasses

from coraplex.robot_plans.actions.core.container import OpenAction
from coraplex.robot_plans.actions.core.navigation import NavigateAction
from coraplex.robot_plans.actions.core.pick_up import PickUpAction
from coraplex.robot_plans.actions.core.placing import PlaceAction
from coraplex.robot_plans.actions.core.robot_body import (
    MoveManipulatorAction,
    ParkArmsAction,
    SetGripperAction,
)
from coraplex.robot_plans.motions.gripper import (
    MoveGripperMotion,
    MoveManipulatorMotion,
    MoveToolCenterPointMotion,
)
from coraplex.robot_plans.mixins import (
    ArmDrivenToGoal,
    EndEffectorPoseParameters,
    GraspParameters,
    GripperActuationParameters,
    GripperStallTolerated,
    GripperStateSet,
    HandleOperatedOn,
    HasApproachesGraspPoses,
    HandleOperationParameters,
    ObjectActedOn,
    PlaceTuningParameters,
    TargetLocationMovedTo,
    ToolCenterPointGoalThresholds,
    UsedArm,
    UsedEndEffector,
    UsedGrasp,
)
from semantic_digital_twin.datastructures.definitions import GripperState
from semantic_digital_twin.semantic_annotations.semantic_annotations import Handle, Milk
from semantic_digital_twin.spatial_types.spatial_types import Pose


def test_action_inherits_parameter_mixins():
    assert issubclass(PickUpAction, UsedArm)
    assert issubclass(PickUpAction, UsedGrasp)


def test_bundle_mixins_compose_leaf_mixins():
    # bundles inherit their constituent leaf mixins ...
    assert issubclass(GraspParameters, UsedGrasp)
    assert issubclass(GraspParameters, UsedArm)
    assert issubclass(GraspParameters, HasApproachesGraspPoses)
    assert issubclass(GraspParameters, ArmDrivenToGoal)
    assert issubclass(GripperActuationParameters, GripperStateSet)
    assert issubclass(GripperActuationParameters, UsedEndEffector)
    assert issubclass(HandleOperationParameters, HandleOperatedOn)
    assert issubclass(HandleOperationParameters, UsedArm)


def test_classes_inherit_bundle_mixins():
    # ... and concrete classes inherit the bundles while still exposing the leaf interface.
    assert issubclass(PickUpAction, GraspParameters)
    assert issubclass(PickUpAction, UsedArm)
    assert issubclass(NavigateAction, TargetLocationMovedTo)
    assert issubclass(OpenAction, HandleOperationParameters)
    assert issubclass(MoveGripperMotion, GripperActuationParameters)


def test_pick_up_action_takes_its_grasp_and_arm(pr2_apartment_context):
    world, view, context = pr2_apartment_context
    milk = world.get_semantic_annotations_by_type(Milk)[0]
    grasp = milk.grasp_candidates()[0]
    arm = context.robot.left_arm

    action = PickUpAction(grasp=grasp, arm=arm)

    assert action.arm is arm
    assert action.grasp is grasp
    assert action.grasp.graspable is milk

    parameters = action.designator_parameter
    assert parameters["arm"] is arm
    assert parameters["grasp"] is grasp


def test_move_gripper_motion_exposes_its_end_effector(pr2_apartment_context):
    world, view, context = pr2_apartment_context
    end_effector = context.robot.left_arm.end_effector

    motion = MoveGripperMotion(motion=GripperState.OPEN, end_effector=end_effector)

    assert motion.end_effector is end_effector
    assert motion.motion is GripperState.OPEN
    assert issubclass(MoveGripperMotion, UsedEndEffector)
    assert issubclass(MoveGripperMotion, GripperStateSet)


def test_open_action_operates_on_handle(pr2_apartment_context):
    world, view, context = pr2_apartment_context
    handle = world.get_semantic_annotations_by_type(Handle)[0]
    arm = context.robot.left_arm

    action = OpenAction(handle=handle, arm=arm)

    assert action.handle is handle
    assert action.arm is arm
    assert issubclass(OpenAction, HandleOperatedOn)
    assert issubclass(OpenAction, UsedArm)


# %% runtime resolution of the inherited field types


def field_types(designator_type: type) -> dict:
    """
    :return: The declared type of each of the designator's dataclass fields, by name.
    """
    return {
        parameter.name: parameter.type
        for parameter in dataclasses.fields(designator_type)
    }


def test_inherited_parameters_keep_their_declared_types():
    """
    The fields a designator inherits from the mixins carry the types the mixins declare
    as type objects, not as strings the designator's own module would have to resolve.
    """
    hints = field_types(PlaceAction)

    assert (
        hints["object_designator"] is ObjectActedOn.__annotations__["object_designator"]
    )
    assert (
        hints["target_location"]
        is TargetLocationMovedTo.__annotations__["target_location"]
    )
    assert (
        hints["placing_linear_velocity"]
        == PlaceTuningParameters.__annotations__["placing_linear_velocity"]
    )


# %% tuning carried by the behaviour it tunes


def test_behaviours_driving_a_tool_center_point_carry_their_own_tolerances():
    """
    The goal tolerances belong to driving an arm to a goal, so a behaviour that does
    that has them without naming a second mixin.
    """
    assert issubclass(MoveToolCenterPointMotion, ArmDrivenToGoal)
    assert issubclass(ArmDrivenToGoal, UsedArm)
    assert issubclass(ArmDrivenToGoal, ToolCenterPointGoalThresholds)

    hints = field_types(MoveToolCenterPointMotion)
    assert (
        hints["position_threshold"]
        == ToolCenterPointGoalThresholds.__annotations__["position_threshold"]
    )


def test_behaviours_without_a_tool_center_point_goal_have_no_tolerances():
    """
    Parking the arms and setting a gripper drive no tool center point, so folding the
    tolerances into the arm parameters must not reach them.
    """
    assert not issubclass(ParkArmsAction, ToolCenterPointGoalThresholds)
    assert "position_threshold" not in {
        parameter.name for parameter in dataclasses.fields(ParkArmsAction)
    }

    assert issubclass(SetGripperAction, GripperActuationParameters)
    assert not issubclass(SetGripperAction, GripperStallTolerated)
    assert "tolerate_stall" not in {
        parameter.name for parameter in dataclasses.fields(SetGripperAction)
    }


def test_only_the_gripper_motion_tolerates_a_stall():
    """
    Stalling is something the motion commanding the fingers tolerates, so the stall
    parameters sit on the gripper actuation the motion uses rather than beside it.
    """
    assert issubclass(MoveGripperMotion, GripperStallTolerated)
    assert issubclass(GripperStallTolerated, GripperActuationParameters)


def test_manipulator_action_and_motion_share_their_end_effector_pose_parameters(
    pr2_apartment_context,
):
    """
    The motion takes the same end effector, target pose, collision permission and
    tolerances as the action that commands it.
    """
    world, view, context = pr2_apartment_context
    assert issubclass(MoveManipulatorAction, EndEffectorPoseParameters)
    assert issubclass(MoveManipulatorMotion, EndEffectorPoseParameters)

    end_effector = context.robot.left_arm.end_effector
    target_pose = Pose(reference_frame=world.root)
    motion = MoveManipulatorMotion(end_effector=end_effector, target_pose=target_pose)

    assert motion.end_effector is end_effector
    assert motion.target_pose is target_pose
