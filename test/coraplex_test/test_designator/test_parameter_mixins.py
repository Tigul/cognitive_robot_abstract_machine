import dataclasses

from coraplex.robot_plans.actions.core.container import OpenAction
from coraplex.robot_plans.actions.core.navigation import (
    NavigateAction,
    PathPlanningNavigateAction,
)
from coraplex.robot_plans.actions.core.pick_up import PickUpAction
from coraplex.robot_plans.actions.core.placing import PlaceAction
from coraplex.robot_plans.actions.core.robot_body import (
    MoveManipulatorAction,
    ParkArmsAction,
    SetGripperAction,
)
from coraplex.robot_plans.motions.base import BaseMotion
from coraplex.robot_plans.motions.navigation import MoveMotion
from coraplex.robot_plans.motions.robot_body import MoveJointsMotion
from coraplex.robot_plans.motions.gripper import (
    MoveGripperMotion,
    MoveManipulatorMotion,
    MoveToolCenterPointMotion,
)
from coraplex.robot_plans import mixins
from coraplex.robot_plans.mixins import (
    DesignatorParameterMixin,
    HasArmParameter,
    HasEndEffectorParameter,
    HasGoalThresholdParameters,
    HasGraspableObjectParameter,
    HasGraspApproachParameters,
    HasGraspCandidateParameter,
    HasGripperStateParameter,
    HasHandleParameter,
    HasNavigationTargetParameter,
    HasPlaceTuningParameters,
    HasPlacementTargetParameter,
    HasStallToleranceParameters,
)
from semantic_digital_twin.datastructures.definitions import GripperState
from semantic_digital_twin.semantic_annotations.semantic_annotations import Handle, Milk
from semantic_digital_twin.spatial_types.spatial_types import Pose


def parameter_mixins(designator_type: type) -> set[type]:
    """
    :return: The parameter mixins the designator inherits.
    """
    return {
        base
        for base in designator_type.__mro__
        if base.__module__ == mixins.__name__
        and issubclass(base, DesignatorParameterMixin)
        and base is not DesignatorParameterMixin
    }


def test_action_inherits_parameter_mixins():
    assert parameter_mixins(PickUpAction) >= {
        HasArmParameter,
        HasGraspCandidateParameter,
        HasGoalThresholdParameters,
        HasGraspApproachParameters,
    }


def test_parameter_mixins_do_not_stack_other_parameter_mixins():
    """
    Each parameter mixin declares its own parameters only, so a designator names every
    parameter mixin it has.
    """
    for mixin in vars(mixins).values():
        if not isinstance(mixin, type) or not issubclass(
            mixin, DesignatorParameterMixin
        ):
            continue
        if mixin is DesignatorParameterMixin:
            continue
        assert mixin.__bases__ == (DesignatorParameterMixin,)


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
    assert issubclass(MoveGripperMotion, HasEndEffectorParameter)
    assert issubclass(MoveGripperMotion, HasGripperStateParameter)


def test_open_action_operates_on_handle(pr2_apartment_context):
    world, view, context = pr2_apartment_context
    handle = world.get_semantic_annotations_by_type(Handle)[0]
    arm = context.robot.left_arm

    action = OpenAction(handle=handle, arm=arm)

    assert action.handle is handle
    assert action.arm is arm
    assert issubclass(OpenAction, HasHandleParameter)
    assert issubclass(OpenAction, HasArmParameter)


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
        hints["object_designator"]
        is HasGraspableObjectParameter.__annotations__["object_designator"]
    )
    assert (
        hints["target_location"]
        is HasPlacementTargetParameter.__annotations__["target_location"]
    )
    assert (
        hints["placing_linear_velocity"]
        == HasPlaceTuningParameters.__annotations__["placing_linear_velocity"]
    )


# %% tuning carried by the behaviour it tunes


def test_behaviours_driving_a_tool_center_point_carry_their_own_tolerances():
    """
    The goal tolerances belong to driving an arm to a goal, so a behaviour that does
    that has them without naming a second mixin.
    """
    assert issubclass(MoveToolCenterPointMotion, HasArmParameter)
    assert issubclass(MoveToolCenterPointMotion, HasGoalThresholdParameters)

    hints = field_types(MoveToolCenterPointMotion)
    assert (
        hints["position_threshold"]
        == HasGoalThresholdParameters.__annotations__["position_threshold"]
    )


def test_behaviours_without_a_tool_center_point_goal_have_no_tolerances():
    """
    Parking the arms and setting a gripper drive no tool center point, so folding the
    tolerances into the arm parameters must not reach them.
    """
    assert not issubclass(ParkArmsAction, HasGoalThresholdParameters)
    assert "position_threshold" not in {
        parameter.name for parameter in dataclasses.fields(ParkArmsAction)
    }

    assert issubclass(SetGripperAction, HasGripperStateParameter)
    assert not issubclass(SetGripperAction, HasStallToleranceParameters)
    assert "tolerate_stall" not in {
        parameter.name for parameter in dataclasses.fields(SetGripperAction)
    }


def test_only_the_gripper_motion_tolerates_a_stall():
    """
    Stalling is something the motion commanding the fingers tolerates, not the action
    that sets the gripper.
    """
    assert issubclass(MoveGripperMotion, HasStallToleranceParameters)
    assert not issubclass(SetGripperAction, HasStallToleranceParameters)


def test_manipulator_action_and_motion_share_their_end_effector_pose_parameters(
    pr2_apartment_context,
):
    """
    The motion takes the same end effector, target pose, collision permission and
    tolerances as the action that commands it.
    """
    world, view, context = pr2_apartment_context
    assert parameter_mixins(MoveManipulatorAction) == parameter_mixins(
        MoveManipulatorMotion
    )

    end_effector = context.robot.left_arm.end_effector
    target_pose = Pose(reference_frame=world.root)
    motion = MoveManipulatorMotion(end_effector=end_effector, target_pose=target_pose)

    assert motion.end_effector is end_effector
    assert motion.target_pose is target_pose


def test_navigation_behaviours_take_a_planar_target():
    """
    Driving the base, whether straight, along a planned path or as a single motion,
    takes the same planar target.
    """
    for navigation in (NavigateAction, PathPlanningNavigateAction, MoveMotion):
        assert issubclass(navigation, HasNavigationTargetParameter)
        assert (
            field_types(navigation)["target_location"]
            == HasNavigationTargetParameter.__annotations__["target_location"]
        )


def test_move_joints_motion_takes_only_joint_targets_and_a_velocity_cap():
    """
    Moving joints needs the joints, their target positions and an optional speed cap; it
    carries no end effector alignment.
    """
    joint_fields = {
        parameter.name for parameter in dataclasses.fields(MoveJointsMotion)
    } - {parameter.name for parameter in dataclasses.fields(BaseMotion)}

    assert joint_fields == {"names", "positions", "max_joint_velocity"}
