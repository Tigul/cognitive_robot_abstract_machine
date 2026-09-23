"""
Reusable parameters for actions and motions: the inputs a behaviour is given and the
knobs that tune how it carries them out.

.. note:: Annotations here are evaluated at class creation, so this module must not
    defer them with ``from __future__ import annotations``.
    :meth:`~coraplex.plans.designator.Designator.get_type_hints` resolves a designator's
    inherited fields against the *concrete* class's module, which does not import the
    types declared here.
"""

from dataclasses import dataclass, field

from typing_extensions import Optional

from coraplex.config.action_conf import ActionConfig
from coraplex.datastructures.enums import Arms, MovementType
from coraplex.datastructures.grasp import GraspDescription
from semantic_digital_twin.datastructures.definitions import GripperState, TorsoState
from semantic_digital_twin.robots.robot_parts import Camera, EndEffector
from semantic_digital_twin.semantic_annotations.mixins import IsGraspable
from semantic_digital_twin.semantic_annotations.semantic_annotations import Handle
from semantic_digital_twin.spatial_types.spatial_types import Pose
from semantic_digital_twin.world_description.world_entity import (
    SemanticAnnotation,
)

# %% input parameters


@dataclass(eq=False)
class UsedArm:
    """
    Mixin for behaviours that operate one of the robot's arms.
    """

    arm: Arms = field(kw_only=True)
    """
    The arm the behaviour uses.
    """


@dataclass(eq=False)
class ObjectActedOn:
    """
    Mixin for behaviours that act on a single graspable object.
    """

    target_object: IsGraspable = field(kw_only=True)
    """
    The graspable annotation the behaviour acts on; its :attr:`root` body is used where the
    underlying kinematic body is required.
    """


@dataclass(eq=False)
class HandleOperatedOn:
    """
    Mixin for behaviours that grasp and articulate a handle, such as opening or closing a
    container.
    """

    handle: Handle = field(kw_only=True)
    """
    The handle annotation the behaviour operates; its :attr:`root` body is used where the
    underlying kinematic body is required.
    """


@dataclass(eq=False)
class UsedGraspDescription:
    """
    Mixin for behaviours that approach a body with a defined grasp.
    """

    grasp_description: GraspDescription = field(kw_only=True)
    """
    The grasp the behaviour uses to approach the body.
    """


@dataclass(eq=False)
class JointStatesKept:
    """
    Mixin for behaviours that can preserve the robot's joint states while moving the base.
    """

    keep_joint_states: bool = field(default=False, kw_only=True)
    """
    Whether the joint states are kept unchanged during the behaviour.
    """


@dataclass(eq=False)
class GripperCollisionAllowed:
    """
    Mixin for behaviours that may permit the gripper to collide with the environment.
    """

    allow_gripper_collision: Optional[bool] = field(default=None, kw_only=True)
    """
    Whether the gripper is allowed to collide during the behaviour.
    """


@dataclass(eq=False)
class TargetLocationMovedTo:
    """
    Mixin for behaviours that move the robot or an object to a destination pose.
    """

    target_location: Pose = field(kw_only=True)
    """
    The destination pose the behaviour moves to.
    """


@dataclass(eq=False)
class TargetPoseReached:
    """
    Mixin for behaviours that drive an end effector to a target pose.
    """

    target_pose: Pose = field(kw_only=True)
    """
    The pose the end effector reaches.
    """


@dataclass(eq=False)
class TargetLookedAt:
    """
    Mixin for behaviours that orient the robot toward a pose.
    """

    look_at_target: Pose = field(kw_only=True)
    """
    The pose the behaviour orients toward.
    """


@dataclass(eq=False)
class StandingPositionMovedTo:
    """
    Mixin for behaviours that first move the robot's base to a standing pose.
    """

    standing_position: Pose = field(kw_only=True)
    """
    The pose the robot stands at before manipulating.
    """


@dataclass(eq=False)
class UsedMovementType:
    """
    Mixin for behaviours whose Cartesian motion follows a selectable movement type.
    """

    movement_type: MovementType = field(default=MovementType.CARTESIAN, kw_only=True)
    """
    The type of Cartesian movement the behaviour performs.
    """


@dataclass(eq=False)
class GripperStateSet:
    """
    Mixin for behaviours that set the gripper to an open or closed state.
    """

    motion: GripperState = field(kw_only=True)
    """
    The gripper state the behaviour sets.
    """


@dataclass(eq=False)
class UsedEndEffector:
    """
    Mixin for behaviours that act through a specific end effector.
    """

    end_effector: EndEffector = field(kw_only=True)
    """
    The end effector the behaviour uses.
    """


@dataclass(eq=False)
class UsedCamera:
    """
    Mixin for behaviours that point a camera.
    """

    camera: Optional[Camera] = field(default=None, kw_only=True)
    """
    The camera the behaviour points; ``None`` selects the robot's default camera.
    """


@dataclass(eq=False)
class UsedTool:
    """
    Mixin for behaviours that manipulate an object with a held tool.
    """

    tool: SemanticAnnotation = field(kw_only=True)
    """
    The tool the behaviour uses.
    """


@dataclass(eq=False)
class UsedTechnique:
    """
    Mixin for behaviours that can be parametrised by a named technique.
    """

    technique: Optional[str] = field(default=None, kw_only=True)
    """
    The technique the behaviour applies.
    """


@dataclass(eq=False)
class UsedGraspingPreposeDistance:
    """
    Mixin for behaviours that approach a handle from a prepose offset before grasping.
    """

    grasping_prepose_distance: float = field(
        default=ActionConfig.grasping_prepose_distance, kw_only=True
    )
    """
    The distance in meters between the gripper and the handle before approaching to grasp.
    """


@dataclass(eq=False)
class PoseSequenceReversed:
    """
    Mixin for behaviours whose pose sequence can be reversed to move away instead of toward
    the target.
    """

    reverse_pose_sequence: bool = field(default=False, kw_only=True)
    """
    Whether the pose sequence is reversed.
    """


@dataclass(eq=False)
class TorsoStateSet:
    """
    Mixin for behaviours that set the torso to a defined state.
    """

    torso_state: TorsoState = field(kw_only=True)
    """
    The torso state the behaviour sets.
    """


@dataclass(eq=False)
class LinkAlignmentApplied:
    """
    Mixin for behaviours that can align an end-effector link with a goal axis.

    .. note:: The directional axes differ in representation between behaviours (axis identifier
        versus normal vector) and therefore stay declared on the concrete classes.
    """

    align: Optional[bool] = field(default=False, kw_only=True)
    """
    Whether the end effector is aligned with a goal axis.
    """

    tip_link: Optional[str] = field(default=None, kw_only=True)
    """
    The name of the tip link to align.
    """

    root_link: Optional[str] = field(default=None, kw_only=True)
    """
    The name of the root link to align against.
    """


# %% higher-order input parameters


@dataclass(eq=False)
class ObjectManipulationParameters(ObjectActedOn, UsedArm):
    """
    Bundle of the parameters shared by every behaviour that manipulates an object with an arm:
    the object and the arm acting on it.
    """


@dataclass(eq=False)
class GraspParameters(ObjectManipulationParameters, UsedGraspDescription):
    """
    Bundle of the parameters for grasping an object: the object, the arm, and the grasp the
    arm approaches it with.
    """


@dataclass(eq=False)
class ToolUsageParameters(ObjectManipulationParameters, UsedTool, UsedTechnique):
    """
    Bundle of the parameters for acting on an object with a held tool: the object, the arm,
    the tool, and the technique.
    """


@dataclass(eq=False)
class MobileManipulationParameters(
    ObjectManipulationParameters, StandingPositionMovedTo, JointStatesKept
):
    """
    Bundle of the parameters for manipulating an object after first driving the base to a
    standing pose: the object, the arm, the standing pose, and whether joint states are kept.
    """


@dataclass(eq=False)
class HandleOperationParameters(HandleOperatedOn, UsedArm):
    """
    Bundle of the parameters for articulating a handle with an arm: the handle and the arm.
    """


@dataclass(eq=False)
class EndEffectorPoseParameters(
    UsedEndEffector, TargetPoseReached, GripperCollisionAllowed
):
    """
    Bundle of the parameters for driving an end effector to a target pose: the end effector,
    the target pose, and whether gripper collision is allowed.
    """


@dataclass(eq=False)
class CameraTargetParameters(UsedCamera, TargetLookedAt):
    """
    Bundle of the parameters for pointing a camera at a target: the camera and the pose it is
    pointed at.
    """


@dataclass(eq=False)
class NavigationParameters(TargetLocationMovedTo, JointStatesKept):
    """
    Bundle of the parameters for navigating the base to a destination: the destination and
    whether joint states are kept.
    """


@dataclass(eq=False)
class GripperActuationParameters(GripperStateSet, UsedArm):
    """
    Bundle of the parameters for setting a gripper to an open or closed state: the gripper
    state and the arm.
    """


# %% tuning parameters


@dataclass
class HasMaxJointVelocity:
    """
    Adds an optional joint velocity cap to an action or motion.
    """

    max_joint_velocity: Optional[float] = field(default=None, kw_only=True)
    """
    Maximum joint velocity (in rad/s or m/s, per joint), enforced via
    :class:`~giskardpy.motion_statechart.tasks.joint_tasks.JointVelocityLimit`. ``None``
    leaves the speed unconstrained.
    """


@dataclass
class HasApproachVelocity:
    """
    Adds an optional pre-approach speed to an action that reaches towards a target
    before its main motion.

    Shared by :class:`~coraplex.robot_plans.actions.core.pick_up.ReachAction` and
    :class:`~coraplex.robot_plans.actions.core.pick_up.PickUpAction`, since a pick-up's
    reach is itself a :class:`ReachAction` and forwards this same value to it.
    """

    pre_approach_linear_velocity: Optional[float] = field(default=None, kw_only=True)
    """
    Maximum linear speed (in m/s) for the initial pre-pose approach, enforced via
    :class:`~giskardpy.motion_statechart.tasks.cartesian_tasks.CartesianPositionVelocityLimit`.
    ``None`` leaves the speed unconstrained.
    """


@dataclass
class HasGraspDetectionThreshold:
    """
    Adds a grasp-detection sensitivity threshold to an action that checks whether an
    object is held between the gripper's fingers.

    Shared by :class:`~coraplex.robot_plans.actions.core.pick_up.ReachAction`,
    :class:`~coraplex.robot_plans.actions.core.pick_up.PickUpAction` and
    :class:`~coraplex.robot_plans.actions.core.placing.PlaceAction`.
    """

    grasp_detection_threshold: float = field(default=0.9, kw_only=True)
    """
    Minimum fraction of sampled rays between the gripper's fingers that must hit the
    target object for it to count as grasped/held (see
    :func:`~semantic_digital_twin.reasoning.robot_predicates.is_body_gripped`).
    """


@dataclass
class ReachTuningParameters(HasApproachVelocity):
    """
    Tunable approach speeds for :class:`~coraplex.robot_plans.actions.core.pick_up.ReachAction`.
    """

    final_approach_linear_velocity: Optional[float] = field(default=None, kw_only=True)
    """
    Maximum linear speed (in m/s) for the final approach onto the target pose, enforced
    via
    :class:`~giskardpy.motion_statechart.tasks.cartesian_tasks.CartesianPositionVelocityLimit`.
    ``None`` leaves the speed unconstrained.
    """


@dataclass
class PickUpTuningParameters(ReachTuningParameters):
    """
    Tunable grasp speeds and target-object friction for
    :class:`~coraplex.robot_plans.actions.core.pick_up.PickUpAction`.

    Extends :class:`ReachTuningParameters` rather than just :class:`HasApproachVelocity`:
    :class:`~coraplex.robot_plans.actions.core.pick_up.PickUpAction` forwards both
    ``pre_approach_linear_velocity`` and ``final_approach_linear_velocity`` verbatim to
    the internal :class:`~coraplex.robot_plans.actions.core.pick_up.ReachAction` it
    builds, so both fields are literally the same value under the same name in both
    places rather than two similarly-named-but-distinct fields.
    """

    grasp_closing_velocity: Optional[float] = field(default=None, kw_only=True)
    """
    Maximum finger joint velocity (in m/s) used while closing onto the object, enforced
    via
    :class:`~giskardpy.motion_statechart.tasks.joint_tasks.JointVelocityLimit`. ``None``
    leaves the speed unconstrained.
    """

    lift_linear_velocity: Optional[float] = field(default=None, kw_only=True)
    """
    Maximum linear speed (in m/s) for lifting the object clear of the table after
    grasping, enforced via
    :class:`~giskardpy.motion_statechart.tasks.cartesian_tasks.CartesianPositionVelocityLimit`.
    ``None`` leaves the speed unconstrained.
    """

    grasp_stall_minimum_time: Optional[float] = field(default=None, kw_only=True)
    """
    Minimum stall dwell time (in seconds, see
    :attr:`~coraplex.robot_plans.motions.gripper.MoveGripperMotion.stall_minimum_time`)
    for the CLOSE motion. ``None`` keeps the default.
    """

    object_friction: Optional[float] = field(default=None, kw_only=True)
    """
    Sliding friction coefficient to apply to the target object's geom before this pick,
    overriding the world's default. Not consumed by this action itself -- applying it is
    the caller's responsibility (see
    :meth:`~physics_simulators.mujoco_simulator.MujocoSimulator.set_geom_friction`);
    recorded here for persistence. ``None`` leaves the friction untouched.
    """


@dataclass
class PlaceTuningParameters:
    """
    Tunable transport/placing/release speeds for
    :class:`~coraplex.robot_plans.actions.core.placing.PlaceAction`.
    """

    placing_linear_velocity: Optional[float] = field(default=None, kw_only=True)
    """
    Maximum linear speed (in m/s) for the final descent onto the target location,
    enforced via
    :class:`~giskardpy.motion_statechart.tasks.cartesian_tasks.CartesianPositionVelocityLimit`.
    ``None`` leaves the speed unconstrained.
    """

    transport_linear_velocity: Optional[float] = field(default=None, kw_only=True)
    """
    Maximum linear speed (in m/s) for carrying the held object above the target
    location, before the final descent, enforced via
    :class:`~giskardpy.motion_statechart.tasks.cartesian_tasks.CartesianPositionVelocityLimit`.
    ``None`` leaves the speed unconstrained.
    """

    release_opening_velocity: Optional[float] = field(default=None, kw_only=True)
    """
    Maximum finger joint velocity (in m/s) used while opening the gripper to release
    the object, enforced via
    :class:`~giskardpy.motion_statechart.tasks.joint_tasks.JointVelocityLimit`. ``None``
    leaves the speed unconstrained.
    """

    retract_linear_velocity: Optional[float] = field(default=None, kw_only=True)
    """
    Maximum linear speed (in m/s) for retracting the end effector away from the placed
    object, enforced via
    :class:`~giskardpy.motion_statechart.tasks.cartesian_tasks.CartesianPositionVelocityLimit`.
    ``None`` leaves the speed unconstrained.
    """


@dataclass
class GripperStallToleranceParameters:
    """
    Adds an optional finger speed and stall-tolerance to a gripper open/close motion.
    """

    finger_velocity: Optional[float] = field(default=None, kw_only=True)
    """
    Maximum finger joint velocity (in m/s), enforced via
    :class:`~giskardpy.motion_statechart.tasks.joint_tasks.JointVelocityLimit`. ``None``
    leaves the speed unconstrained.
    """

    stall_minimum_time: Optional[float] = field(default=None, kw_only=True)
    """
    Minimum stall dwell time (in seconds, see
    :attr:`~giskardpy.motion_statechart.monitors.monitors.LocalMinimumReached.minimum_time`)
    to command. Only meaningful when :attr:`tolerate_stall` is True. ``None`` keeps the
    default.
    """

    tolerate_stall: bool = field(default=False, kw_only=True)
    """
    Whether this motion is also considered done once the fingers' velocities settle
    near zero, even without reaching their nominal target position -- checked via a
    separate :class:`~giskardpy.motion_statechart.monitors.monitors.LocalMinimumReached`
    monitor alongside the goal, not by the goal's own observation, since stalling does
    not mean the goal itself was reached.
    """


@dataclass
class CartesianVelocityLimitParameters:
    """
    Adds an optional linear and angular speed cap to a Cartesian tool-center-point
    motion.
    """

    max_linear_velocity: Optional[float] = field(default=None, kw_only=True)
    """
    Maximum linear speed (in m/s) of the tool center point, enforced via
    :class:`~giskardpy.motion_statechart.tasks.cartesian_tasks.CartesianPositionVelocityLimit`.
    ``None`` leaves the linear speed unconstrained (other than the robot's own hardware
    limits).
    """

    max_angular_velocity: Optional[float] = field(default=None, kw_only=True)
    """
    Maximum angular speed (in rad/s) of the tool center point, enforced via
    :class:`~giskardpy.motion_statechart.tasks.cartesian_tasks.CartesianRotationVelocityLimit`.
    Only meaningful for :class:`~giskardpy.motion_statechart.tasks.cartesian_tasks.CartesianPose`
    (i.e. when not :attr:`~coraplex.datastructures.enums.MovementType.TRANSLATION`).
    ``None`` leaves the angular speed unconstrained.
    """


@dataclass
class HasTcpGoalThresholds:
    """
    Adds optional tool-center-point goal-achievement thresholds to a motion, falling
    back to :attr:`~coraplex.datastructures.dataclasses.Context.motion_tolerances` when
    left unset.

    Meant to be mixed into a :class:`~coraplex.robot_plans.motions.base.BaseMotion`
    subclass, whose ``context`` the resolver methods below rely on.
    """

    position_threshold: Optional[float] = field(default=None, kw_only=True)
    """
    Distance threshold in meters for goal achievement. ``None`` falls back to
    :attr:`~coraplex.datastructures.dataclasses.MotionToleranceConfig.default_tcp_position_threshold`.
    """

    orientation_threshold: Optional[float] = field(default=None, kw_only=True)
    """
    Rotation threshold in rad for goal achievement. ``None`` falls back to
    :attr:`~coraplex.datastructures.dataclasses.MotionToleranceConfig.tool_orientation_threshold`.
    """

    def resolved_position_threshold(self) -> float:
        """
        :return: :attr:`position_threshold` if set, otherwise the context's default.
        """
        if self.position_threshold is not None:
            return self.position_threshold
        return self.context.motion_tolerances.default_tcp_position_threshold

    def resolved_orientation_threshold(self) -> float:
        """
        :return: :attr:`orientation_threshold` if set, otherwise the context's default.
        """
        if self.orientation_threshold is not None:
            return self.orientation_threshold
        return self.context.motion_tolerances.tool_orientation_threshold
