from __future__ import annotations

import logging
from dataclasses import dataclass, field

from typing_extensions import Any, Dict, Optional

from coraplex.locations.pose_validator import AreReachableBy, IsObjectReachableBy
from coraplex.plans.attachment_nodes import AttachNode
from coraplex.plans.plan_node import PlanNode
from krrood.entity_query_language.core.variable import Variable
from krrood.entity_query_language.factories import (
    and_,
    or_,
    not_,
    variable_from,
    ConditionType,
)
from coraplex.datastructures.dataclasses import Context
from coraplex.datastructures.enums import (
    Arms,
    MovementType,
)
from coraplex.datastructures.grasp import GraspDescription
from coraplex.plans.factories import sequential
from coraplex.querying.predicates import GripperIsFree
from coraplex.robot_plans.actions.base import ActionDescription
from coraplex.robot_plans.mixins import (
    HasGraspDetectionThreshold,
    PickUpTuningParameters,
    ReachTuningParameters,
)
from coraplex.robot_plans.motions.gripper import (
    MoveGripperMotion,
    MoveToolCenterPointMotion,
)
from coraplex.robot_plans.parameter_mixins import (
    GraspParameters,
    PoseSequenceReversed,
    TargetPoseReached,
)
from coraplex.view_manager import ViewManager
from semantic_digital_twin.datastructures.definitions import GripperState
from semantic_digital_twin.reasoning.predicates import allclose
from semantic_digital_twin.reasoning.robot_predicates import is_body_gripped
from semantic_digital_twin.robots.robot_part_mixins import HasMobileBase
from semantic_digital_twin.spatial_types.spatial_types import Pose
from semantic_digital_twin.semantic_annotations.mixins import IsGraspable

logger = logging.getLogger(__name__)


@dataclass
class ReachAction(
    ActionDescription,
    GraspParameters,
    TargetPoseReached,
    PoseSequenceReversed,
ReachTuningParameters, HasGraspDetectionThreshold
):
    """
    Let the robot reach a specific pose.
    """

    target_object: Optional[IsGraspable] = field(default=None, kw_only=True)
    """
    The graspable annotation describing the object that should be reached for.
    """

    open_gripper_at_pre_pose: bool = False
    """
    Whether to open the gripper once the pre-pose is reached, used by
    :class:`PickUpAction` to open before its slower final approach.
    """

    @property
    def _action_plan(self) -> PlanNode:

        target_body = (
            self.target_object.root if self.target_object is not None else None
        )
        target_pre_pose, target_pose, _ = self.grasp_description.pose_sequence(
            self.target_pose, target_body, reverse=self.reverse_pose_sequence
        )
        children = [
            MoveToolCenterPointMotion(
                target_pose=target_pre_pose,
                    arm=self.arm,
                    allow_gripper_collision=False,
                max_linear_velocity=self.pre_approach_linear_velocity,
            ),
        ]
        if self.open_gripper_at_pre_pose:
            children.append(
                MoveGripperMotion(motion=GripperState.OPEN, arm=self.arm)
            )
        children.append(
            MoveToolCenterPointMotion(
                target_pose=target_pose,
                arm=self.arm,
                allow_gripper_collision=False,
                movement_type=MovementType.CARTESIAN,
                max_linear_velocity=self.final_approach_linear_velocity,
            )
        )
        return sequential(children=children)

    def execute(self) -> Any:
        self.add_subplan(self.action_plan).perform()

    @staticmethod
    def pre_condition(
        variables: Dict[str, Variable], context: Context, kwargs: Dict[str, Any]
    ) -> ConditionType:
        """
        The sequence in which the robot would reach the target pose needs to be
        achievable.
        """
        return and_(
            IsObjectReachableBy(
                context=Context(
                    robot=context.robot,
                    world=context.world,
                    alternative_motion_mappings=context.alternative_motion_mappings,
                ),
                arm=variables["arm"],
                object_designator=kwargs["target_object"],
                grasp_description=kwargs["grasp_description"],
                target_pose=kwargs["target_pose"],
                reverse=kwargs["reverse_pose_sequence"],
            ),
        )

    @staticmethod
    def post_condition(
        variables: Dict[str, Variable], context: Context, kwargs: Dict[str, Any]
    ) -> ConditionType:
        """
        The end effector needs to be close to the target pose.
        """
        end_effector = ViewManager.get_end_effector_view(kwargs["arm"], context.robot)
        return or_(
            is_body_gripped(
                variable_from(kwargs["target_object"].root), end_effector
            ,
                threshold=kwargs["grasp_detection_threshold"],
            ),
            allclose(
                variable_from(kwargs["target_object"].root).global_pose.to_position(),
                variable_from(end_effector.tool_frame).global_pose.to_position(),
                atol=3e-2,
            ),
        )


@dataclass
class PickUpAction(ActionDescription, GraspParameters, PickUpTuningParameters, HasGraspDetectionThreshold):
    """
    Let the robot pick up an object.
    """

    tolerate_grasp_stall: bool = field(kw_only=True, default=False)

    @property
    def _action_plan(self) -> PlanNode:

        _, _, lift_to_pose = self.grasp_description.grasp_pose_sequence(
            self.target_object.root
        )
        return sequential(
            children=[
                ReachAction(
                    target_pose=self.target_object.root.global_pose,
                    target_object=self.target_object,
                    arm=self.arm,
                    grasp_description=self.grasp_description,
                    pre_approach_linear_velocity=self.pre_approach_linear_velocity,
                    final_approach_linear_velocity=self.final_approach_linear_velocity,
                    open_gripper_at_pre_pose=True,
                ),
                MoveGripperMotion(
                    motion=GripperState.CLOSE,
                    arm=self.arm,
                    finger_velocity=self.grasp_closing_velocity,
                    stall_minimum_time=self.grasp_stall_minimum_time,
                    tolerate_stall=self.tolerate_grasp_stall,
                ),
                AttachNode(
                    body=self.target_object.root,
                    new_parent=ViewManager.get_end_effector_view(
                        self.arm, self.robot
                    ).tool_frame,
                ),
            ],
        )

    @staticmethod
    def pre_condition(
        variables: Dict, context: Context, kwargs: Dict[str, Any]
    ) -> ConditionType:
        """
        The gripper with which to grasp the object needs to be free and the object needs
        to be reachable.
        """
        end_effector = ViewManager.get_end_effector_view(
            variables["arm"], context.robot
        )
        return and_(
            GripperIsFree(end_effector=end_effector),
            IsObjectReachableBy(
                context=Context(
                    robot=context.robot,
                    world=context.world,
                    alternative_motion_mappings=context.alternative_motion_mappings,
                ),
                arm=variables["arm"],
                object_designator=kwargs["target_object"],
                grasp_description=kwargs["grasp_description"],
            ),
        )

    @staticmethod
    def post_condition(
        variables: Dict, context: Context, kwargs: Dict[str, Any]
    ) -> ConditionType:
        """
        The object needs to be in the gripper frame.
        """
        end_effector = ViewManager.get_end_effector_view(
            variables["arm"], context.robot
        )
        return or_(
            not_(GripperIsFree(end_effector)),
            is_body_gripped(
                variable_from(kwargs["target_object"].root), end_effector
            ,
                threshold=kwargs["grasp_detection_threshold"],
            ),
        )


@dataclass
class GraspingAction(ActionDescription, GraspParameters):
    """
    Grasps an object described by the given Object Designator description.
    """

    @property
    def _action_plan(self) -> PlanNode:
        pre_pose, grasp_pose, _ = self.grasp_description.grasp_pose_sequence(
            self.target_object.root
        )

        return sequential(
            [
                MoveToolCenterPointMotion(target_pose=pre_pose, arm=self.arm),
                MoveGripperMotion(motion=GripperState.OPEN, arm=self.arm),
                MoveToolCenterPointMotion(
                    target_pose=grasp_pose,
                    arm=self.arm,
                    allow_gripper_collision=True,
                ),
                MoveGripperMotion(
                    motion=GripperState.CLOSE,
                    arm=self.arm,
                    allow_gripper_collision=True,
                ),
            ]
        )
