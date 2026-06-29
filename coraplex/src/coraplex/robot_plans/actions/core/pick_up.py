from __future__ import annotations

import logging
from copy import deepcopy
from dataclasses import dataclass, field

from typing_extensions import Any, Dict, Optional

from coraplex.locations.pose_validator import AreReachableBy
from krrood.entity_query_language.core.base_expressions import SymbolicExpression
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
from coraplex.plans.factories import sequential, execute_single
from coraplex.querying.predicates import GripperIsFree
from coraplex.robot_plans.actions.base import ActionDescription
from coraplex.robot_plans.motions.gripper import (
    MoveGripperMotion,
    MoveToolCenterPointMotion,
)
from coraplex.robot_plans.parameter_mixins import (
    ObjectActedOn,
    PoseSequenceReversed,
    TargetPoseReached,
    UsedArm,
    UsedGraspDescription,
)
from coraplex.view_manager import ViewManager
from semantic_digital_twin.datastructures.definitions import GripperState
from semantic_digital_twin.reasoning.predicates import allclose
from semantic_digital_twin.reasoning.robot_predicates import is_body_in_gripper
from semantic_digital_twin.robots.robot_part_mixins import HasMobileBase
from semantic_digital_twin.spatial_types.spatial_types import Pose
from semantic_digital_twin.world_description.world_entity import Body

logger = logging.getLogger(__name__)


@dataclass
class ReachAction(
    TargetPoseReached,
    UsedArm,
    UsedGraspDescription,
    ObjectActedOn,
    PoseSequenceReversed,
    ActionDescription,
):
    """
    Let the robot reach a specific pose.
    """

    object_designator: Optional[Body] = field(default=None, kw_only=True)
    """
    Object designator_description describing the object that should be picked up
    """

    def execute(self) -> None:

        target_pre_pose, target_pose, _ = self.grasp_description._pose_sequence(
            self.target_pose, self.object_designator, reverse=self.reverse_pose_sequence
        )
        self.add_subplan(
            sequential(
                children=[
                    MoveToolCenterPointMotion(
                        target_pose=target_pre_pose,
                        arm=self.arm,
                        allow_gripper_collision=False,
                    ),
                    MoveToolCenterPointMotion(
                        target_pose=target_pose,
                        arm=self.arm,
                        allow_gripper_collision=False,
                        movement_type=MovementType.CARTESIAN,
                    ),
                ]
            )
        ).perform()

    @staticmethod
    def pre_condition(
        variables, context: Context, kwargs: Dict[str, Any]
    ) -> ConditionType:
        """
        The sequence in which the robot would reach the target pose needs to be achiveable
        """
        end_effector = ViewManager.get_end_effector_view(
            variables["arm"], context.robot
        )
        test_world = deepcopy(context.world)
        grasp_pose_sequence = kwargs["grasp_description"]._pose_sequence(
            kwargs["target_pose"],
            kwargs["object_designator"],
            reverse=kwargs["reverse_pose_sequence"],
        )
        return and_(
            AreReachableBy(
                world=test_world,
                robot=test_world.get_semantic_annotations_by_type(type(context.robot))[
                    0
                ],
                pose_sequence=grasp_pose_sequence,
                tip_link=end_effector.tool_frame,
            ),
        )

    @staticmethod
    def post_condition(
        variables, context: Context, kwargs: Dict[str, Any]
    ) -> ConditionType | bool:
        """
        The end effector needs to be close to the target pose
        """
        end_effector = ViewManager.get_end_effector_view(kwargs["arm"], context.robot)
        return or_(
            is_body_in_gripper(variable_from(kwargs["object_designator"]), end_effector)
            > 0.9,
            allclose(
                variable_from(kwargs["object_designator"].global_pose.to_position()),
                ViewManager.get_end_effector_view(
                    kwargs["arm"], context.robot
                ).tool_frame.global_pose.to_position(),
                atol=3e-2,
            ),
        )


@dataclass
class PickUpAction(ActionDescription, ObjectActedOn, UsedArm, UsedGraspDescription):
    """
    Let the robot pick up an object.
    """

    def execute(self) -> None:
        self.add_subplan(
            sequential(
                children=[
                    MoveGripperMotion(motion=GripperState.OPEN, arm=self.arm),
                    ReachAction(
                        target_pose=self.object_designator.global_pose,
                        object_designator=self.object_designator,
                        arm=self.arm,
                        grasp_description=self.grasp_description,
                    ),
                    MoveGripperMotion(motion=GripperState.CLOSE, arm=self.arm),
                ]
            )
        ).perform()
        end_effector = ViewManager.get_end_effector_view(self.arm, self.robot)

        # Attach the object to the end effector
        with self.world.modify_world():
            self.world.move_branch_with_fixed_connection(
                self.object_designator, end_effector.tool_frame
            )

        _, _, lift_to_pose = self.grasp_description.grasp_pose_sequence(
            self.object_designator
        )
        self.add_subplan(
            execute_single(
                MoveToolCenterPointMotion(
                    target_pose=lift_to_pose,
                    arm=self.arm,
                    allow_gripper_collision=True,
                    movement_type=MovementType.TRANSLATION,
                )
            )
        ).perform()

    @staticmethod
    def pre_condition(
        variables: Dict, context: Context, kwargs: Dict[str, Any]
    ) -> ConditionType:
        """
        The gripper with which to grasp the object needs to be free and the object needs to be reachable
        """
        end_effector = ViewManager.get_end_effector_view(
            variables["arm"], context.robot
        )
        test_world = deepcopy(context.world)
        grasp_pose_sequence = kwargs["grasp_description"].grasp_pose_sequence(
            kwargs["object_designator"]
        )
        return and_(
            GripperIsFree(end_effector),
            AreReachableBy(
                world=test_world,
                robot=test_world.get_semantic_annotations_by_type(type(context.robot))[
                    0
                ],
                pose_sequence=grasp_pose_sequence,
                tip_link=end_effector.tool_frame,
            ),
        )

    @staticmethod
    def post_condition(
        variables: Dict, context: Context, kwargs: Dict[str, Any]
    ) -> SymbolicExpression:
        """
        The object needs to be in the griper frame
        """
        end_effector = ViewManager.get_end_effector_view(
            variables["arm"], context.robot
        )
        return or_(
            not_(GripperIsFree(end_effector)),
            is_body_in_gripper(kwargs["object_designator"], end_effector) > 0.9,
        )


@dataclass
class GraspingAction(ObjectActedOn, UsedArm, UsedGraspDescription, ActionDescription):
    """
    Grasps an object described by the given Object Designator description
    """

    def execute(self) -> None:
        pre_pose, grasp_pose, _ = self.grasp_description.grasp_pose_sequence(
            self.object_designator
        )

        self.add_subplan(
            sequential(
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
        ).perform()
