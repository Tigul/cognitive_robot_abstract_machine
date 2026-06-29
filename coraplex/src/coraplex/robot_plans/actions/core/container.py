from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass

import numpy as np
from typing_extensions import Any, Dict

from krrood.entity_query_language.core.base_expressions import SymbolicExpression
from krrood.entity_query_language.factories import and_, ConditionType
from coraplex.datastructures.dataclasses import Context
from coraplex.datastructures.enums import (
    Arms,
    ApproachDirection,
    VerticalAlignment,
)
from coraplex.datastructures.grasp import GraspDescription
from coraplex.locations.pose_validator import IsReachableBy
from coraplex.plans.factories import sequential
from coraplex.querying.predicates import GripperIsFree
from coraplex.robot_plans.actions.base import ActionDescription
from coraplex.robot_plans.actions.core.pick_up import GraspingAction
from coraplex.robot_plans.motions.container import OpeningMotion, ClosingMotion
from coraplex.robot_plans.motions.gripper import MoveGripperMotion
from coraplex.robot_plans.parameter_mixins import (
    HandleOperatedOn,
    UsedArm,
    UsedGraspingPreposeDistance,
)
from coraplex.view_manager import ViewManager
from semantic_digital_twin.datastructures.definitions import GripperState
from semantic_digital_twin.reasoning.robot_predicates import is_body_in_gripper
from semantic_digital_twin.robots.robot_part_mixins import HasMobileBase
from semantic_digital_twin.robots.robot_parts import AbstractRobot
from semantic_digital_twin.semantic_annotations.semantic_annotations import Handle
from semantic_digital_twin.world_description.connections import ActiveConnection1DOF


@dataclass
class OpenAction(
    HandleOperatedOn, UsedArm, UsedGraspingPreposeDistance, ActionDescription
):
    """
    Opens a container like object
    """

    def execute(self) -> None:
        arm = ViewManager.get_arm_view(self.arm, self.robot)
        end_effector = arm.end_effector

        grasp_description = GraspDescription(
            ApproachDirection.FRONT,
            VerticalAlignment.NoAlignment,
            end_effector,
        )

        self.add_subplan(
            sequential(
                [
                    GraspingAction(
                        object_designator=self.handle.root,
                        arm=self.arm,
                        grasp_description=grasp_description,
                    ),
                    OpeningMotion(handle=self.handle, arm=self.arm),
                    MoveGripperMotion(
                        motion=GripperState.OPEN,
                        arm=self.arm,
                        allow_gripper_collision=True,
                    ),
                ]
            )
        ).perform()

    @staticmethod
    def pre_condition(
        variables, context: Context, kwargs: Dict[str, Any]
    ) -> ConditionType:
        """
        The gripper with which to open the container has to be free and the handle has to be reachable.
        """
        test_world = deepcopy(context.world)
        test_robot: AbstractRobot = test_world.get_semantic_annotation_by_id(
            context.robot.id
        )
        end_effector = ViewManager.get_end_effector_view(variables["arm"], test_robot)

        return and_(
            GripperIsFree(end_effector),
            IsReachableBy(
                world=test_world,
                robot=test_world.get_semantic_annotations_by_type(type(context.robot))[
                    0
                ],
                pose=kwargs["handle"].root.global_pose,
                tip_link=end_effector.tool_frame,
                grasp_description=GraspDescription(
                    ApproachDirection.FRONT,
                    VerticalAlignment.NoAlignment,
                    next(end_effector.evaluate()),
                ),
            ),
        )

    @staticmethod
    def post_condition(
        variables, context: Context, kwargs: Dict[str, Any]
    ) -> SymbolicExpression | bool:
        """
        The handle has to be in the gripper of the robot and the container has to be open.
        """
        end_effector = ViewManager.get_end_effector_view(kwargs["arm"], context.robot)
        handle_body = kwargs["handle"].root
        parent_connection = handle_body.get_first_parent_connection_of_type(
            ActiveConnection1DOF
        )
        return (
            is_body_in_gripper(handle_body, end_effector) > 0.9
            or np.allclose(
                handle_body.global_pose.to_position(),
                ViewManager.get_end_effector_view(
                    kwargs["arm"], context.robot
                ).tool_frame.global_pose.to_position(),
                atol=3e-2,
            )
        ) and bool(parent_connection.position > 0.3)


@dataclass
class CloseAction(
    HandleOperatedOn, UsedArm, UsedGraspingPreposeDistance, ActionDescription
):
    """
    Closes a container like object.
    """

    def execute(self) -> None:
        arm = ViewManager.get_arm_view(self.arm, self.robot)
        end_effector = arm.end_effector

        grasp_description = GraspDescription(
            ApproachDirection.FRONT,
            VerticalAlignment.NoAlignment,
            end_effector,
        )

        self.add_subplan(
            sequential(
                [
                    GraspingAction(
                        object_designator=self.handle.root,
                        arm=self.arm,
                        grasp_description=grasp_description,
                    ),
                    ClosingMotion(handle=self.handle, arm=self.arm),
                    MoveGripperMotion(
                        motion=GripperState.OPEN,
                        arm=self.arm,
                        allow_gripper_collision=True,
                    ),
                ]
            )
        ).perform()

    @staticmethod
    def post_condition(
        variables, context: Context, kwargs: Dict[str, Any]
    ) -> SymbolicExpression | bool:
        """
        The container has to be closed
        """
        close_connection = kwargs["handle"].root.get_first_parent_connection_of_type(
            ActiveConnection1DOF
        )

        return bool(close_connection.position < 0.1)
