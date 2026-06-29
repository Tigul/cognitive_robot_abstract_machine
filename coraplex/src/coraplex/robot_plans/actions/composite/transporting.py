from __future__ import annotations

from dataclasses import dataclass, field
from datetime import timedelta
from typing import List

from typing_extensions import Optional, Any

from krrood.entity_query_language.factories import (
    an,
    entity,
    variable,
    underspecified,
)
from coraplex.config.action_conf import ActionConfig
from coraplex.datastructures.enums import Arms, ApproachDirection, VerticalAlignment
from coraplex.datastructures.grasp import GraspDescription
from coraplex.locations.factories import reachability_location
from coraplex.plans.factories import sequential, execute_single
from coraplex.plans.failures import BodyUnfetchable
from coraplex.robot_plans.actions.base import ActionDescription
from coraplex.robot_plans.actions.composite.facing import FaceAtAction
from coraplex.robot_plans.actions.core.container import OpenAction
from coraplex.robot_plans.actions.core.navigation import NavigateAction
from coraplex.robot_plans.actions.core.pick_up import PickUpAction
from coraplex.robot_plans.actions.core.placing import PlaceAction
from coraplex.robot_plans.actions.core.robot_body import ParkArmsAction, MoveTorsoAction
from coraplex.robot_plans.parameter_mixins import (
    ObjectActedOn,
    StandingPositionMovedTo,
    TargetLocationMovedTo,
    JointStatesKept,
    UsedArm,
    UsedGraspDescription,
)
from coraplex.view_manager import ViewManager
from semantic_digital_twin.datastructures.definitions import TorsoState
from semantic_digital_twin.reasoning.predicates import InsideOf
from semantic_digital_twin.semantic_annotations.semantic_annotations import Drawer
from semantic_digital_twin.spatial_types.spatial_types import Pose
from semantic_digital_twin.world_description.world_entity import Body


@dataclass
class TransportAction(
    ObjectActedOn,
    TargetLocationMovedTo,
    UsedArm,
    UsedGraspDescription,
    ActionDescription,
):
    """
    Transports an object to a position using an arm
    """

    object_designator: Body = field(repr=False, kw_only=True)
    """
    Object designator_description describing the object that should be transported.
    """

    grasp_description: Optional[GraspDescription] = field(default=None, kw_only=True)
    """
    Grasp Description that should be used for picking up the object
    """

    def inside_container(self) -> List[Body]:
        bodies = []
        for body in self.world.bodies:
            if body == self.object_designator:
                continue
            if InsideOf(self.object_designator, body).compute_containment_ratio() > 0.9:
                bodies.append(body)
        return bodies

    def open_container(self, container: Body):

        drawer_annotation = an(
            entity(
                drawer := variable(Drawer, domain=self.world.semantic_annotations)
            ).where(drawer.root == container)
        )
        drawer_annotation = list(drawer_annotation.evaluate())
        if len(drawer_annotation) == 0:
            return
        handle = drawer_annotation[0].handle

        self.add_subplan(
            sequential(
                [
                    NavigateAction(
                        target_location=reachability_location(
                            handle.root.global_pose, self.context, self.arm
                        ).ground(),
                        keep_joint_states=True,
                    ),
                    OpenAction(handle=handle, arm=self.arm),
                ]
            )
        ).perform()

    def execute(self) -> None:
        self.grasp_description = self.grasp_description or GraspDescription(
            ApproachDirection.FRONT,
            VerticalAlignment.NoAlignment,
            ViewManager.get_end_effector_view(self.arm, self.robot),
        )

        for container in self.inside_container():
            self.open_container(container)

        self.add_subplan(execute_single(ParkArmsAction(arm=Arms.BOTH))).perform()

        pickup_loc = reachability_location(
            self.object_designator,
            self.context,
            self.arm,
            self.grasp_description,
        )
        # Tries to find a pick-up position for the robot that uses the given arm

        pickup_pose = pickup_loc.ground()

        if not pickup_pose:
            raise BodyUnfetchable(self.object_designator, self.arm)

        self.add_subplan(
            sequential(
                [
                    NavigateAction(target_location=pickup_pose, keep_joint_states=True),
                    PickUpAction(
                        object_designator=self.object_designator,
                        arm=self.arm,
                        grasp_description=self.grasp_description,
                    ),
                    ParkArmsAction(arm=Arms.BOTH),
                    MoveTorsoAction(torso_state=TorsoState.HIGH),
                ]
            )
        ).perform()

        self.add_subplan(self._make_place_plan()).perform()

    def _make_place_plan(self):

        return sequential(
            children=[
                self._make_navigate_action_for_placing(self.grasp_description),
                PlaceAction(
                    object_designator=self.object_designator,
                    target_location=self.target_location,
                    arm=self.arm,
                ),
                ParkArmsAction(arm=Arms.BOTH),
            ]
        )

    def _make_navigate_action_for_placing(self, grasp_description: GraspDescription):
        """
        :param grasp_description: The grasp description that should be used for placing the object.
        :return: The navigate action that will be used to place the object.
        """
        return underspecified(NavigateAction)(
            target_location=variable(
                Pose,
                domain=reachability_location(
                    self.target_location, self.context, self.arm, self.grasp_description
                ),
            ),
            keep_joint_states=True,
        )


@dataclass
class PickAndPlaceAction(
    ObjectActedOn,
    TargetLocationMovedTo,
    UsedArm,
    UsedGraspDescription,
    ActionDescription,
):
    """
    Transports an object to a position using an arm without moving the base of the robot
    """

    def execute(self) -> None:
        self.add_subplan(
            sequential(
                [
                    ParkArmsAction(arm=Arms.BOTH),
                    PickUpAction(
                        object_designator=self.object_designator,
                        arm=self.arm,
                        grasp_description=self.grasp_description,
                    ),
                    ParkArmsAction(arm=Arms.BOTH),
                    PlaceAction(
                        object_designator=self.object_designator,
                        target_location=self.target_location,
                        arm=self.arm,
                    ),
                    ParkArmsAction(arm=Arms.BOTH),
                ]
            )
        ).perform()


@dataclass
class MoveAndPlaceAction(
    StandingPositionMovedTo,
    ObjectActedOn,
    TargetLocationMovedTo,
    UsedArm,
    JointStatesKept,
    ActionDescription,
):
    """
    Navigate to `standing_position`, then turn towards the object and pick it up.
    """

    keep_joint_states: bool = field(
        default=ActionConfig.navigate_keep_joint_states, kw_only=True
    )
    """
    Keep the joint states of the robot the same during the navigation.
    """

    def execute(self):
        self.add_subplan(
            sequential(
                [
                    NavigateAction(
                        target_location=self.standing_position,
                        keep_joint_states=self.keep_joint_states,
                    ),
                    FaceAtAction(
                        look_at_target=self.target_location,
                        keep_joint_states=self.keep_joint_states,
                    ),
                    PlaceAction(
                        object_designator=self.object_designator,
                        target_location=self.target_location,
                        arm=self.arm,
                    ),
                ]
            )
        ).perform()


@dataclass
class MoveAndPickUpAction(
    StandingPositionMovedTo,
    ObjectActedOn,
    UsedArm,
    UsedGraspDescription,
    JointStatesKept,
    ActionDescription,
):
    """
    Navigate to `standing_position`, then turn towards the object and pick it up.
    """

    keep_joint_states: bool = field(
        default=ActionConfig.navigate_keep_joint_states, kw_only=True
    )
    """
    Keep the joint states of the robot the same during the navigation.
    """

    def execute(self):
        self.add_subplan(
            sequential(
                [
                    NavigateAction(
                        target_location=self.standing_position,
                        keep_joint_states=self.keep_joint_states,
                    ),
                    FaceAtAction(
                        look_at_target=self.object_designator.global_pose,
                        keep_joint_states=self.keep_joint_states,
                    ),
                    PickUpAction(
                        object_designator=self.object_designator,
                        arm=self.arm,
                        grasp_description=self.grasp_description,
                    ),
                ]
            )
        ).perform()
