from __future__ import annotations

from dataclasses import dataclass

from typing_extensions import List, cast

from coraplex.datastructures.enums import Arms, DetectionTechnique, InsertionPosition
from coraplex.exceptions import PerceptionTargetMissing
from coraplex.locations.factories import reachability_location
from coraplex.plans.plan_node import ActionLike, ActionNode, MotionNode, PlanNode
from coraplex.plans.plan_transformation import (
    ActionMatch,
    InsertionRewrite,
    PlanMatch,
)
from coraplex.robot_plans.actions.core.container import OpenAction
from coraplex.robot_plans.actions.core.misc import DetectAction
from coraplex.robot_plans.actions.core.navigation import LookAtAction, NavigateAction
from coraplex.robot_plans.actions.core.pick_up import PickUpAction, ReachAction
from coraplex.robot_plans.actions.core.robot_body import ParkArmsAction
from krrood.entity_query_language.factories import a, variable
from semantic_digital_twin.reasoning.predicates import InsideOf
from semantic_digital_twin.semantic_annotations.semantic_annotations import Drawer
from semantic_digital_twin.spatial_types.spatial_types import Pose

# %% perceiving before a grasp


@dataclass
class DetectBeforeGrasp(InsertionRewrite, ActionMatch[ReachAction]):
    """
    Looks at the object and detects it before a reach makes its final approach, so that
    the approach acts on a freshly perceived pose instead of the one the world holds.
    """

    @property
    def position(self) -> InsertionPosition:
        return InsertionPosition.BEFORE

    def final_approach(self, plan_node: ActionNode) -> MotionNode:
        """
        :param plan_node: The node of the reach
        :return: The reach's last motion, which brings the tool center point onto the
            object.
        """
        motions = [
            node for node in plan_node.descendants if isinstance(node, MotionNode)
        ]
        return motions[-1]

    def anchor(self, plan_node: ActionNode) -> PlanNode:
        return self.final_approach(plan_node)

    def nodes_to_insert(self, plan_node: ActionNode) -> List[ActionLike]:
        reach = cast(ReachAction, plan_node.action)
        if reach.object_designator is None:
            raise PerceptionTargetMissing(reach)
        return [
            LookAtAction(self.final_approach(plan_node).motion.target),
            DetectAction(
                DetectionTechnique.TYPES,
                object_sem_annotation=type(reach.object_designator),
                accept_first_if_multiple=True,
            ),
        ]


# %% opening what the object lies in


@dataclass
class OpenDrawerBeforePickUp(InsertionRewrite, ActionMatch[PickUpAction]):
    """
    Opens the drawers an object lies in before the robot picks it up, so that it reaches
    into an open drawer instead of a closed one.
    """

    minimum_containment_ratio: float = 0.9
    """
    How much of the object has to lie within a drawer for it to count as being in it.
    """

    @property
    def position(self) -> InsertionPosition:
        return InsertionPosition.BEFORE

    def containing_drawers(self, pick_up: PickUpAction) -> List[Drawer]:
        """
        :param pick_up: The pick-up whose object to locate
        :return: The drawers the object lies in.
        """
        object_body = pick_up.object_designator.root
        return [
            drawer
            for drawer in pick_up.world.get_semantic_annotations_by_type(Drawer)
            if InsideOf(object_body, drawer.root).compute_containment_ratio()
            > self.minimum_containment_ratio
        ]

    def is_applicable(self, plan_node: PlanNode) -> bool:
        return bool(self.containing_drawers(cast(PickUpAction, plan_node.action)))

    def anchor(self, plan_node: PlanNode) -> PlanNode:
        return plan_node

    def nodes_to_insert(self, plan_node: PlanNode) -> List[ActionLike]:
        pick_up = cast(PickUpAction, plan_node.action)
        nodes = []
        for drawer in self.containing_drawers(pick_up):
            handle = drawer.handle.root
            nodes.extend(
                [
                    a(NavigateAction)(
                        target_location=variable(
                            Pose,
                            domain=reachability_location(
                                handle.global_pose, pick_up.context, pick_up.arm
                            ),
                        ),
                        keep_joint_states=True,
                    ),
                    OpenAction(handle, pick_up.arm),
                ]
            )
        return nodes


# %% parking before anything else


@dataclass
class ParkArmsBeforeFirstAction(InsertionRewrite, PlanMatch[ActionNode]):
    """
    Parks the arms in front of the first action of a plan.

    An action that is grounded against the world, such as a drive to a pose the object
    can be reached from, judges the robot in the configuration it is in. Arms left
    wherever an earlier plan dropped them stand in collision at every candidate pose,
    which rules out the whole location before it is ever checked for reachability.
    """

    arm: Arms = Arms.BOTH
    """
    The arms that are parked.
    """

    @property
    def position(self) -> InsertionPosition:
        return InsertionPosition.BEFORE

    def is_applicable(self, plan_node: PlanNode) -> bool:
        return plan_node.plan.actions[0] is plan_node and not isinstance(
            plan_node.action, ParkArmsAction
        )

    def anchor(self, plan_node: PlanNode) -> PlanNode:
        return plan_node

    def nodes_to_insert(self, plan_node: PlanNode) -> List[ActionLike]:
        return [ParkArmsAction(self.arm)]
