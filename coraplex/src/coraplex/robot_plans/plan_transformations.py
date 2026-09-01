from __future__ import annotations

from dataclasses import dataclass

from typing_extensions import List, cast

from coraplex.datastructures.enums import DetectionTechnique
from coraplex.exceptions import PerceptionTargetMissing
from coraplex.plans.plan_node import ActionLike, ActionNode, MotionNode, PlanNode
from coraplex.plans.plan_transformation import (
    ActionTransformation,
    InsertionTransformation,
)
from coraplex.robot_plans.actions.core.misc import DetectAction
from coraplex.robot_plans.actions.core.navigation import LookAtAction
from coraplex.robot_plans.actions.core.pick_up import ReachAction


@dataclass
class DetectBeforeGrasp(InsertionTransformation, ActionTransformation[ReachAction]):
    """
    Looks at the object and detects it before a reach makes its final approach, so that
    the approach acts on a freshly perceived pose instead of the one the world holds.
    """

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
