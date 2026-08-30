from __future__ import annotations

from abc import abstractmethod
from dataclasses import dataclass, field

from typing_extensions import Callable, Optional, ClassVar

from coraplex.plans.plan_node import PlanNode, ActionNode
from coraplex.robot_plans.actions.base import ActionDescription
from coraplex.robot_plans.actions.core.pick_up import ReachAction


@dataclass
class TransformationRule:
    node_type: ClassVar[type[PlanNode]]
    """
    Type of the node to which this rule applies.
    """

    condition: Optional[Callable[[PlanNode], bool]] = field(default=lambda plan_node: True)
    """
    Condition under which this rule 
    """

    @abstractmethod
    def apply(self, plan_node: PlanNode) -> None:
        pass

@dataclass
class ActionTransformationRule(TransformationRule):
    node_type: ClassVar[type[ActionNode]] = field(init=False, default=ActionNode)

    action_type: ClassVar[type[ActionNode]]

    @abstractmethod
    def apply(self, plan_node: PlanNode) -> None:
        pass


@dataclass
class DetectBeforeGrasp(ActionTransformationRule):
    action_type: ClassVar[type[ActionDescription]] = ReachAction

    def apply(self, plan_node: PlanNode) -> None:
        move_pre_pose = plan_node.children[0]
        plan_node.plan.insert_node_after(move_pre_pose, )