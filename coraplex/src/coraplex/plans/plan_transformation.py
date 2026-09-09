from __future__ import annotations

from abc import abstractmethod, ABC
from dataclasses import dataclass

from typing_extensions import (
    Callable,
    ClassVar,
    Dict,
    Generic,
    List,
    Protocol,
    Type,
    TypeVar,
    runtime_checkable,
)

from coraplex.datastructures.enums import InsertionPosition
from coraplex.plans.factories import make_node
from coraplex.plans.plan import Plan
from coraplex.plans.plan_node import ActionLike, ActionNode, PlanNode
from coraplex.robot_plans.actions.base import ActionDescription
from krrood.patterns.subclass_safe_generic import SubClassSafeGeneric
from krrood.utils import get_generic_type_parameters

NodeType = TypeVar("NodeType", bound=PlanNode)
ActionType = TypeVar("ActionType", bound=ActionDescription)


# %% transformations


@runtime_checkable
class PlanTransformation(Protocol):
    """
    Rewrites the part of a plan that a node expanded into.

    A transformation is one matching part, which selects the nodes it applies to, and
    one rewriting part, which changes the plan around them. It is applied to every node
    it applies to, right after that node has been expanded and before the nodes below it
    are expanded in turn.
    """

    def applies_to(self, plan_node: PlanNode) -> bool:
        """
        :param plan_node: The node that was just expanded
        :return: Whether this transformation rewrites the given node.
        """

    def apply(self, plan_node: PlanNode) -> None:
        """
        Rewrites the plan around the given node.

        :param plan_node: The node this transformation applies to
        """


# %% matching


@dataclass
class PlanMatch(Generic[NodeType], SubClassSafeGeneric, ABC):
    """
    Selects the nodes of the bound type.
    """

    @property
    def node_type(self) -> Type[PlanNode]:
        """
        :return: The type of node this selects.
        """
        return get_generic_type_parameters(
            type(self), PlanMatch, include_root_generic_base=False
        )[0]

    def applies_to(self, plan_node: PlanNode) -> bool:
        """
        :param plan_node: The node that was just expanded
        :return: Whether this selects the given node.
        """
        return isinstance(plan_node, self.node_type)


@dataclass
class ActionMatch(PlanMatch[ActionNode], Generic[ActionType], SubClassSafeGeneric, ABC):
    """
    Selects the nodes of actions of the bound action type.
    """

    @property
    def node_type(self) -> Type[PlanNode]:
        # Concrete matches of this family bind the action type, so the node type is read
        # from what the family itself binds.
        return get_generic_type_parameters(
            ActionMatch,
            PlanMatch,
            include_root_generic_base=False,
        )[0]

    @property
    def action_type(self) -> Type[ActionDescription]:
        """
        :return: The type of action this selects the nodes of.
        """
        return get_generic_type_parameters(
            type(self), ActionMatch, include_root_generic_base=False
        )[0]

    def applies_to(self, plan_node: PlanNode) -> bool:
        return super().applies_to(plan_node) and isinstance(
            plan_node.designator, self.action_type
        )


# %% rewriting


@dataclass
class PlanRewrite(ABC):
    """
    Changes the plan around a node.
    """

    @abstractmethod
    def apply(self, plan_node: PlanNode) -> None:
        """
        Rewrites the plan around the given node.

        :param plan_node: The node this rewrite is applied to
        """


@dataclass
class InsertionRewrite(PlanRewrite):
    """
    Rewrites a plan by inserting freshly built nodes next to an anchor node.

    The nodes are built anew on every application, since a node belongs to the one plan
    it was inserted into.
    """

    position: InsertionPosition = InsertionPosition.BEFORE
    """
    Where the inserted nodes are placed relative to the anchor node.
    """

    _insertion_methods: ClassVar[Dict[InsertionPosition, Callable[..., None]]] = {
        InsertionPosition.BEFORE: Plan.insert_before,
        InsertionPosition.AFTER: Plan.insert_after,
        InsertionPosition.BELOW: Plan.insert_below,
    }
    """
    The way each position inserts a node into the plan.
    """

    @abstractmethod
    def anchor(self, plan_node: PlanNode) -> PlanNode:
        """
        :param plan_node: The node this rewrite is applied to
        :return: The node the new nodes are inserted next to.
        """

    @abstractmethod
    def nodes_to_insert(self, plan_node: PlanNode) -> List[ActionLike]:
        """
        :param plan_node: The node this rewrite is applied to
        :return: The actions, motions or nodes to insert, in the order they take.
        """

    def apply(self, plan_node: PlanNode) -> None:
        anchor = self.anchor(plan_node)
        insert = self._insertion_methods[self.position]
        for action_like in self.nodes_to_insert(plan_node):
            node = make_node(action_like)
            insert(plan_node.plan, anchor, node)
            if self.position is InsertionPosition.AFTER:
                # each further node goes behind the one before it, keeping their order
                anchor = node
