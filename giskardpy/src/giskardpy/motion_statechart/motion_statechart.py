from __future__ import annotations

from dataclasses import dataclass

from typing_extensions import (
    List,
)

from cramph.node import (
    StatechartNode,
)
from giskardpy.motion_statechart.graph_node import (
    DebugExpression,
    EndMotion,
    MotionStatechartNode,
    Task,
)
from giskardpy.qp.constraint_collection import ConstraintCollection
from cramph.statechart import Statechart


@dataclass
class MotionStatechart(Statechart):
    """
    A statechart whose nodes may contribute constraints and debug expressions to motion
    control.
    """

    @staticmethod
    def _create_structure_node_copy(node: StatechartNode) -> StatechartNode:
        match node:
            case Task():
                return Task(name=node.name)
            case EndMotion():
                return EndMotion(name=node.name)
            case MotionStatechartNode():
                return MotionStatechartNode(name=node.name)
            case _:
                return Statechart._create_structure_node_copy(node)

    def collect_debug_expressions(self) -> List[DebugExpression]:
        """
        Gather the debug expressions registered by every node into a single flat list.

        :return: The debug expressions of every node.
        """
        return [
            debug_expression
            for node in self.get_nodes_by_type(MotionStatechartNode)
            for debug_expression in node.debug_expressions
        ]

    def combine_constraint_collections_of_nodes(self) -> ConstraintCollection:
        """
        :return: The constraint collections of all nodes, merged into one, with each node's
            constraints prefixed by its :attr:`~cramph.node.StatechartNode.unique_name`.
        """
        combined_constraint_collection = ConstraintCollection()
        for node in self.get_nodes_by_type(MotionStatechartNode):
            combined_constraint_collection.merge(
                name_prefix=node.unique_name, other=node.constraint_collection
            )
        return combined_constraint_collection
