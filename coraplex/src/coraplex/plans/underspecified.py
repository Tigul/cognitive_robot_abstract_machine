from __future__ import annotations

from dataclasses import dataclass, field

from typing_extensions import Any, Dict, List

from coraplex.datastructures.dataclasses import Context, PlanContextExtension
from coraplex.plans.plan_transformation import PlanRewriting
from cramph.composites import CompositeNodeChoosingItsChild
from cramph.node import StatechartNode
from krrood.adapters.json_serializer import JSONField
from krrood.entity_query_language.query.match import Match
from krrood.patterns.field_metadata import JSONMetadata
from krrood.utils import get_full_class_name


@dataclass(eq=False, repr=False)
class UnderspecifiedNode(CompositeNodeChoosingItsChild):
    """
    Runs an action described by an underspecified `a(...)` / `an(...)` statement.

    The statement is grounded only when this node starts, against the world the nodes
    before it left behind, and the grounded actions are tried until one succeeds. The
    node fails once the statement yields no further action. If you want to limit the
    number of attempts, add a limit clause to the statement.
    """

    statement: Match = field(
        kw_only=True, metadata=JSONMetadata(serialize=False).as_dict()
    )
    """
    The underspecified statement the actions are grounded from.

    Not serialized: a process ticking the statechart elsewhere receives the chosen
    actions rather than grounding them itself.
    """

    @classmethod
    def for_step(cls, step: StatechartNode | Match) -> StatechartNode:
        """
        :param step: A node, or an underspecified statement of an action.
        :return: `step` itself, or the node grounding the statement when it runs.
        """
        if isinstance(step, Match):
            return cls(statement=step)
        return step

    @property
    def chosen_actions(self) -> List[StatechartNode]:
        """
        :return: The grounded action every child chosen so far runs, in the order they
            were chosen, each found below whatever runs it.
        """
        return [
            next(
                node
                for node in [child, *child.descendants]
                if isinstance(node, self.statement._type_)
            )
            for child in self.children
        ]

    @property
    def context(self) -> Context:
        """
        :return: The plan context the statement is grounded in.
        """
        return self.statechart.context.require_extension(PlanContextExtension).context

    def adopt_chosen_child(self, child: StatechartNode) -> None:
        """
        Run `child` next, rewritten by the plan transformations of the statechart, if it
        carries any.
        """
        super().adopt_chosen_child(child)
        rewriting = self.statechart.context.get_extension(PlanRewriting)
        if rewriting is not None:
            rewriting.rewrite(child)

    def to_json(self, **kwargs) -> Dict[str, Any]:
        """
        :return: The JSON representation of the node choosing its child that this is, so
            a process ticking the statechart elsewhere needs nothing of coraplex.
        """
        return {
            **super().to_json(**kwargs),
            JSONField.TYPE: get_full_class_name(CompositeNodeChoosingItsChild),
        }

    def __repr__(self):
        return f"{self.statement._type_.__name__}"
