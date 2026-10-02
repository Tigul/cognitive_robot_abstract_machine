from __future__ import annotations

from dataclasses import dataclass, field

from typing_extensions import Any, Dict

from cramph.composites import CompositeNodeChoosingItsChild
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
        return f"{self.statement.type.__name__}"
