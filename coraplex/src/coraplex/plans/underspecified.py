from __future__ import annotations

from dataclasses import dataclass, field

from cramph.composites import CompositeNodeChoosingItsChild
from krrood.entity_query_language.query.match import Match
from krrood.patterns.field_metadata import JSONMetadata


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

    def __repr__(self):
        return f"{self.statement.type.__name__}"
