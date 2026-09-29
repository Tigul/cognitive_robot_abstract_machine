from __future__ import annotations

from dataclasses import dataclass, field

import krrood.symbolic_math.symbolic_math as sm
from cramph.context import StatechartContext
from cramph.data_types import SuccessDecider
from cramph.node import NodeArtifacts, StatechartNode
from semantic_digital_twin.world_description.world_entity import Body

# %% changing the kinematic structure


@dataclass(eq=False, repr=False)
class MoveBranch(StatechartNode):
    """
    Moves a body, with everything below it, under a new parent when it starts, see
    :meth:`~semantic_digital_twin.world.World.move_branch`, and succeeds once it did.

    The statechart then builds every node again, so that nodes reading the moved branch
    follow its new parent.
    """

    success_decided_by = SuccessDecider.ITSELF

    body: Body = field(kw_only=True)
    """
    The root of the branch that is moved.
    """

    new_parent: Body = field(kw_only=True)
    """
    The body the branch is moved under.
    """

    def build_artifacts(self, context: StatechartContext) -> NodeArtifacts:
        return NodeArtifacts(observation=sm.Scalar.const_true())

    def on_start(self, context: StatechartContext):
        context.world.move_branch(self.body, self.new_parent)
