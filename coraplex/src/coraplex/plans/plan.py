from __future__ import annotations

from dataclasses import dataclass, field

from typing_extensions import Optional, TYPE_CHECKING

from coraplex.exceptions import ContextIsUnavailable
from coraplex.plans.plan_execution import PlanExecution
from cramph.node import StatechartNode

if TYPE_CHECKING:
    from coraplex.datastructures.dataclasses import Context


@dataclass
class Plan:
    """
    Something a robot is to do, described by the statechart node it runs and the context
    it runs in.

    Performing a plan runs its node as the root of one statechart, see
    :class:`~coraplex.plans.plan_execution.PlanExecution`. A plan built without a
    context only describes a step of another plan, which takes over its root.
    """

    root: StatechartNode
    """
    The node the plan consists of.
    """

    context: Optional[Context] = field(default=None, kw_only=True)
    """
    The context the plan is performed in.
    """

    def perform(self) -> None:
        """
        Run the root until it succeeded.

        :raises ContextIsUnavailable: If the plan has no context to be performed in.
        :raises MotionDidNotFinish: If the root did not succeed.
        """
        if self.context is None:
            raise ContextIsUnavailable(self)
        PlanExecution(root=self.root, context=self.context).run()
