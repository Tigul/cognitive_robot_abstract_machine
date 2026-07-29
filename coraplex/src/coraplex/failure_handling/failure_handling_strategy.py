from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass

from typing_extensions import ClassVar, Optional, TYPE_CHECKING, Type

from coraplex.datastructures.enums import TaskStatus
from coraplex.plans.factories import execute_single
from coraplex.plans.failures import PlanFailure

if TYPE_CHECKING:
    from coraplex.plans.plan_node import ActionLike, PlanNode, UnderspecifiedNode

# %% resolutions


@dataclass
class FailureResolution(ABC):
    """
    The decision a :class:`FailureHandlingStrategy` made about how plan execution
    continues after a failure.

    A resolution interprets itself inside the except block of
    :meth:`~coraplex.plans.plan_node.PlanNode.perform`: :meth:`apply` *returning* means
    "re-run this frame", :meth:`apply` *raising* means "propagate to the parent frame",
    which applies the resolution again. The performing frame never branches on
    resolution or node types.
    """

    failure: PlanFailure
    """
    The refined failure this resolution resolves.
    """

    @abstractmethod
    def apply(self, node: PlanNode) -> None:
        """
        Interpret this resolution at the given perform frame.

        Returning re-runs the frame; raising the carried failure hands it to the parent
        frame, which applies this resolution again.

        :param node: The node whose perform frame is applying this resolution.
        """

    def propagate(self, node: PlanNode) -> None:
        """
        Record the carried failure on the frame and raise it to the parent frame.

        The carried failure keeps a reference to this resolution, so the ancestor frame
        that catches it applies the already decided resolution instead of consulting the
        handler again.

        :param node: The node whose perform frame the failure passes through.
        :raises PlanFailure: Always, with the carried failure.
        """
        node.status = TaskStatus.FAILED
        node.reason = self.failure
        self.failure.resolution = self
        raise self.failure


@dataclass
class Propagate(FailureResolution):
    """
    Give up on handling: the carried failure propagates through every enclosing frame
    and finally out of the plan.
    """

    def apply(self, node: PlanNode) -> None:
        self.propagate(node)


@dataclass
class TargetedResolution(FailureResolution, ABC):
    """
    A resolution that re-runs one specific ancestor frame: it propagates through every
    frame below the target and returns once the target frame applies it.
    """

    target_node: PlanNode
    """
    The node whose perform frame is re-run.
    """

    def apply(self, node: PlanNode) -> None:
        if node is self.target_node:
            self.failure.resolution = None
            return
        self.propagate(node)


@dataclass
class RetryNode(TargetedResolution):
    """
    Re-run the target frame as it is, typically after a recovery sub-plan repaired the
    situation the failure described.
    """


@dataclass
class Reparameterize(TargetedResolution):
    """
    Re-run the frame of an enclosing underspecified node, which advances it to its next
    action candidate.
    """

    target_node: UnderspecifiedNode
    """
    The underspecified node that generates a fresh action candidate when its frame is
    re-run.
    """


# %% strategies


@dataclass
class FailureHandlingStrategy(ABC):
    """
    Decides how plan execution continues after a refined failure.

    A strategy declares the failure type it handles; the
    :class:`~coraplex.failure_handling.failure_handler.FailureHandler` selects the most
    specific applicable strategy. Attempt bookkeeping (for example maximum retries)
    lives in strategy instances.
    """

    handled_failure_type: ClassVar[Type[PlanFailure]] = PlanFailure
    """
    The failure type this strategy resolves.
    """

    def applies(self, failure: PlanFailure) -> bool:
        """
        :param failure: The refined failure to check.
        :return: Whether this strategy can resolve the failure.
        """
        return isinstance(failure, self.handled_failure_type)

    @abstractmethod
    def resolve(self, failure: PlanFailure) -> FailureResolution:
        """
        Decide how execution continues after the failure.

        :param failure: The refined failure to resolve.
        :return: The resolution the performing frames apply.
        """


# %% recovery-plan strategies


@dataclass
class RecoveryPlanStrategy(FailureHandlingStrategy, ABC):
    """
    A strategy that recovers by performing real robot actions before execution
    continues.

    The recovery sub-plan runs as a separate plan sharing the failing plan's
    :class:`~coraplex.datastructures.dataclasses.Context` (same world and robot).

    ..note:: Recording the recovery sub-plan inside the failing plan's tree (via
        :meth:`~coraplex.plans.plan_node.PlanNode.mount_subplan`) is follow-up work.
    """

    @abstractmethod
    def recovery_plan(self, failure: PlanFailure) -> Optional[ActionLike]:
        """
        Build the recovery sub-plan for the failure.

        :param failure: The refined failure to recover from.
        :return: The plan to perform before execution continues, or None if no recovery
            is possible.
        """

    @abstractmethod
    def resolution_after_recovery(self, failure: PlanFailure) -> FailureResolution:
        """
        :param failure: The refined failure the recovery plan just repaired.
        :return: The resolution applied after the recovery plan succeeded, typically a
            :class:`RetryNode` targeting the failing action.
        """

    def resolve(self, failure: PlanFailure) -> FailureResolution:
        recovery_plan = self.recovery_plan(failure)
        if recovery_plan is None:
            return Propagate(failure=failure)

        recovery_root = execute_single(recovery_plan, context=failure.context)
        try:
            recovery_root.perform()
        except PlanFailure:
            return Propagate(failure=failure)
        return self.resolution_after_recovery(failure)
