from __future__ import annotations
from dataclasses import dataclass

from typing_extensions import List, TypeVar

from coraplex.plans.failures import PlanFailure
from coraplex.plans.plan_node import PlanNode

T = TypeVar("T")


@dataclass
class FailureRefiner:
    """
    Refines a failure that happens during plan execution by running a list of failure detectors which all detect a
    specific kind of failure.
    """

    failure_detectors: List[FailureDetector]
    """
    The failure detectors that narrow down the failure that happened
    """

    def refine(self, exception: PlanFailure) -> Exception:
        pass


@dataclass
class FailureDetector[T]:
    """
    A detector that detects a specific kind of failure given the thrown exception and the node that caused it.
    """

    def detect(self, exception: PlanFailure) -> Exception:
        pass
