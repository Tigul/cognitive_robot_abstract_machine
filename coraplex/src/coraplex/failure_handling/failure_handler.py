from dataclasses import dataclass

from coraplex.failure_handling.failure_refiner import FailureRefiner
from coraplex.plans.failures import PlanFailure


@dataclass
class FailureHandler:
    refiner: FailureRefiner

    def handle(self, failure: PlanFailure):
        pass
