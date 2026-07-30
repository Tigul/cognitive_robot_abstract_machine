from __future__ import annotations

from coraplex.failure_handling.failure_handler import FailureHandler
from coraplex.failure_handling.failure_refiner import FailureRefiner
from coraplex.failure_handling.strategies.underspecified_reparameterization_strategy import (
    UnderspecifiedReparameterizationStrategy,
)

# %% baseline handler


def baseline_failure_handler() -> FailureHandler:
    """
    :return: The handler every plan context starts with: no detectors and only the
        baseline re-parameterization strategy, which reproduces the pre-failure-handling
        execution semantics.
    """
    return FailureHandler(
        refiner=FailureRefiner(),
        strategies=[UnderspecifiedReparameterizationStrategy()],
    )
