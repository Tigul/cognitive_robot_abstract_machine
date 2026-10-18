from __future__ import annotations

from dataclasses import dataclass

from semantic_digital_twin.exceptions import UsageError

# %% ray tracing


@dataclass
class RayEndpointCountMismatchError(UsageError):
    """
    Raised when a ray test is given a different number of origins than targets.
    """

    origin_count: int
    """
    How many ray origins were given.
    """

    target_count: int
    """
    How many ray targets were given.
    """

    def error_message(self) -> str:
        return (
            f"A ray test got {self.origin_count} origins but {self.target_count} "
            f"targets."
        )

    def suggest_correction(self) -> str:
        return "Give exactly one target per origin."
