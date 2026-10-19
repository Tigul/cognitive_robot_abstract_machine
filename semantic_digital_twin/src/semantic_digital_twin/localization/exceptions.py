from __future__ import annotations

from dataclasses import dataclass

from krrood.exceptions import DataclassException
from semantic_digital_twin.world_description.connections import WheeledDrive

# %% motion models


@dataclass
class NoMotionModelForDrive(DataclassException):
    """
    Raised when no motion model describes how a kind of drive moves.
    """

    drive_type: type[WheeledDrive]
    """
    The kind of drive no motion model was found for.
    """

    def error_message(self) -> str:
        return f"No motion model describes a drive of type {self.drive_type.__name__}."

    def suggest_correction(self) -> str:
        return (
            f"Add a subclass of MotionModel bound to {self.drive_type.__name__}, as in "
            f"MotionModel[{self.drive_type.__name__}]."
        )
