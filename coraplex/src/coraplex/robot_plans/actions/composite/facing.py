from __future__ import annotations

from dataclasses import dataclass

from typing_extensions import List

from cramph.node import StatechartNode
from coraplex.robot_plans.actions.base import Action
from coraplex.robot_plans.actions.core.navigation import FaceAtAction, LookAtAction


@dataclass(eq=False, repr=False)
class FaceAndLookAtAction(Action):
    """
    Turns the robot's base towards a target, then looks at it.
    """

    face_at: FaceAtAction
    """
    The turn of the base towards the target.
    """

    look_at: LookAtAction
    """
    The look at the target once the base faces it.
    """

    @property
    def _sub_nodes(self) -> List[StatechartNode]:
        return [self.face_at, self.look_at]
