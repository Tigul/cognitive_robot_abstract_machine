from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from typing_extensions import List

from coraplex.plans.factories import ActionLike
from coraplex.robot_plans.actions.base import Action
from coraplex.robot_plans.actions.core.navigation import NavigateAction, LookAtAction
from semantic_digital_twin.spatial_types import (
    Quaternion,
)
from semantic_digital_twin.spatial_types.spatial_types import Pose


@dataclass(eq=False, repr=False)
class FaceAtAction(Action):
    """
    Turn the robot chassis such that is faces the ``pose`` and after that perform a look
    at action.
    """

    pose: Pose
    """
    The pose to face.
    """

    @property
    def _sub_nodes(self) -> List[ActionLike]:
        # get the robot position
        robot_position = self.robot.root.global_transform

        # calculate orientation for robot to face the object
        angle = (
            np.arctan2(
                robot_position.y - self.pose.y,
                robot_position.x - self.pose.x,
            )
            + np.pi
        )

        # create new robot pose
        new_robot_pose = Pose(
            robot_position.to_position(),
            Quaternion.from_rpy(0, 0, angle),
            reference_frame=self.world.root,
        )

        return [
            NavigateAction(new_robot_pose),  # turn robot
            LookAtAction(self.pose),  # look at the target
        ]
