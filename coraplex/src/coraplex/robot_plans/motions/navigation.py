from dataclasses import dataclass

from giskardpy.motion_statechart.monitors.overwrite_state_monitors import SetOdometry
from giskardpy.motion_statechart.tasks.cartesian_tasks import CartesianPose
from coraplex.robot_plans.motions.base import BaseMotion
from coraplex.robot_plans.parameter_mixins import JointStatesKept, TargetLocationMovedTo
from semantic_digital_twin.spatial_types.spatial_types import Pose


@dataclass
class MoveMotion(BaseMotion, TargetLocationMovedTo, JointStatesKept):
    """
    Moves the robot to a designated location
    """

    def perform(self):
        return

    @property
    def _motion_chart(self):
        return CartesianPose(
            root_link=self.world.root,
            tip_link=self.robot.root,
            goal_pose=self.target_location,
        )
