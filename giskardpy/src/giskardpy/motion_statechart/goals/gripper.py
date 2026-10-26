from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field

from cramph.composites import Parallel
from cramph.node import StatechartNode
from giskardpy.motion_statechart.graph_node import MotionStatechartNode
from giskardpy.motion_statechart.mixins import (
    HasFingerStallToleranceParameters,
    HasGoalParameter,
    HasJointVelocityLimitParameter,
    HasThresholdParameter,
    HasTwoFingerGripperParameters,
)
from giskardpy.motion_statechart.monitors.monitors import LocalMinimumReached
from giskardpy.motion_statechart.tasks.cartesian_tasks import DistanceAlongAxis
from giskardpy.motion_statechart.tasks.joint_tasks import (
    JointPositionList,
    JointVelocityLimit,
)
from semantic_digital_twin.datastructures.joint_state import JointState
from semantic_digital_twin.world_description.connections import ActiveConnection1DOF

# %% finger goals


@dataclass(eq=False, repr=False, kw_only=True)
class MoveFingers(
    Parallel,
    HasJointVelocityLimitParameter,
    HasFingerStallToleranceParameters,
    ABC,
):
    """
    Drive the fingers of an end effector to a goal.

    Fingers that close on an object stop short of their goal, so the goal can be told to
    settle for fingers that have stopped moving instead.
    """

    nodes: list[StatechartNode] = field(default_factory=list, init=False)
    """
    The finger goal, together with the speed cap if one was asked for.
    """

    @property
    @abstractmethod
    def finger_connections(self) -> list[ActiveConnection1DOF]:
        """
        :return: The connections that move the fingers.
        """

    @abstractmethod
    def create_finger_task(self) -> MotionStatechartNode:
        """
        :return: A new task driving the fingers to the goal.
        """

    def expand(self) -> None:
        """
        Create the finger goal and the speed cap, after the nodes this goal was handed.
        """
        self.add_node(self._finger_goal())
        if self.max_joint_velocity is not None:
            self.add_node(
                JointVelocityLimit(
                    connections=self.finger_connections,
                    max_joint_velocity=self.max_joint_velocity,
                )
            )

    def _finger_goal(self) -> StatechartNode:
        """
        :return: The node that is done once the fingers arrived, which with
            :attr:`tolerate_finger_stall` is also done once they stopped moving short of it.

        The stall monitor sits beside the finger task rather than replacing its
        observation, because fingers standing still does not mean the goal was reached.
        """
        finger_task = self.create_finger_task()
        if not self.tolerate_finger_stall:
            return finger_task
        return Parallel(nodes=[finger_task, self._stall_monitor()], minimum_success=1)

    def _stall_monitor(self) -> LocalMinimumReached:
        """
        :return: A monitor that observes the fingers, and only those, standing still for
            long enough.
        """
        return LocalMinimumReached(
            degrees_of_freedom=[
                connection.raw_dof for connection in self.finger_connections
            ],
            minimum_time=(
                LocalMinimumReached.minimum_time
                if self.finger_stall_minimum_time is None
                else self.finger_stall_minimum_time
            ),
            measure_from_own_start=True,
        )


@dataclass(eq=False, repr=False, kw_only=True)
class MoveGripper(MoveFingers, HasTwoFingerGripperParameters, HasThresholdParameter):
    """
    Open or close a two-finger gripper until its fingertips' facing surfaces stand
    :attr:`width` apart, with an error in metres.
    """

    threshold: float = 0.001
    """
    A millimetre by default.
    """

    @property
    def finger_connections(self) -> list[ActiveConnection1DOF]:
        return self.gripper.finger_connections

    def create_finger_task(self) -> DistanceAlongAxis:
        """
        :return: A new task bringing the facing points of the fingertips :attr:`width`
            apart along the axis the fingers close along.
        """
        fingertips = self.gripper.fingertips
        return DistanceAlongAxis(
            name=f"{self.name}/width",
            root_link=fingertips.tool_frame,
            reference_feature=fingertips.finger_P_facing_point,
            controlled_feature=fingertips.thumb_P_facing_point,
            axis=fingertips.tool_V_finger_to_thumb,
            distance=self.width,
            threshold=self.threshold,
        )


@dataclass(eq=False, repr=False, kw_only=True)
class MoveFingersToJointState(MoveFingers, HasGoalParameter[JointState]):
    """
    Drive the fingers of a hand to the joint state :attr:`goal`, for a hand whose
    fingers do not close to a single width.
    """

    @property
    def finger_connections(self) -> list[ActiveConnection1DOF]:
        return list(self.goal.connections)

    def create_finger_task(self) -> JointPositionList:
        """
        :return: A new task driving the fingers to :attr:`goal`.
        """
        return JointPositionList(goal=self.goal)
