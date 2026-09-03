from __future__ import annotations

from dataclasses import dataclass
from datetime import timedelta

from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    SemanticEnvironmentAnnotation,
)
from semantic_digital_twin.world_description.graph_of_convex_sets.boxes import (
    PlanarGraphOfBoundingBoxes,
)
from semantic_digital_twin.world_description.geometry import VolumetricBoundingBox
from semantic_digital_twin.world_description.shape_collection import (
    BoundingBoxCollection,
)
from typing_extensions import Optional, Any, Dict, List

from krrood.entity_query_language.core.variable import Variable
from krrood.entity_query_language.factories import variable_from, and_, ConditionType
from coraplex.config.action_conf import ActionConfig
from coraplex.datastructures.dataclasses import Context
from coraplex.plans.factories import execute_single, sequential
from coraplex.plans.plan_node import PlanNode
from coraplex.robot_plans.actions.base import ActionDescription
from coraplex.robot_plans.motions.navigation import MoveMotion
from coraplex.robot_plans.motions.robot_body import LookingMotion
from semantic_digital_twin.reasoning.predicates import allclose
from semantic_digital_twin.reasoning.robot_predicates import is_pose_free_for_robot
from semantic_digital_twin.robots.robot_parts import Camera
from semantic_digital_twin.spatial_types.spatial_types import (
    Pose,
    HomogeneousTransformationMatrix,
    Point2,
    Point3,
    RotationMatrix,
    Vector3,
)


@dataclass
class NavigateAction(ActionDescription):
    """
    Navigates the Robot to a position.
    """

    target_location: Pose
    """
    Where the robot should stand, and which way it should face given as the pose's
    x-axis.
    """

    keep_joint_states: bool = ActionConfig.navigate_keep_joint_states
    """
    Keep the joint states of the robot the same during the navigation.
    """

    @property
    def _action_plan(self) -> PlanNode:
        return execute_single(
            MoveMotion(
                self.robot.mobile_base.pose_facing(self.target_location),
                self.keep_joint_states,
            )
        )

    @staticmethod
    def pre_condition(
        variables: Dict[str, Variable], context: Context, kwargs: Dict[str, Any]
    ) -> ConditionType:
        """
        The robot needs to have a drive and the target location needs to be free from
        obstacles.
        """
        drive_variable = variable_from(context.robot.drive is not None)
        return and_(
            is_pose_free_for_robot(context.robot, variables["target_location"]),
            drive_variable,
        )

    @staticmethod
    def post_condition(
        variables: Dict[str, Variable], context: Context, kwargs: Dict[str, Any]
    ) -> ConditionType:
        """
        The robot needs to be within 3 cm of where the heading puts its base.
        """
        return allclose(
            variable_from(context.robot.root).global_pose,
            context.robot.mobile_base.pose_facing(kwargs["target_location"]),
            atol=0.03,
        )


@dataclass
class LookAtAction(ActionDescription):
    """
    Lets the robot look at a position.
    """

    target: Pose
    """
    Position at which the robot should look, given as 6D pose.
    """

    camera: Optional[Camera] = None
    """
    Camera that should be looking at the target.
    """

    @property
    def _action_plan(self) -> PlanNode:
        camera = self.camera or self.robot.get_default_camera()
        return execute_single(LookingMotion(target=self.target, camera=camera))


@dataclass
class GCSNavigateAction(ActionDescription):
    """
    Navigates the robot to a pose along a path through the environment's free space.

    The free space is decomposed into a graph of convex sets, so the robot drives around
    the furniture and walls between it and the target instead of straight at them.
    """

    target: Pose
    """
    Where the robot should stand at the end of the path, with its base.
    """

    @property
    def _action_plan(self) -> PlanNode:
        return sequential([MoveMotion(waypoint) for waypoint in self._path()])

    def _navigation_map(self, floor_level: float) -> PlanarGraphOfBoundingBoxes:
        """
        The floor plan of everything the robot can drive on.

        :param floor_level: The height the robot's base stands at.
        :return: The navigation map covering the whole environment.
        """
        origin = HomogeneousTransformationMatrix(reference_frame=self.world.root)
        environment = SemanticEnvironmentAnnotation(
            root=self.world.root, _world=self.world
        )
        extent = environment.as_bounding_box_collection_at_origin(origin).bounding_box()
        search_space = BoundingBoxCollection(
            [
                VolumetricBoundingBox(
                    min_x=extent.min_x,
                    min_y=extent.min_y,
                    min_z=floor_level,
                    max_x=extent.max_x,
                    max_y=extent.max_y,
                    max_z=floor_level + ActionConfig.navigation_map_height,
                    origin=origin,
                )
            ],
            self.world.root,
        )
        return PlanarGraphOfBoundingBoxes.navigation_map_from_world(
            self.world,
            search_space=search_space,
            bloat_obstacles=ActionConfig.navigation_map_clearance,
        )

    def _path(self) -> list[Pose]:
        """
        The poses the robot drives to, one per leg of the path.

        Each pose faces the waypoint after it, so the leg leaving a waypoint no longer
        has to begin by turning. The waypoint the robot already stands on is left out,
        and the last pose is the requested target.

        .. note::
            The orientation aims the base's x-axis, which is the axis a drive travels
            along, rather than the base's
            :attr:`~semantic_digital_twin.robots.robot_parts.MobileBase.forward_axis`.
            The two differ on a base whose front is not its direction of travel, and it
            is travel that these orientations exist to line up.

        :return: The poses to drive to, in order.
        """
        waypoints = self._waypoints()
        poses = [
            HomogeneousTransformationMatrix.from_point_rotation_matrix(
                Point3(waypoint.x, waypoint.y, self.robot.root.global_pose.z, waypoint.reference_frame),
                RotationMatrix.from_vectors(
                    x=Vector3(
                        next_waypoint.x - waypoint.x,
                        next_waypoint.y - waypoint.y,
                        0,
                        reference_frame=waypoint.reference_frame,
                    ),
                    z=Vector3.Z(),
                    reference_frame=waypoint.reference_frame,
                ),
                reference_frame=waypoint.reference_frame,
            ).to_pose()
            for waypoint, next_waypoint in zip(waypoints[1:], waypoints[2:])
        ]
        return poses + [self.target]

    def _waypoints(self) -> list[Point2]:
        """
        The points the robot travels through to get from where it stands to the target.

        :return: The path, beginning at the robot's own position and ending at the
            target's.
        """
        base_pose = self.robot.root.global_pose
        return self._navigation_map(float(base_pose.z)).path_from_to(
            Point2.from_pose(base_pose), Point2.from_pose(self.target)
        )
