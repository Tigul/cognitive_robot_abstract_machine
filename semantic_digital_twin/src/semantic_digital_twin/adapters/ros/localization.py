from __future__ import annotations

from dataclasses import dataclass, field
from enum import StrEnum

import numpy as np
import numpy.typing as npt
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.duration import Duration
from rclpy.subscription import Subscription
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster

from semantic_digital_twin.adapters.ros.exceptions import LidarWithoutScanTopic
from semantic_digital_twin.adapters.ros.ros2_node import HasROS2Node
from semantic_digital_twin.adapters.ros.ros2_to_semdt_converters import (
    LaserScanToSemDTConverter,
    PoseToSemDTConverter,
)
from semantic_digital_twin.adapters.ros.semdt_to_ros2_converters import (
    HomogeneousTransformationMatrixToRos2Converter,
)
from semantic_digital_twin.adapters.ros.tfwrapper import TFWrapper
from semantic_digital_twin.adapters.sensors.lidar import Lidar
from semantic_digital_twin.localization.monte_carlo_localization import (
    MonteCarloLocalization,
    PoseEstimate,
)
from semantic_digital_twin.localization.motion_models import MotionModel
from semantic_digital_twin.localization.particles import ParticleSet
from semantic_digital_twin.localization.resampling import AdaptiveResampling
from semantic_digital_twin.spatial_computations.raytracer import RayTracer
from semantic_digital_twin.spatial_types.spatial_types import (
    HomogeneousTransformationMatrix,
    Pose2D,
)
from semantic_digital_twin.world_description.connections import WheeledDrive
from semantic_digital_twin.world_description.geometry import PlanarBoundingBox
from semantic_digital_twin.world_description.world_entity import Body

# %% topics


class LocalizationTopic(StrEnum):
    """
    The topics a localization node receives on besides the sweeps of its lidar.
    """

    INITIAL_POSE = "initialpose"
    """
    A pose of the robot's base in the world, as RViz's pose estimate tool publishes it.
    """


class CoordinateFrame(StrEnum):
    """
    The tf frames of a mobile robot, as REP 105 names them.
    """

    MAP = "map"
    """
    The frame of the world the robot is localized in.
    """

    ODOM = "odom"
    """
    The frame the odometry of the base is measured in.
    """

    BASE_LINK = "base_link"
    """
    The frame of the robot's base.
    """


# %% localization node


@dataclass
class LocalizationNode(HasROS2Node):
    """
    Localizes a robot base in a world from the sweeps of a lidar it carries and the
    odometry on tf, and publishes on tf where the base's odometry frame stands in the
    world root's frame.

    Every sweep is matched against the world's collision geometry without the robot's
    own, at the odometry pose tf holds for the sweep's stamp; a sweep for whose stamp tf
    holds no odometry is skipped.
    """

    drive: WheeledDrive
    """
    The drive moving the base within its odometry frame.
    """

    lidar: Lidar
    """
    The lidar on the base.
    """

    region: PlanarBoundingBox
    """
    The part of the world root's x-y plane the base may stand in.
    """

    map_frame: str = CoordinateFrame.MAP
    """
    The tf frame of the world root.
    """

    odom_frame: str = CoordinateFrame.ODOM
    """
    The tf frame of the parent of :attr:`drive`, the frame the odometry is measured in.
    """

    base_frame: str = CoordinateFrame.BASE_LINK
    """
    The tf frame of the child of :attr:`drive`, the base whose odometry is measured.
    """

    start_pose: Pose2D | None = None
    """
    Where the base is believed to start, or None to search the whole region.
    """

    start_covariance: npt.NDArray[np.float64] = field(
        default_factory=lambda: np.diag([0.25, 0.25, (np.pi / 12) ** 2])
    )
    """
    The covariance of the x, y and yaw of :attr:`start_pose` (AMCL's initial_cov_xx,
    initial_cov_yy and initial_cov_aa).
    """

    transform_tolerance: float = 1.0
    """
    How far past a sweep's stamp a published transform is stamped, in seconds, so that
    it stays valid until the next one arrives (AMCL's transform_tolerance).
    """

    random_generator: np.random.Generator = field(default_factory=np.random.default_rng)
    """
    The source of the localization's randomness.
    """

    localization: MonteCarloLocalization = field(init=False)
    """
    The filter estimating the base's pose.
    """

    tf_wrapper: TFWrapper = field(init=False)
    """
    Looks up the odometry at a sweep's stamp.
    """

    transform_broadcaster: TransformBroadcaster = field(init=False)
    """
    Publishes the estimated pose of the odometry frame.
    """

    scan_subscription: Subscription = field(init=False)
    """
    Receives the sweeps of :attr:`lidar`.
    """

    initial_pose_subscription: Subscription = field(init=False)
    """
    Receives poses the localization restarts around.
    """

    def __post_init__(self):
        if self.lidar.topic_name is None:
            raise LidarWithoutScanTopic(lidar=self.lidar)
        world = self.drive._world
        resampling = AdaptiveResampling()
        robot = world.get_kinematic_structure_entities_of_branch(self.drive.child)
        self.localization = MonteCarloLocalization(
            ray_tracer=RayTracer(
                world=world,
                excluded_bodies=[
                    entity for entity in robot if isinstance(entity, Body)
                ],
            ),
            base_T_lidar=world.compute_forward_kinematics(
                self.drive.child, self.lidar.root
            ),
            region=self.region,
            particles=self._start_particles(resampling.maximum_particle_count),
            motion_model=MotionModel.for_drive_type(type(self.drive)),
            resampling=resampling,
            random_generator=self.random_generator,
        )
        self.tf_wrapper = TFWrapper(node=self.node)
        self.transform_broadcaster = TransformBroadcaster(self.node)
        self.scan_subscription = self.node.create_subscription(
            LaserScan, self.lidar.topic_name, self.localize, 10
        )
        self.initial_pose_subscription = self.node.create_subscription(
            PoseWithCovarianceStamped,
            LocalizationTopic.INITIAL_POSE,
            self.restart_around,
            10,
        )

    def _start_particles(self, count: int) -> ParticleSet:
        """
        :param count: How many particles to draw.
        :return: Particles around :attr:`start_pose`, or spread over :attr:`region` if
            there is none.
        """
        if self.start_pose is None:
            return ParticleSet.uniform(self.region, count, self.random_generator)
        return ParticleSet.around(
            self.start_pose, self.start_covariance, count, self.random_generator
        )

    def localize(self, scan: LaserScan) -> None:
        """
        Weighs the localization by a sweep and publishes the estimate it leads to.

        :param scan: A sweep of :attr:`lidar`.
        """
        stamp = Time.from_msg(scan.header.stamp)
        if not self.tf_wrapper.tf_buffer.can_transform(
            self.odom_frame, self.base_frame, stamp
        ):
            self.node.get_logger().debug(
                f"Skipping a sweep: no odometry on tf at its stamp {stamp}."
            )
            return
        odom_T_base = PoseToSemDTConverter.convert(
            self.tf_wrapper.lookup_pose(
                self.odom_frame, self.base_frame, scan.header.stamp
            ).pose,
            self.drive._world,
        ).homogeneous_matrix
        estimate = self.localization.step(
            odom_T_base, LaserScanToSemDTConverter.convert(scan, self.drive._world)
        )
        self._publish(estimate, stamp)

    def _publish(self, estimate: PoseEstimate, stamp: Time) -> None:
        """
        Publishes where an estimate places the odometry frame in the world root's frame.

        :param estimate: The estimate of the latest sweep.
        :param stamp: The stamp of that sweep.
        """
        world_T_odom = HomogeneousTransformationMatrixToRos2Converter.convert(
            HomogeneousTransformationMatrix(estimate.world_T_odom.to_np())
        )
        world_T_odom.header.frame_id = self.map_frame
        world_T_odom.child_frame_id = self.odom_frame
        world_T_odom.header.stamp = (
            stamp + Duration(seconds=self.transform_tolerance)
        ).to_msg()
        self.transform_broadcaster.sendTransform(world_T_odom)

    def restart_around(self, initial_pose: PoseWithCovarianceStamped) -> None:
        """
        Restarts the localization from particles spread around a pose of the base.

        :param initial_pose: The pose of the base in the world root's frame, with the
            covariance of its x, y and yaw.
        """
        covariance = np.reshape(initial_pose.pose.covariance, (6, 6))
        planar_axes = [0, 1, 5]
        self.localization.restart_from(
            ParticleSet.around(
                Pose2D.from_pose(
                    PoseToSemDTConverter.convert(
                        initial_pose.pose.pose, self.drive._world
                    )
                ),
                covariance[np.ix_(planar_axes, planar_axes)],
                self.localization.resampling.maximum_particle_count,
                self.random_generator,
            )
        )

    def close(self) -> None:
        """
        Stops receiving sweeps and initial poses and publishing estimates.
        """
        self.node.destroy_subscription(self.scan_subscription)
        self.node.destroy_subscription(self.initial_pose_subscription)
        self.tf_wrapper.tf_listener.unregister()
        self.node.destroy_publisher(self.transform_broadcaster.pub_tf)
