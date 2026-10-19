from __future__ import annotations

import time
from uuid import uuid4
from dataclasses import dataclass, field

import numpy as np
import pytest
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster
from typing_extensions import Callable

from semantic_digital_twin.adapters.ros.exceptions import LidarWithoutScanTopic
from semantic_digital_twin.adapters.ros.localization import (
    CoordinateFrame,
    LocalizationNode,
    LocalizationTopic,
)
from semantic_digital_twin.adapters.ros.ros2_to_semdt_converters import (
    PoseToSemDTConverter,
)
from semantic_digital_twin.adapters.ros.semdt_to_ros2_converters import (
    HomogeneousTransformationMatrixToRos2Converter,
    PoseToRos2Converter,
)
from semantic_digital_twin.adapters.ros.tfwrapper import TFWrapper
from semantic_digital_twin.adapters.sensors.lidar import Lidar
from semantic_digital_twin.datastructures.lidar_reading import LidarReading
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.spatial_types.spatial_types import (
    HomogeneousTransformationMatrix,
    Pose2D,
)
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import (
    Connection6DoF,
    DifferentialDrive,
    FixedConnection,
)
from semantic_digital_twin.world_description.world_entity import Body

from ..test_localization.room import (
    FullCircleLidar,
    Room,
    SimulatedRobot,
    position_error,
    yaw_error,
)

# %% the twin the node localizes in

ROBOT_NAME = "simulated_robot"
"""
The prefix the twin gives the robot's links, as a robot description's name does.
"""


@dataclass
class RobotTwin:
    """
    A copy of the room holding a model of the robot: its odometry frame localized in the
    room, its base driven within that frame and its lidar mounted on the base.

    Its bodies are named as a robot specification names them, with prefixes the robot's
    own tf frames do not carry.
    """

    world: World
    """
    The room with the robot in it.
    """

    drive: DifferentialDrive
    """
    The drive moving the base within the odometry frame.
    """

    lidar: Lidar
    """
    The lidar on the base.
    """

    @classmethod
    def in_room(cls, room: Room, robot: SimulatedRobot) -> RobotTwin:
        """
        :param robot: The simulated robot whose lidar mount the twin copies.
        :return: A twin of the robot in a copy of the room.
        """
        world = room.build()
        odom_identifier = uuid4()
        odom = Body(
            name=PrefixedName(CoordinateFrame.ODOM, prefix=str(odom_identifier)),
            id=odom_identifier,
        )
        base = Body(name=PrefixedName(CoordinateFrame.BASE_LINK, prefix=ROBOT_NAME))
        lidar_body = Body(
            name=PrefixedName(robot.lidar.root.name.name, prefix=ROBOT_NAME)
        )
        with world.modify_world():
            world.add_connection(
                Connection6DoF.create_with_dofs(
                    world=world, parent=world.root, child=odom
                )
            )
            drive = DifferentialDrive.create_with_dofs(
                world=world, parent=odom, child=base
            )
            world.add_connection(drive)
            world.add_connection(
                FixedConnection(
                    parent=base,
                    child=lidar_body,
                    parent_T_connection_expression=HomogeneousTransformationMatrix(
                        robot.base_T_lidar.to_np()
                    ),
                )
            )
        return cls(
            world=world,
            drive=drive,
            lidar=FullCircleLidar.with_simulated_source(lidar_body),
        )


@dataclass(eq=False)
class SilentLidar(FullCircleLidar):
    """
    A lidar whose description names no topic.
    """

    topic_name = None


# %% what the robot publishes


@dataclass
class RobotPublisher:
    """
    Publishes the odometry of a simulated robot on tf and the sweeps of its lidar, as
    the drivers of a real robot do.
    """

    node: Node
    """
    The node publishing.
    """

    robot: SimulatedRobot
    """
    The robot whose odometry and sweeps are published.
    """

    twin: RobotTwin
    """
    The twin naming the frames.
    """

    transform_broadcaster: TransformBroadcaster = field(init=False)
    """
    Publishes the odometry.
    """

    scan_publisher: Publisher = field(init=False)
    """
    Publishes the sweeps.
    """

    initial_pose_publisher: Publisher = field(init=False)
    """
    Publishes poses the localization is to restart around.
    """

    def __post_init__(self):
        self.transform_broadcaster = TransformBroadcaster(self.node)
        self.scan_publisher = self.node.create_publisher(
            LaserScan, self.twin.lidar.topic_name, 10
        )
        self.initial_pose_publisher = self.node.create_publisher(
            PoseWithCovarianceStamped, LocalizationTopic.INITIAL_POSE, 10
        )

    def publish_odometry(self, stamp: Time) -> None:
        """
        Publishes where the robot's odometry places its base at a time.
        """
        odom_T_base = HomogeneousTransformationMatrixToRos2Converter.convert(
            HomogeneousTransformationMatrix(self.robot.odometry.to_np())
        )
        odom_T_base.header.frame_id = CoordinateFrame.ODOM
        odom_T_base.child_frame_id = CoordinateFrame.BASE_LINK
        odom_T_base.header.stamp = stamp.to_msg()
        self.transform_broadcaster.sendTransform(odom_T_base)

    def publish_scan(self, stamp: Time) -> None:
        """
        Publishes a sweep of the robot's lidar taken at a time.
        """
        reading: LidarReading = self.robot.read_scan()
        pattern = reading.scan_pattern
        scan = LaserScan()
        scan.header.frame_id = self.twin.lidar.root.name.name
        scan.header.stamp = stamp.to_msg()
        scan.angle_min = pattern.minimum_angle
        scan.angle_max = pattern.maximum_angle
        scan.angle_increment = pattern.angle_increment
        scan.range_min = pattern.minimum_range
        scan.range_max = pattern.maximum_range
        scan.ranges = reading.ranges.astype(float).tolist()
        self.scan_publisher.publish(scan)

    def publish_initial_pose(self, pose: Pose2D) -> None:
        """
        Publishes a pose of the robot's base in the room, as a person sets one in RViz.
        """
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.frame_id = CoordinateFrame.MAP
        initial_pose.pose.pose = PoseToRos2Converter.convert(
            pose.homogeneous_matrix.pose
        )
        initial_pose.pose.covariance = np.diag([0.01, 0.01, 0, 0, 0, 0.0025]).ravel()
        self.initial_pose_publisher.publish(initial_pose)


# %% helpers


def published_estimate(
    tf_wrapper: TFWrapper,
    twin: RobotTwin,
    localization_node: LocalizationNode,
    scan_stamp: Time,
) -> HomogeneousTransformationMatrix | None:
    """
    :return: The pose of the odometry frame in the room the node published for the
        sweep taken at a time, or None if it published none within a second.
    """
    published_stamp = scan_stamp + Duration(
        seconds=localization_node.transform_tolerance
    )
    if not tf_wrapper.wait_for_transform(
        CoordinateFrame.MAP,
        CoordinateFrame.ODOM,
        published_stamp,
        Duration(seconds=1.0),
    ):
        return None
    return PoseToSemDTConverter.convert(
        tf_wrapper.lookup_pose(
            CoordinateFrame.MAP, CoordinateFrame.ODOM, published_stamp.to_msg()
        ).pose,
        twin.world,
    ).homogeneous_matrix


def drive_and_publish(
    publisher: RobotPublisher, forward: float, rotation: float
) -> Time:
    """
    Drives the robot and publishes its odometry and then a sweep, both stamped now.

    :return: The stamp of both.
    """
    publisher.robot.drive(forward, rotation)
    stamp = publisher.node.get_clock().now()
    publisher.publish_odometry(stamp)
    publisher.publish_scan(stamp)
    return stamp


def wait_until(condition: Callable[[], bool], timeout: float = 1.0) -> bool:
    """
    :return: Whether the condition became true within the timeout, in seconds.
    """
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if condition():
            return True
        time.sleep(0.01)
    return condition()


def estimated_pose(
    world_T_odom: HomogeneousTransformationMatrix, robot: SimulatedRobot
) -> Pose2D:
    """
    :return: Where a published estimate places the robot's base at its current
        odometry reading.
    """
    return Pose2D.from_pose((world_T_odom @ robot.odometry).pose)


@pytest.fixture
def scene(rclpy_node):
    """
    A simulated robot in the room, its twin, a publisher of what it measures and a tf
    listener.
    """
    room = Room()
    robot = SimulatedRobot(
        room.build(), Pose2D(x=3.0, y=1.0, yaw=0.0), np.random.default_rng(0)
    )
    twin = RobotTwin.in_room(room, robot)
    return (
        room,
        robot,
        twin,
        RobotPublisher(node=rclpy_node, robot=robot, twin=twin),
        TFWrapper(node=rclpy_node),
    )


# %% localizing


def test_the_node_publishes_where_the_odometry_frame_stands_in_the_room(
    rclpy_node, scene
):
    room, robot, twin, publisher, tf_wrapper = scene
    localization_node = LocalizationNode(
        node=rclpy_node,
        drive=twin.drive,
        lidar=twin.lidar,
        region=room.floor,
        start_pose=Pose2D(x=3.15, y=0.9, yaw=0.1),
    )

    for _ in range(10):
        stamp = drive_and_publish(publisher, 0.2, 0.2)
        world_T_odom = published_estimate(tf_wrapper, twin, localization_node, stamp)
        assert world_T_odom is not None

    estimate = estimated_pose(world_T_odom, robot)
    assert position_error(estimate, robot.pose) < 0.1
    assert yaw_error(estimate, robot.pose) < 0.05
    localization_node.close()


def test_a_sweep_without_odometry_at_its_stamp_is_skipped(rclpy_node, scene):
    room, robot, twin, publisher, tf_wrapper = scene
    localization_node = LocalizationNode(
        node=rclpy_node, drive=twin.drive, lidar=twin.lidar, region=room.floor
    )

    stamp = rclpy_node.get_clock().now()
    publisher.publish_scan(stamp)

    assert published_estimate(tf_wrapper, twin, localization_node, stamp) is None
    localization_node.close()


def test_an_initial_pose_restarts_the_localization_around_it(rclpy_node, scene):
    room, robot, twin, publisher, tf_wrapper = scene
    localization_node = LocalizationNode(
        node=rclpy_node,
        drive=twin.drive,
        lidar=twin.lidar,
        region=room.floor,
        start_pose=Pose2D(x=1.5, y=3.0, yaw=np.pi),
    )
    stamp = drive_and_publish(publisher, 0.0, 0.0)
    assert published_estimate(tf_wrapper, twin, localization_node, stamp) is not None

    publisher.publish_initial_pose(robot.pose)
    assert wait_until(lambda: localization_node.localization.estimate is None)
    stamp = drive_and_publish(publisher, 0.0, 0.0)

    world_T_odom = published_estimate(tf_wrapper, twin, localization_node, stamp)
    assert position_error(estimated_pose(world_T_odom, robot), robot.pose) < 0.1
    localization_node.close()


def test_a_lidar_without_a_scan_topic_is_rejected(rclpy_node, scene):
    room, robot, twin, publisher, tf_wrapper = scene
    with pytest.raises(LidarWithoutScanTopic):
        LocalizationNode(
            node=rclpy_node,
            drive=twin.drive,
            lidar=SilentLidar.with_simulated_source(twin.lidar.root),
            region=room.floor,
        )
