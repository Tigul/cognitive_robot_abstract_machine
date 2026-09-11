from __future__ import annotations

from dataclasses import dataclass, field

from rclpy.node import Node
from rclpy.subscription import Subscription
from sensor_msgs.msg import LaserScan
from typing_extensions import Optional

from semantic_digital_twin.adapters.ros.ros2_to_semdt_converters import (
    LaserScanToSemDTConverter,
)
from semantic_digital_twin.adapters.sensors.lidar import Lidar, LidarSource
from semantic_digital_twin.datastructures.lidar_reading import LidarReading
from semantic_digital_twin.datastructures.scan_pattern import ScanPattern
from semantic_digital_twin.exceptions import NoLaserScanReceived


@dataclass
class SubscribedLidarSource(LidarSource):
    """
    A source that reports what a real scanner publishes on a ROS 2 topic.

    The scanner itself decides what it sweeps, so reading it adopts the pattern the
    received scan was taken with.
    """

    node: Node = field(kw_only=True)
    """
    The node the scans are received on.
    """

    topic_name: str = field(kw_only=True)
    """
    The topic the scans are published on.
    """

    latest_scan: Optional[LaserScan] = field(default=None, init=False)
    """
    The most recently received scan, or ``None`` while none has arrived.
    """

    subscription: Subscription = field(init=False, repr=False)
    """
    The subscription the scans arrive through.
    """

    def __post_init__(self):
        self.subscription = self.node.create_subscription(
            LaserScan,
            topic=self.topic_name,
            callback=self.store_scan,
            qos_profile=10,
        )

    def store_scan(self, scan: LaserScan) -> None:
        """
        Keeps a received scan as the one this source reports.

        :param scan: The scan that was received.
        """
        self.latest_scan = scan

    @property
    def received_scan(self) -> LaserScan:
        """
        :return: The most recently received scan.
        :raises NoLaserScanReceived: If no scan has arrived yet.
        """
        if self.latest_scan is None:
            raise NoLaserScanReceived(self.topic_name)
        return self.latest_scan

    def get_lidar_reading(self, lidar: Lidar) -> LidarReading:
        scan = self.received_scan
        lidar.scan_pattern = ScanPattern(
            minimum_angle=scan.angle_min,
            maximum_angle=scan.angle_max,
            angle_increment=scan.angle_increment,
            minimum_range=scan.range_min,
            maximum_range=scan.range_max,
        )
        return LaserScanToSemDTConverter.convert(scan, lidar.root._world)
