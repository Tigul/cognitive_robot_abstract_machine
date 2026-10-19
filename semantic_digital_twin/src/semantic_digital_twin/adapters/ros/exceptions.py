from __future__ import annotations

from dataclasses import dataclass, field
from typing import Type

from krrood.exceptions import DataclassException
from semantic_digital_twin.adapters.sensors.lidar import Lidar
from semantic_digital_twin.robots.robot_parts import AbstractRobot


@dataclass
class ROS2ConversionError(DataclassException):
    """
    Base class for errors that occur during ROS2 message conversion.
    """


@dataclass
class CannotConvertSemDTToRos2Error(ROS2ConversionError):
    """
    Raised when a semDT object cannot be converted to a ROS2 message.
    """

    data_type: Type = field(kw_only=True)
    """
    The type of the object that could not be converted.
    """

    def error_message(self) -> str:
        return f"Cannot convert {self.data_type.__name__} to ROS2 message."

    def suggest_correction(self) -> str:
        return ""


@dataclass
class CannotConvertRos2ToSemDTError(ROS2ConversionError):
    """
    Raised when a ROS2 message cannot be converted to a semDT object.
    """

    data_type: Type = field(kw_only=True)
    """
    The type of the message that could not be converted.
    """

    def error_message(self) -> str:
        return f"Cannot convert {self.data_type.__name__} to our semDT type."

    def suggest_correction(self) -> str:
        return ""


@dataclass
class LaserScanBeamCountMismatch(ROS2ConversionError):
    """
    Raised when a laser scan holds a different number of measurements than the beams its
    own angles describe.
    """

    beam_count: int = field(kw_only=True)
    """
    How many beams the scan's angles describe.
    """

    range_count: int = field(kw_only=True)
    """
    How many measurements the scan holds.
    """

    def error_message(self) -> str:
        return f"Laser scan describes {self.beam_count} beams but holds {self.range_count} measurements."

    def suggest_correction(self) -> str:
        return "check that the scan's angle_min, angle_max and angle_increment match its ranges."


@dataclass
class LidarWithoutScanTopic(DataclassException):
    """
    Raised when the sweeps of a lidar are to be received, but its description names no
    topic they are published on.
    """

    lidar: Lidar = field(kw_only=True)
    """
    The lidar whose topic is missing.
    """

    def error_message(self) -> str:
        return (
            f"{type(self.lidar).__name__} names no topic its sweeps are published on."
        )

    def suggest_correction(self) -> str:
        return f"give {type(self.lidar).__name__} a topic_name classproperty."


@dataclass
class RobotToLocalizeNotFound(DataclassException):
    """
    Raised when a fetched world holds no robot to localize, or not the one asked for.
    """

    robot_names: list[str] = field(kw_only=True)
    """
    The names of the robots the world holds.
    """

    requested_name: str | None = field(kw_only=True)
    """
    The name of the robot asked for, or None if the world was to hold exactly one robot.
    """

    def error_message(self) -> str:
        if self.requested_name is None:
            return f"Expected exactly one robot to localize, found {self.robot_names}."
        return f"No robot named {self.requested_name} among {self.robot_names}."

    def suggest_correction(self) -> str:
        return "name one of the robots the world holds."


@dataclass
class RobotWithoutBaseLidar(DataclassException):
    """
    Raised when a robot to localize has no mobile base carrying a lidar.
    """

    robot: AbstractRobot = field(kw_only=True)
    """
    The robot that cannot be localized.
    """

    def error_message(self) -> str:
        return f"{self.robot.name} has no mobile base carrying a lidar."

    def suggest_correction(self) -> str:
        return "localize a robot whose mobile base has a lidar."
