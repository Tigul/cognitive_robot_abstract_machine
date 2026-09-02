from __future__ import annotations

import math
from dataclasses import dataclass

import pytest
from typing_extensions import Type

from semantic_digital_twin.adapters.urdf import URDFParser
from semantic_digital_twin.robots.hsrb import HSRB, HSRBBaseLaserScanner
from semantic_digital_twin.robots.pr2 import PR2, PR2BaseLaserScanner
from semantic_digital_twin.robots.robot_parts import AbstractRobot, LaserScanner
from semantic_digital_twin.robots.stretch import Stretch, StretchBaseLaserScanner
from semantic_digital_twin.robots.tiago import Tiago, TiagoBaseLaserScanner

# %% the robots carrying a base scanner


@dataclass(frozen=True)
class ScannerCase:
    """
    One robot's base laser scanner, as its description mounts it.
    """

    robot: Type[AbstractRobot]
    """
    The robot the scanner is mounted on.
    """

    scanner: Type[LaserScanner]
    """
    The scanner the robot's mobile base is expected to carry.
    """

    laser_link: str
    """
    The body the robot's description mounts the scanner on.
    """

    def annotate_own_world(self) -> AbstractRobot:
        """
        :return: This robot, annotated in a world parsed from its own description.
        """
        return self.robot.from_world(
            URDFParser.from_file(self.robot.get_ros_file_path()).parse()
        )


SCANNER_CASES = [
    ScannerCase(PR2, PR2BaseLaserScanner, "base_laser_link"),
    ScannerCase(HSRB, HSRBBaseLaserScanner, "base_range_sensor_link"),
    ScannerCase(Tiago, TiagoBaseLaserScanner, "base_laser_link"),
    ScannerCase(Stretch, StretchBaseLaserScanner, "laser"),
]


@pytest.fixture(
    scope="module", params=SCANNER_CASES, ids=lambda case: case.robot.__name__
)
def scanner_case(request) -> tuple[ScannerCase, AbstractRobot]:
    """
    Annotates one robot in a world of its own, kept only for as long as this module
    runs.
    """
    case: ScannerCase = request.param
    return case, case.annotate_own_world()


# %% the scanner the mobile base carries


def test_the_mobile_base_carries_the_robots_scanner(scanner_case):
    case, robot = scanner_case

    assert isinstance(robot.mobile_base.laser_scanner, case.scanner)


def test_the_scanner_sits_on_the_link_its_description_names(scanner_case):
    case, robot = scanner_case

    assert robot.mobile_base.laser_scanner.root.name.name == case.laser_link


def test_the_scanner_is_one_of_the_robots_sensors(scanner_case):
    _, robot = scanner_case

    assert robot.mobile_base.laser_scanner in robot.get_sensors()


# %% the readings the mobile base hands back


def test_the_mobile_base_reports_one_measurement_per_beam(scanner_case):
    _, robot = scanner_case
    beam_count = robot.mobile_base.laser_scanner.laser_source.scan_pattern.beam_count

    reading = robot.mobile_base.get_laser_reading()

    assert len(reading.direction) == len(reading.distance) == beam_count


def test_the_beams_are_expressed_in_the_scanners_own_frame(scanner_case):
    _, robot = scanner_case
    scanner = robot.mobile_base.laser_scanner

    reading = robot.mobile_base.get_laser_reading()

    assert {direction.reference_frame for direction in reading.direction} == {
        scanner.root
    }


# %% a scanner reading a world it stands in


def test_a_scanner_in_a_furnished_world_measures_the_surfaces_around_it(
    pr2_apartment_world,
):
    robot = pr2_apartment_world.get_semantic_annotations_by_type(PR2)[0]

    reading = robot.mobile_base.get_laser_reading()

    assert any(math.isfinite(distance) for distance in reading.distance)
