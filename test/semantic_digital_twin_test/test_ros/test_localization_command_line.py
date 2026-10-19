from __future__ import annotations

import time
from copy import deepcopy

import numpy as np
import pytest
from typing_extensions import Callable

from semantic_digital_twin.adapters.ros.exceptions import (
    RobotToLocalizeNotFound,
    RobotWithoutBaseLidar,
)
from semantic_digital_twin.adapters.ros.localization_command_line import (
    LocalizationCommandLine,
)
from semantic_digital_twin.adapters.ros.world_fetcher import FetchWorldServer
from semantic_digital_twin.adapters.ros.world_synchronizer import WorldSynchronizer
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.localization.motion_models import OmniDriveMotionModel
from semantic_digital_twin.robots.garmi import Garmi
from semantic_digital_twin.robots.stretch import Stretch
from semantic_digital_twin.spatial_types.spatial_types import (
    HomogeneousTransformationMatrix,
    Pose2D,
)
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import FixedConnection
from semantic_digital_twin.world_description.world_entity import Body

# %% helpers


def wait_until(condition: Callable[[], bool], timeout: float = 5.0) -> bool:
    """
    :return: Whether the condition became true within the timeout, in seconds.
    """
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if condition():
            return True
        time.sleep(0.05)
    return condition()


def serve(world: World, rclpy_node):
    """
    Lets other processes fetch a world and follow its changes.

    :return: The server and the synchronizer, to be closed by the caller.
    """
    return FetchWorldServer(node=rclpy_node, world=world), WorldSynchronizer(
        node=rclpy_node, _world=world
    )


@pytest.fixture
def served_stretch_apartment(rclpy_node, stretch_apartment_world):
    """
    A Stretch in the apartment, served to the localization.
    """
    world = deepcopy(stretch_apartment_world)
    server, synchronizer = serve(world, rclpy_node)
    yield world
    synchronizer.close()
    server.close()


# %% what the localization is built from


def test_the_localization_drives_the_robot_of_the_fetched_world(
    rclpy_node, served_stretch_apartment
):
    stretch = served_stretch_apartment.get_semantic_annotations_by_type(Stretch)[0]

    process = LocalizationCommandLine().start(rclpy_node)

    assert process.localization_node.drive.id == stretch.drive.id
    assert process.localization_node.lidar.root.id == stretch.mobile_base.lidar.root.id
    process.close()


def test_the_localization_starts_where_the_twin_places_the_base(
    rclpy_node, served_stretch_apartment
):
    stretch = served_stretch_apartment.get_semantic_annotations_by_type(Stretch)[0]
    world_T_base = served_stretch_apartment.compute_forward_kinematics(
        served_stretch_apartment.root, stretch.drive.child
    )

    process = LocalizationCommandLine().start(rclpy_node)

    np.testing.assert_allclose(
        process.localization_node.start_pose.to_np(),
        Pose2D.from_pose(world_T_base.pose).to_np(),
        atol=1e-9,
    )
    process.close()


def test_a_global_localization_searches_the_whole_region(
    rclpy_node, served_stretch_apartment
):
    process = LocalizationCommandLine(search_whole_region=True).start(rclpy_node)

    assert process.localization_node.start_pose is None
    process.close()


def test_the_region_covers_every_body_of_the_environment(
    rclpy_node, served_stretch_apartment
):
    stretch = served_stretch_apartment.get_semantic_annotations_by_type(Stretch)[0]
    environment = [
        body
        for body in served_stretch_apartment.bodies_with_collision
        if body not in stretch.bodies
    ]

    process = LocalizationCommandLine().start(rclpy_node)

    region = process.localization_node.region
    for body in environment:
        bounds = (
            body.collision.as_bounding_box_collection_in_frame(
                served_stretch_apartment.root
            )
            .bounding_box()
            .to_array_bounds()
        )
        assert region.min_x <= bounds.lower[0] + 1e-9
        assert region.min_y <= bounds.lower[1] + 1e-9
        assert bounds.upper[0] <= region.max_x + 1e-9
        assert bounds.upper[1] <= region.max_y + 1e-9
    process.close()


def test_the_twin_follows_changes_of_the_served_world(
    rclpy_node, served_stretch_apartment
):
    process = LocalizationCommandLine().start(rclpy_node)
    box_name = PrefixedName("box_placed_after_the_start")

    with served_stretch_apartment.modify_world():
        box = Body(name=box_name)
        served_stretch_apartment.add_connection(
            FixedConnection(parent=served_stretch_apartment.root, child=box)
        )

    assert wait_until(
        lambda: bool(process.world.get_kinematic_structure_entities_by_name(box_name))
    )
    process.close()


@pytest.fixture
def served_garmi_apartment(rclpy_node, _garmi_world_setup, _apartment_world_setup):
    """
    GARMI in the apartment, served to the localization.
    """
    world = deepcopy(_apartment_world_setup)
    world.merge_world_at_pose(
        deepcopy(_garmi_world_setup),
        HomogeneousTransformationMatrix.from_xyz_rpy(1.5, 2, 0),
    )
    server, synchronizer = serve(world, rclpy_node)
    yield world
    synchronizer.close()
    server.close()


def test_garmi_is_localized_from_its_merged_scan_with_its_omni_drive(
    rclpy_node, served_garmi_apartment
):
    garmi = served_garmi_apartment.get_semantic_annotations_by_type(Garmi)[0]

    process = LocalizationCommandLine().start(rclpy_node)

    assert process.localization_node.lidar.root.id == garmi.mobile_base.lidar.root.id
    assert type(process.localization_node.localization.motion_model) is (
        OmniDriveMotionModel
    )
    process.close()


def test_the_localization_reads_the_tf_frames_it_is_given(
    rclpy_node, served_stretch_apartment
):
    settings = LocalizationCommandLine(
        map_frame="world", odom_frame="stretch/odom", base_frame="stretch/base_link"
    )

    process = settings.start(rclpy_node)

    assert (
        process.localization_node.map_frame,
        process.localization_node.odom_frame,
        process.localization_node.base_frame,
    ) == (settings.map_frame, settings.odom_frame, settings.base_frame)
    process.close()


# %% robots that cannot be localized


def test_a_robot_name_no_robot_carries_is_rejected(
    rclpy_node, served_stretch_apartment
):
    with pytest.raises(RobotToLocalizeNotFound):
        LocalizationCommandLine(robot_name="no_such_robot").start(rclpy_node)


def test_a_robot_without_a_lidar_on_its_base_is_rejected(rclpy_node, tracy_world):
    server, synchronizer = serve(deepcopy(tracy_world), rclpy_node)

    with pytest.raises(RobotWithoutBaseLidar):
        LocalizationCommandLine().start(rclpy_node)
    synchronizer.close()
    server.close()
