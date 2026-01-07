import os
from copy import deepcopy
from functools import partial

import pytest
import rclpy

from pycram.datastructures.dataclasses import Context

from semantic_digital_twin.adapters.viz_marker import VizMarkerPublisher
from semantic_digital_twin.robots.hsrb import HSRB
from semantic_digital_twin.robots.pr2 import PR2


@pytest.fixture(scope="session")
def viz_marker_publisher():
    rclpy.init()
    node = rclpy.create_node("test_viz_marker_publisher")
    # VizMarkerPublisher(world, node)  # Initialize the publisher
    yield partial(VizMarkerPublisher, node=node)
    rclpy.shutdown()


@pytest.fixture(scope="function")
def mutable_model_world(pr2_apartment_world):
    world = deepcopy(pr2_apartment_world)
    pr2 = PR2.from_world(world)
    return world, pr2, Context(world, pr2)


@pytest.fixture(scope="function")
def immutable_model_world(pr2_apartment_world):
    world = pr2_apartment_world
    pr2 = pr2_apartment_world.get_semantic_annotations_by_type(PR2)[0]
    state = deepcopy(world.state.data)
    yield world, pr2, Context(world, pr2)
    world.state.data = state


@pytest.fixture
def immutable_simple_pr2_world(simple_pr2_world_setup):
    world, robot_view, context = simple_pr2_world_setup
    state = deepcopy(world.state.data)
    yield world, robot_view, context
    world.state.data = state


@pytest.fixture
def mutable_simple_pr2_world(simple_pr2_world_setup):
    world, robot_view, context = simple_pr2_world_setup
    copy_world = deepcopy(world)
    robot_view = world.get_semantic_annotations_by_type(PR2)[0]
    return copy_world, robot_view, Context(copy_world, robot_view)


@pytest.fixture
def immutable_simple_hsr_world(simple_hsr_world_setup):
    world, robot_view, context = simple_hsr_world_setup
    state = deepcopy(world.state.data)
    robot_view = world.get_semantic_annotations_by_type(HSRB)[0]
    yield world, robot_view, Context(world, robot_view)
    world.state.data = state


@pytest.fixture
def mutable_simple_hsr_world(simple_hsr_world_setup):
    world, robot_view, context = simple_hsr_world_setup
    copy_world = deepcopy(world)
    robot_view = world.get_semantic_annotations_by_type(HSRB)[0]
    return copy_world, robot_view, Context(copy_world, robot_view)
