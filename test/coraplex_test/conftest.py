# %% ORM interfaces

# Built before the imports below, which read a mapped datastructure: pytest imports every
# conftest of a run before calling any hook, so a hook would fire too late. The build runs
# once per process and never on an xdist worker.
from ..orm_interface_build import regenerate_orm_interfaces

regenerate_orm_interfaces()


from functools import partial

import pytest

from typing_extensions import List, Optional

from coraplex.datastructures.dataclasses import Context
from cramph.node import StatechartNode
from cramph.statechart import Statechart
from giskardpy.motion_statechart.tasks.cartesian_tasks import CartesianPose
from semantic_digital_twin.predetermined_maps.building_floor import BuildingFloor
from semantic_digital_twin.spatial_types.spatial_types import Pose
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix
from semantic_digital_twin.world import World

try:
    import rclpy
except ModuleNotFoundError:
    pass
from sqlalchemy.orm import sessionmaker

from krrood.ormatic.utils import create_engine, drop_database

try:
    from coraplex.datastructures.dataclasses import Context
except ModuleNotFoundError:
    pass

try:
    from coraplex.orm.ormatic_interface import Base
except ImportError:
    pass
try:
    from semantic_digital_twin.adapters.ros.visualization.viz_marker import (
        VizMarkerPublisher,
    )
except ModuleNotFoundError:
    pass
from semantic_digital_twin.robots.pr2 import PR2
from semantic_digital_twin.robots.stretch import Stretch
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix
from semantic_digital_twin.world_description.geometry import VolumetricBoundingBox

from .world_snapshot import WorldSnapshot
from semantic_digital_twin.robots.robot_parts import AbstractRobot, Arm

from ..conftest import SAMPLING_SEED

# %% the arm a test runs with on any robot


def left_or_only_arm(robot: AbstractRobot) -> Arm:
    """
    :return: The left arm of a robot that names one, otherwise its first arm.
    """
    return robot.get_left_arm_if_specified() or robot.all_arms[0]


def right_or_only_arm(robot: AbstractRobot) -> Arm:
    """
    :return: The right arm of a robot that names one, otherwise its first arm.
    """
    return robot.get_right_arm_if_specified() or robot.all_arms[0]


@pytest.fixture(scope="session")
def viz_marker_publisher():
    rclpy.init()
    node = rclpy.create_node("test_viz_marker_publisher")
    # VizMarkerPublisher(world, node)  # Initialize the publisher
    yield partial(VizMarkerPublisher, node=node)
    rclpy.shutdown()


# %% world rollback


@pytest.fixture(scope="function")
def pr2_apartment_context(pr2_apartment_world):
    """
    The shared PR2 apartment world, its robot and a context for both, returned to its
    initial model and state after the test.
    """
    snapshot = WorldSnapshot.capture(pr2_apartment_world)
    pr2 = pr2_apartment_world.get_semantic_annotations_by_type(PR2)[0]
    yield pr2_apartment_world, pr2, Context(
        pr2_apartment_world, pr2, sampling_seed=SAMPLING_SEED
    )
    snapshot.restore()


@pytest.fixture(scope="function")
def simple_pr2_context(simple_pr2_world_setup):
    """
    The shared PR2 world in the simple apartment, its robot and a context for both,
    returned to its initial model and state after the test.
    """
    world, robot_view, context = simple_pr2_world_setup
    snapshot = WorldSnapshot.capture(world)
    yield world, robot_view, context
    snapshot.restore()


@pytest.fixture(scope="function")
def stretch_apartment_context(stretch_apartment_world):
    """
    The shared Stretch apartment world, its robot and a context for both, returned to
    its initial model and state after the test.
    """
    snapshot = WorldSnapshot.capture(stretch_apartment_world)
    robot = stretch_apartment_world.get_semantic_annotations_by_type(Stretch)[0]
    yield stretch_apartment_world, robot, Context(
        stretch_apartment_world, robot, sampling_seed=SAMPLING_SEED
    )
    snapshot.restore()


# %% database session


@pytest.fixture(scope="function")
def coraplex_testing_session():
    engine = create_engine("sqlite:///:memory:")
    session_maker = sessionmaker(engine)
    session = session_maker()
    Base.metadata.create_all(bind=session.bind)
    yield session
    drop_database(session.bind)
    session.close()
    engine.dispose()


# %% perception regions


@pytest.fixture
def whole_scene_region(pr2_apartment_context) -> VolumetricBoundingBox:
    """
    A region large enough to contain everything in the apartment fixture.

    Lets a perception test say "look everywhere" without restating the extents.
    """
    world, _, _ = pr2_apartment_context
    return VolumetricBoundingBox(
        origin=HomogeneousTransformationMatrix(reference_frame=world.root),
        min_x=-10,
        min_y=-10,
        min_z=-10,
        max_x=10,
        max_y=10,
        max_z=10,
    )


# %% building the giskard goals a test plan is made of


def tool_center_point_goal(
    context: Context,
    arm: Optional[Arm] = None,
    target: Optional[Pose] = None,
) -> CartesianPose:
    """
    Build the goal an action would build to move an arm's tool center point.

    Lets a test about plans and charts state which arm moves where without restating how
    an action assembles that goal.

    :param context: The context the plan runs in, supplying the robot and the link the
        goal is expressed relative to.
    :param arm: Which arm's tool center point moves, the left or only arm of the
        context's robot by default.
    :param target: Where it should end up, the world's origin by default.
    :return: The goal moving that tool center point there.
    """
    return CartesianPose(
        root_link=context.controlled_root,
        tip_link=(
            left_or_only_arm(context.robot) if arm is None else arm
        ).end_effector.tool_frame,
        goal_pose=(
            Pose(reference_frame=context.world.root) if target is None else target
        ),
    )


def expand(plan: StatechartNode, context: Context) -> StatechartNode:
    """
    Expand `plan` in a statechart of its own, without running it, so a test can read the
    nodes its steps expand into.

    :param plan: The plan to expand.
    :param context: The context the plan is expanded in.
    :return: The plan, expanded.
    """
    statechart = Statechart(context=context.create_statechart_context())
    statechart.add_node(plan)
    return plan


def motion_nodes_of(plan: StatechartNode, context: Context) -> List[StatechartNode]:
    """
    :param plan: The plan whose motions to read.
    :param context: The context the plan is expanded in.
    :return: Every node the plan's steps expand into, so a test can look for a task
        without knowing whether the action wrapped it alongside speed caps or collision
        rules.
    """
    root = expand(plan, context)
    return [root, *root.descendants]
