import pytest

from pycram.datastructures.enums import (
    TorsoState,
    Arms,
    ApproachDirection,
    VerticalAlignment,
)
from pycram.datastructures.grasp import GraspDescription
from pycram.datastructures.pose import PoseStamped
from pycram.language import SequentialPlan
from pycram.process_module import simulated_robot
from pycram.robot_plans import (
    NavigateActionDescription,
    MoveTorsoActionDescription,
    ParkArmsActionDescription,
    ReachActionDescription,
)


def test_hsr_navigate(immutable_simple_hsr_world):
    world, robot_view, context = immutable_simple_hsr_world

    nav = NavigateActionDescription(PoseStamped.from_list([1, 1, 0], frame=world.root))

    plan = SequentialPlan(context, nav)

    with simulated_robot:
        plan.perform()

    base_pose = PoseStamped.from_spatial_type(robot_view.root.global_pose)
    assert base_pose.position.to_list() == pytest.approx([1, 1, 0], abs=0.01)


def test_hsr_move_torso(immutable_simple_hsr_world):
    world, robot_view, context = immutable_simple_hsr_world

    move_torso = MoveTorsoActionDescription(TorsoState.HIGH)
    plan = SequentialPlan(context, move_torso)

    with simulated_robot:
        plan.perform()

    dof = world.get_degree_of_freedom_by_name("torso_lift_joint")
    assert world.state[dof.id].position == pytest.approx(0.3, abs=0.01)


def test_hsr_park_arm(immutable_simple_hsr_world):
    world, robot_view, context = immutable_simple_hsr_world

    park_arm = ParkArmsActionDescription(Arms.LEFT)

    plan = SequentialPlan(context, park_arm)

    with simulated_robot:
        plan.perform()


def test_hsr_reach_to_pick_up(immutable_simple_hsr_world):
    world, robot_view, context = immutable_simple_hsr_world
    grasp_description = GraspDescription(
        ApproachDirection.FRONT, VerticalAlignment.NoAlignment, False
    )

    reach = ReachActionDescription(
        target_pose=PoseStamped.from_spatial_type(
            world.get_body_by_name("milk.stl").global_pose
        ),
        object_designator=world.get_body_by_name("milk.stl"),
        arm=Arms.LEFT,
        grasp_description=grasp_description,
    )

    plan = SequentialPlan(
        context,
        NavigateActionDescription(PoseStamped.from_list([1, 1, 0], frame=world.root)),
        reach,
    )

    with simulated_robot:
        plan.perform()
