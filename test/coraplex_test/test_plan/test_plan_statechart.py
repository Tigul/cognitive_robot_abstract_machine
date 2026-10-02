"""
Tests for running a plan as one statechart, see
:class:`~coraplex.plans.plan_execution.PlanExecution`.
"""

from typing_extensions import List

from coraplex.datastructures.enums import ApproachDirection, Arms, VerticalAlignment
from coraplex.datastructures.grasp import GraspDescription
from coraplex.execution_environment import simulated_robot
from coraplex.locations.base import DeferredLocation
from coraplex.plans.factories import sequential
from coraplex.robot_plans.actions.core.navigation import NavigateAction
from coraplex.robot_plans.actions.core.pick_up import PickUpAction
from coraplex.robot_plans.actions.core.placing import PlaceAction
from coraplex.robot_plans.actions.core.robot_body import MoveTorsoAction
from cramph.data_types import LifeCycleValues
from cramph.statechart import Statechart
from cramph.world_modification_nodes import MoveBranch
from giskardpy.motion_statechart.graph_node import EndMotion
from krrood.entity_query_language.factories import a, variable
from semantic_digital_twin.datastructures.definitions import TorsoState
from semantic_digital_twin.semantic_annotations.semantic_annotations import Milk
from semantic_digital_twin.spatial_types.spatial_types import Pose

# %% helpers


def _pose_in_front_of_the_robot(world, robot) -> Pose:
    """
    :return: A pose in the world frame, a few centimetres in front of where the robot
        stands now.
    """
    return world.transform(
        Pose.from_xyz_rpy(x=0.05, reference_frame=robot.root), world.root
    )


# %% one statechart per plan


def test_a_plan_grounding_an_action_mid_sequence_runs_as_one_statechart(
    immutable_model_world,
):
    world, robot, context = immutable_model_world
    torso = MoveTorsoAction(TorsoState.HIGH)
    plan = sequential(
        [
            torso,
            a(NavigateAction)(
                target_location=variable(
                    Pose, domain=[_pose_in_front_of_the_robot(world, robot)]
                )
            ),
        ],
        context,
    )

    with simulated_robot:
        plan.perform()

    statechart: Statechart = plan.root.statechart
    navigate = plan.root.nodes[1].latest_child
    assert isinstance(navigate, NavigateAction)
    assert navigate.statechart is statechart
    assert torso.statechart is statechart
    assert len(statechart.get_nodes_by_type(EndMotion)) == 1
    assert plan.root.life_cycle_state == LifeCycleValues.SUCCEEDED


def test_an_underspecified_action_is_grounded_against_the_world_the_steps_before_left(
    immutable_model_world,
):
    world, robot, context = immutable_model_world
    torso_high = robot.get_torso().get_joint_state_by_type(TorsoState.HIGH)
    assert not torso_high.is_achieved()
    torso_high_when_grounded: List[bool] = []

    def poses_in_front_of_the_robot():
        torso_high_when_grounded.append(torso_high.is_achieved())
        return [_pose_in_front_of_the_robot(world, robot)]

    plan = sequential(
        [
            MoveTorsoAction(TorsoState.HIGH),
            a(NavigateAction)(
                target_location=variable(
                    Pose, domain=DeferredLocation(poses_in_front_of_the_robot)
                )
            ),
        ],
        context,
    )

    with simulated_robot:
        plan.perform()

    assert torso_high_when_grounded == [True]


def test_a_branch_moved_mid_plan_follows_its_new_parent(mutable_model_world):
    world, robot, context = mutable_model_world
    milk = world.get_body_by_name("milk.stl")
    tool_frame = robot.left_arm.end_effector.tool_frame
    height_before = milk.global_pose.z

    plan = sequential(
        [
            MoveTorsoAction(TorsoState.LOW),
            MoveBranch(body=milk, new_parent=tool_frame),
            MoveTorsoAction(TorsoState.HIGH),
        ],
        context,
    )
    with simulated_robot:
        plan.perform()

    assert milk.parent_connection.parent is tool_frame
    assert milk.global_pose.z > height_before


# %% actions as statechart nodes


def test_actions_with_the_same_parameters_are_different_nodes():
    first = MoveTorsoAction(TorsoState.HIGH)
    second = MoveTorsoAction(TorsoState.HIGH)

    assert first != second


def test_a_place_finds_the_grasp_of_the_pick_up_before_it(mutable_model_world):
    world, robot, context = mutable_model_world
    milk = world.get_semantic_annotations_by_type(Milk)[0]
    pick_up = PickUpAction(
        milk,
        Arms.LEFT,
        GraspDescription(
            ApproachDirection.FRONT,
            VerticalAlignment.NoAlignment,
            robot.left_arm.end_effector,
        ),
    )
    place = PlaceAction(
        milk.root,
        Pose.from_xyz_rpy(0.8, -1.9, 0.7, reference_frame=world.root),
        Arms.LEFT,
    )
    statechart = Statechart(context=context.create_statechart_context())

    statechart.add_node(sequential([pick_up, place]).root)

    assert place.find_earlier_action(PickUpAction) is pick_up
