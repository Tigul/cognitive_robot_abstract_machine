"""
Tests for running a plan as one statechart, see
:class:`~coraplex.plans.plan_execution.PlanExecution`.
"""

import json
from dataclasses import dataclass, field

from typing_extensions import List

from coraplex.datastructures.enums import ApproachDirection, Arms, VerticalAlignment
from coraplex.datastructures.grasp import GraspDescription
from coraplex.datastructures.dataclasses import Context
from coraplex.execution_environment import real_robot, simulated_robot
from coraplex.locations.base import DeferredLocation
from coraplex.plans.plan_execution import PlanExecutor, UnderspecifiedChildChooser
from coraplex.plans.underspecified import UnderspecifiedNode
from coraplex.robot_plans.actions.core.navigation import NavigateAction
from coraplex.robot_plans.actions.core.pick_up import PickUpAction
from coraplex.robot_plans.actions.core.placing import PlaceAction
from coraplex.robot_plans.actions.core.robot_body import MoveTorsoAction
from cramph.composites import ChildChooser, CompositeNodeChoosingItsChild, Sequence
from cramph.data_types import LifeCycleValues
from cramph.statechart import Statechart
from cramph.world_modification_nodes import MoveBranch
from giskardpy.motion_statechart.graph_node import EndMotion
from krrood.adapters.json_serializer import from_json, to_json
from krrood.entity_query_language.factories import a, variable
from semantic_digital_twin.adapters.world_entity_kwargs_tracker import (
    WorldEntityWithIDKwargsTracker,
)
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
    plan = Sequence(
        [
            torso,
            UnderspecifiedNode(
                statement=a(NavigateAction)(
                    target_location=variable(
                        Pose, domain=[_pose_in_front_of_the_robot(world, robot)]
                    )
                )
            ),
        ]
    )

    with simulated_robot:
        executor = PlanExecutor(context)
        executor.compile(plan)
        executor.execute()

    statechart: Statechart = plan.statechart
    navigate = plan.nodes[1].latest_child
    assert isinstance(navigate, NavigateAction)
    assert navigate.statechart is statechart
    assert torso.statechart is statechart
    assert len(statechart.get_nodes_by_type(EndMotion)) == 1
    assert plan.life_cycle_state == LifeCycleValues.SUCCEEDED


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

    plan = Sequence(
        [
            MoveTorsoAction(TorsoState.HIGH),
            UnderspecifiedNode(
                statement=a(NavigateAction)(
                    target_location=variable(
                        Pose, domain=DeferredLocation(poses_in_front_of_the_robot)
                    )
                )
            ),
        ]
    )

    with simulated_robot:
        executor = PlanExecutor(context)
        executor.compile(plan)
        executor.execute()

    assert torso_high_when_grounded == [True]


def test_a_branch_moved_mid_plan_follows_its_new_parent(mutable_model_world):
    world, robot, context = mutable_model_world
    milk = world.get_body_by_name("milk.stl")
    tool_frame = robot.left_arm.end_effector.tool_frame
    height_before = milk.global_pose.z

    plan = Sequence(
        [
            MoveTorsoAction(TorsoState.LOW),
            MoveBranch(body=milk, new_parent=tool_frame),
            MoveTorsoAction(TorsoState.HIGH),
        ]
    )
    with simulated_robot:
        executor = PlanExecutor(context)
        executor.compile(plan)
        executor.execute()

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

    statechart.add_node(Sequence([pick_up, place]))

    assert statechart.get_preceding_node_by_type(place, PickUpAction) is pick_up


# %% running on the robot


@dataclass
class GiskardWrapperRecordingTheGoal:
    """
    Stands in for the connection to Giskard, recording what it is asked to execute.
    """

    executed: List[Statechart] = field(default_factory=list)
    """
    Every statechart sent, in order.
    """

    child_choosers: List[ChildChooser] = field(default_factory=list)
    """
    The chooser handed along with every statechart.
    """

    def execute(self, motion_statechart: Statechart, child_chooser: ChildChooser):
        self.executed.append(motion_statechart)
        self.child_choosers.append(child_chooser)


def test_an_underspecified_node_is_sent_as_a_node_choosing_its_child(
    immutable_model_world,
):
    """
    Giskard receives the children the client chooses, so it needs nothing but the node
    itself, and not the statement it is grounded from.
    """
    world, robot, context = immutable_model_world
    node = UnderspecifiedNode(statement=a(NavigateAction)(target_location=...))

    received = from_json(json.loads(json.dumps(to_json(node))))

    assert type(received) is CompositeNodeChoosingItsChild
    assert received.name == node.name


def test_a_plan_on_the_robot_is_sent_once_with_the_chooser_grounding_its_actions(
    immutable_model_world, monkeypatch
):
    world, robot, context = immutable_model_world
    giskard = GiskardWrapperRecordingTheGoal()
    monkeypatch.setattr(Context, "giskard_wrapper", property(lambda self: giskard))
    plan = Sequence(
        [
            MoveTorsoAction(TorsoState.HIGH),
            UnderspecifiedNode(
                statement=a(NavigateAction)(
                    target_location=variable(
                        Pose, domain=[_pose_in_front_of_the_robot(world, robot)]
                    )
                )
            ),
        ]
    )

    with real_robot:
        executor = PlanExecutor(context)
        executor.compile(plan)
        executor.execute()

    [statechart] = giskard.executed
    assert plan.statechart is statechart
    [chooser] = giskard.child_choosers
    assert isinstance(chooser, UnderspecifiedChildChooser)
    assert chooser.context is context


def test_an_expanded_action_is_received_with_the_nodes_it_runs(immutable_model_world):
    """
    A receiver does not expand the nodes of a statechart again, so an action has to
    arrive knowing the sequence it runs.
    """
    world, robot, context = immutable_model_world
    sent = Statechart(context=context.create_statechart_context())
    sent.add_node(action := MoveTorsoAction(TorsoState.HIGH))

    received = Statechart.from_json(
        json.loads(json.dumps(sent.to_json())),
        context=context.create_statechart_context(),
        **WorldEntityWithIDKwargsTracker.from_world(world).create_kwargs(),
    )

    received_action = received.get_node_by_index(action.index)
    assert received_action._action_body is received.get_node_by_index(
        action._action_body.index
    )
