"""
Tests for how a plan expands into the nodes of its statechart.
"""

import numpy as np
import pytest
from typing_extensions import List

from coraplex.datastructures.enums import (
    Arms,
    ApproachDirection,
    DetectionTechnique,
    VerticalAlignment,
)
from coraplex.datastructures.grasp import GraspDescription
from coraplex.exceptions import PerceptionTargetMissing
from coraplex.execution_environment import simulated_robot
from coraplex.perception import PerceptionQuery, PerceptionTask
from coraplex.plans.factories import (
    cancel_when,
    execute_single,
    parallel,
    pause_until,
    pause_while,
    repeat,
    sequential,
    try_all,
    try_in_order,
)
from coraplex.plans.plan import Plan
from coraplex.plans.plan_execution import PlanExecution
from coraplex.robot_plans.actions.composite.transporting import TransportAction
from coraplex.robot_plans.actions.core.misc import DetectAction
from coraplex.robot_plans.actions.core.pick_up import PickUpAction, ReachAction
from coraplex.robot_plans.actions.core.robot_body import MoveTorsoAction, ParkArmsAction
from cramph.composites import (
    CancelledWhenTrue,
    Parallel,
    PausedUntilTrue,
    PausedWhileTrue,
    Sequence,
    TryAll,
    TryInOrder,
)
from cramph.data_types import LifeCycleValues
from cramph.monitors import CountNodeResets
from cramph.node import CancelStatechart, StatechartNode
from cramph.nodes_for_testing import ConstFalseNode
from cramph.statechart import Statechart
from cramph.world_modification_nodes import MoveBranch
from giskardpy.motion_statechart.goals.gripper import MoveGripper
from giskardpy.motion_statechart.goals.templates import RepeatOnStall
from giskardpy.motion_statechart.monitors.progress_monitors import Stalled
from giskardpy.motion_statechart.tasks.cartesian_tasks import (
    CartesianPose,
    CartesianPosition,
)
from giskardpy.motion_statechart.tasks.joint_tasks import JointPositionList
from semantic_digital_twin.datastructures.definitions import TorsoState
from semantic_digital_twin.semantic_annotations.semantic_annotations import Milk
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix
from semantic_digital_twin.spatial_types.spatial_types import Point3, Pose
from semantic_digital_twin.world_description.geometry import VolumetricBoundingBox

from ..conftest import expand, motion_nodes_of, tool_center_point_goal

# %% helpers


def _compile(plan: Plan) -> Statechart:
    """
    Build the statechart that performing `plan` runs, and compile it without ticking.

    Compiling is what validates the scopes of every transition condition.

    :return: The compiled statechart.
    """
    execution = PlanExecution(root=plan.root, context=plan.context)
    executor = execution.create_executor(plan.context)
    with simulated_robot:
        statechart = execution.create_statechart(executor.context)
    statechart.compile()
    return statechart


def _nodes_of_type(root: StatechartNode, node_type: type) -> List[StatechartNode]:
    """
    :return: Every node of `node_type` at or below `root`, in depth first order.
    """
    return [node for node in [root, *root.descendants] if isinstance(node, node_type)]


# %% an action expands into its motions


def test_an_action_expands_into_its_motion(immutable_model_world):
    world, view, context = immutable_model_world

    root = expand(execute_single(MoveTorsoAction(TorsoState.HIGH), context=context))

    assert [type(node) for node in _nodes_of_type(root, JointPositionList)] == [
        JointPositionList
    ]


# %% the factories build the composite of their construct


@pytest.mark.parametrize(
    "factory, composite",
    [
        (sequential, Sequence),
        (parallel, Parallel),
        (try_all, TryAll),
        (try_in_order, TryInOrder),
    ],
)
def test_a_factory_builds_the_composite_of_its_construct(
    immutable_model_world, factory, composite
):
    """
    Each factory makes its steps the children of the composite that gives them their
    sequential, parallel or try semantics.
    """
    world, view, context = immutable_model_world
    steps = [MoveTorsoAction(TorsoState.LOW), MoveTorsoAction(TorsoState.HIGH)]

    plan = factory(steps, context=context)

    assert type(plan.root) is composite
    assert plan.root.nodes == steps


def test_a_sequence_holds_each_action_with_its_own_motion(immutable_model_world):
    world, view, context = immutable_model_world

    root = expand(
        sequential(
            [MoveTorsoAction(TorsoState.LOW), MoveTorsoAction(TorsoState.HIGH)],
            context=context,
        )
    )

    assert [type(step) for step in root.nodes] == [MoveTorsoAction, MoveTorsoAction]
    assert [len(_nodes_of_type(step, JointPositionList)) for step in root.nodes] == [
        1,
        1,
    ]


def test_a_nested_plan_runs_as_a_step_of_the_plan_around_it(immutable_model_world):
    world, view, context = immutable_model_world
    inner = sequential([MoveTorsoAction(TorsoState.HIGH)])

    plan = sequential([MoveTorsoAction(TorsoState.LOW), inner], context=context)

    assert plan.root.nodes[1] is inner.root


# %% monitored subtrees


def test_pause_monitor_pauses_the_children_goal(immutable_model_world, rclpy_node):
    """
    The monitor and the children's goal are siblings inside the monitored goal, which is
    what makes the pause condition legal: it may only reference a sibling.
    """
    world, view, context = immutable_model_world
    monitor = ConstFalseNode(name="never")

    plan = pause_while(
        [MoveTorsoAction(TorsoState.HIGH)], monitor=monitor, context=context
    )
    _compile(plan)

    monitored_goal = plan.root
    assert type(monitored_goal) is PausedWhileTrue
    assert monitored_goal.nodes == [monitor, monitored_goal.monitored_node]
    assert monitored_goal.monitored_node.pause_condition.free_variables() == [
        monitor.observes_true
    ]


def test_pause_until_monitor_pauses_the_children_goal(
    immutable_model_world, rclpy_node
):
    """
    The children's goal is paused on the negated monitor observation, so it is held
    until the monitor turns True rather than while it is True.
    """
    world, view, context = immutable_model_world
    monitor = ConstFalseNode(name="never")

    plan = pause_until(
        [MoveTorsoAction(TorsoState.HIGH)], monitor=monitor, context=context
    )
    _compile(plan)

    monitored_goal = plan.root
    assert type(monitored_goal) is PausedUntilTrue
    assert monitored_goal.nodes == [monitor, monitored_goal.monitored_node]
    assert monitored_goal.monitored_node.pause_condition.free_variables() == [
        monitor.observes_true
    ]


def test_cancel_monitor_ends_the_children_goal(immutable_model_world, rclpy_node):
    world, view, context = immutable_model_world
    monitor = ConstFalseNode(name="never")

    plan = cancel_when(
        [MoveTorsoAction(TorsoState.HIGH)], monitor=monitor, context=context
    )
    _compile(plan)

    monitored_goal = plan.root
    assert type(monitored_goal) is CancelledWhenTrue
    assert monitored_goal.nodes[:2] == [monitor, monitored_goal.monitored_node]
    # The children's goal already ends itself once it succeeds, so the monitor firing is
    # a reason to interrupt it on top of that. It is read through its last observation,
    # which outlasts a monitor that ends itself on firing.
    assert monitor.last_observed_true in (
        monitored_goal.monitored_node.interrupt_condition.free_variables()
    )


def test_cancel_monitor_ends_the_motion_when_the_monitor_fires(
    immutable_model_world, rclpy_node
):
    """
    The monitored goal holds a node that ends the motion, so giving up on the subtree
    gives up on the plan rather than leaving the rest of it waiting.
    """
    world, view, context = immutable_model_world
    monitor = ConstFalseNode(name="never")

    plan = cancel_when(
        [MoveTorsoAction(TorsoState.HIGH)], monitor=monitor, context=context
    )
    _compile(plan)

    monitored_goal = plan.root
    [cancelled] = [
        node for node in monitored_goal.nodes if isinstance(node, CancelStatechart)
    ]
    assert cancelled.exception == monitored_goal.exception
    assert cancelled.start_condition.free_variables() == [monitor.last_observed_true]


def test_monitored_subtree_nested_in_a_sequence_compiles(
    immutable_model_world, rclpy_node
):
    """
    A monitored subtree is a node like any other in the surrounding sequence.

    Compiling is the real assertion: it runs the condition scope validation that this
    structure exists to satisfy.
    """
    world, view, context = immutable_model_world

    plan = sequential(
        [
            MoveTorsoAction(TorsoState.LOW),
            cancel_when(
                [MoveTorsoAction(TorsoState.HIGH)], monitor=ConstFalseNode(name="never")
            ),
        ],
        context=context,
    )
    statechart = _compile(plan)

    assert len(statechart.get_nodes_by_type(CancelledWhenTrue)) == 1


# %% repeating a subtree


def test_repeat_wraps_its_children_in_a_repeating_goal(
    immutable_model_world, rclpy_node
):
    """
    A repeat builds a goal that holds the children, the attempt counter and the node
    that reports running out of attempts, all as siblings so the wiring between them is
    legal.
    """
    world, view, context = immutable_model_world

    plan = repeat(
        [MoveTorsoAction(TorsoState.HIGH)], maximum_repetitions=3, context=context
    )
    _compile(plan)

    loop = plan.root
    assert type(loop) is RepeatOnStall
    assert loop.task in loop.nodes
    [counter] = [node for node in loop.nodes if isinstance(node, CountNodeResets)]
    assert counter.target == 3
    assert counter is loop.stop_retry_monitor
    [exhausted] = [node for node in loop.nodes if isinstance(node, CancelStatechart)]
    assert exhausted.start_condition.free_variables() == [counter.last_observed_true]


def test_repeat_with_failure_monitor_gives_the_stall_template_one_attempt(
    immutable_model_world, rclpy_node
):
    """
    A failure monitor and the default stall template share one attempt around the
    children, which gives up on whichever of the two fires first.
    """
    world, view, context = immutable_model_world
    never = ConstFalseNode(name="never")

    plan = repeat(
        [MoveTorsoAction(TorsoState.HIGH)],
        maximum_repetitions=3,
        failure_monitor=never,
        context=context,
    )
    _compile(plan)

    loop = plan.root
    assert type(loop) is RepeatOnStall
    assert never in loop.task.failure_monitors
    stall_monitors = [
        monitor
        for monitor in loop.task.failure_monitors
        if isinstance(monitor, Stalled)
    ]
    assert len(stall_monitors) == 1


# %% running actions


def test_a_reach_runs_to_its_target(immutable_model_world, rclpy_node):
    world, view, context = immutable_model_world

    milk_connection = world.get_body_by_name("milk.stl").parent_connection
    milk_connection.origin = HomogeneousTransformationMatrix.from_xyz_rpy(
        2, 1.5, 0.7, 0, 0, 0, reference_frame=milk_connection.parent
    )
    reach = ReachAction(
        Pose.from_xyz_rpy(2, 1.5, 0.7, reference_frame=world.root),
        Arms.RIGHT,
        GraspDescription(
            ApproachDirection.FRONT,
            VerticalAlignment.NoAlignment,
            view.right_arm.end_effector,
        ),
        world.get_semantic_annotations_by_type(Milk)[0],
    )

    with simulated_robot:
        execute_single(reach, context=context).perform()

    assert reach.life_cycle_state == LifeCycleValues.SUCCEEDED


def test_a_pick_up_moves_the_object_to_the_gripper_between_closing_and_lifting(
    immutable_model_world,
):
    """
    The object only follows the gripper once it belongs to it, and has to before the
    lift, so the branch moves between the two inside the pick-up's own statechart.
    """
    world, view, context = immutable_model_world

    root = expand(
        execute_single(
            PickUpAction(
                world.get_semantic_annotations_by_type(Milk)[0],
                Arms.RIGHT,
                GraspDescription(
                    ApproachDirection.FRONT,
                    VerticalAlignment.NoAlignment,
                    view.right_arm.end_effector,
                ),
            ),
            context=context,
        )
    )

    steps = [
        type(node)
        for node in _nodes_of_type(
            root, (MoveGripper, MoveBranch, CartesianPose, CartesianPosition)
        )
    ]
    assert steps[-3:] == [MoveGripper, MoveBranch, CartesianPosition]


def test_a_transport_runs_with_its_underspecified_steps(
    mutable_model_world, rclpy_node
):
    world, view, context = mutable_model_world

    plan = sequential(
        [
            MoveTorsoAction(TorsoState.HIGH),
            ParkArmsAction(Arms.BOTH),
            TransportAction(
                world.get_semantic_annotations_by_type(Milk)[0],
                Pose.from_xyz_rpy(2.37, 2.5, 1.05, reference_frame=world.root),
                Arms.RIGHT,
            ),
        ],
        context=context,
    )

    with simulated_robot:
        plan.perform()

    assert plan.root.life_cycle_state == LifeCycleValues.SUCCEEDED


# %% perception


def test_perceiving_runs_between_the_motions_around_it(immutable_model_world):
    """
    Perception is a step like any other, so it runs in the same statechart as the
    motions around it, in the order the plan gives.
    """
    world, view, context = immutable_model_world
    query = PerceptionQuery(
        Milk,
        VolumetricBoundingBox(
            origin=HomogeneousTransformationMatrix(reference_frame=world.root),
            min_x=-10,
            min_y=-10,
            min_z=-10,
            max_x=10,
            max_y=10,
            max_z=10,
        ),
        view,
        world,
    )

    root = expand(
        sequential(
            [
                tool_center_point_goal(context, Arms.LEFT),
                PerceptionTask(query=query, execution_type=None),
                tool_center_point_goal(context, Arms.RIGHT),
            ],
            context=context,
        )
    )

    assert [
        type(node) for node in _nodes_of_type(root, (CartesianPose, PerceptionTask))
    ] == [
        CartesianPose,
        PerceptionTask,
        CartesianPose,
    ]


def test_a_detect_action_expands_into_a_perception_task(immutable_model_world):
    world, view, context = immutable_model_world

    root = expand(
        execute_single(
            DetectAction(DetectionTechnique.TYPES, object_sem_annotation=Milk),
            context=context,
        )
    )

    assert [type(node) for node in _nodes_of_type(root, PerceptionTask)] == [
        PerceptionTask
    ]


# %% perceiving before the grasp


def detect_actions_of(plan: Plan) -> List[DetectAction]:
    """
    :param plan: The plan to search.
    :return: The detections the plan performs, in no particular order.
    """
    return [node for node in motion_nodes_of(plan) if isinstance(node, DetectAction)]


def reach_action(milk: Milk, view, **kwargs) -> ReachAction:
    """
    :param milk: The object the reach is aimed at.
    :param view: The robot reaching for it.
    :param kwargs: The fields under test.
    :return: A reach at the object's own frame.
    """
    return ReachAction(
        target_pose=Pose(reference_frame=milk.root),
        arm=Arms.RIGHT,
        grasp_description=GraspDescription(
            ApproachDirection.FRONT,
            VerticalAlignment.NoAlignment,
            view.right_arm.end_effector,
        ),
        object_designator=milk,
        **kwargs,
    )


def test_a_reach_does_not_perceive_by_default(immutable_model_world):
    """
    A reach acts on the pose the world already holds, so it must not spend a detection
    the caller did not ask for.
    """
    world, view, context = immutable_model_world
    milk = world.get_semantic_annotations_by_type(Milk)[0]

    plan = execute_single(reach_action(milk, view), context=context)

    assert detect_actions_of(plan) == []


def test_perceiving_before_the_grasp_detects_the_object_being_reached_for(
    immutable_model_world,
):
    """
    The detection has to ask for the object the reach was given, so that a plan grasping
    something else does not query for the wrong thing.
    """
    world, view, context = immutable_model_world
    milk = world.get_semantic_annotations_by_type(Milk)[0]

    plan = execute_single(
        reach_action(milk, view, perceive_before_grasp=True), context=context
    )

    [detection] = detect_actions_of(plan)
    assert detection.object_sem_annotation is type(milk)


def test_a_pick_up_passes_perceiving_on_to_its_reach(immutable_model_world):
    """
    The flag is set on the pick-up, but the detection belongs to the reach inside it, so
    it has to survive that hand-over.
    """
    world, view, context = immutable_model_world
    milk = world.get_semantic_annotations_by_type(Milk)[0]

    plan = execute_single(
        PickUpAction(
            milk,
            Arms.RIGHT,
            GraspDescription(
                ApproachDirection.FRONT,
                VerticalAlignment.NoAlignment,
                view.right_arm.end_effector,
            ),
            perceive_before_grasp=True,
        ),
        context=context,
    )

    [detection] = detect_actions_of(plan)
    assert detection.object_sem_annotation is type(milk)


def test_perceiving_without_an_object_to_detect_is_rejected(immutable_model_world):
    """
    A reach may be given a pose without an object, but then there is nothing to build
    the detection query from, so the contradiction is reported instead of guessed away.
    """
    world, view, context = immutable_model_world

    reach = ReachAction(
        target_pose=Pose(reference_frame=world.root),
        arm=Arms.RIGHT,
        grasp_description=GraspDescription(
            ApproachDirection.FRONT,
            VerticalAlignment.NoAlignment,
            view.right_arm.end_effector,
        ),
        perceive_before_grasp=True,
    )

    with pytest.raises(PerceptionTargetMissing):
        expand(execute_single(reach, context=context))


# %% expansion-time pose capture


def test_pick_up_motions_follow_the_object_moved_after_expansion(immutable_model_world):
    """
    A pick-up expands when its plan starts, before the first motion runs, so one that
    captured the object's pose in world coordinates could never act on a pose corrected
    in between (for example by a detection).

    Keeping the motion targets in the object's own frame is what lets them follow it.
    """
    world, view, context = immutable_model_world
    milk = world.get_semantic_annotations_by_type(Milk)[0]
    milk_body = milk.root

    plan = execute_single(
        PickUpAction(
            milk,
            Arms.RIGHT,
            GraspDescription(
                ApproachDirection.FRONT,
                VerticalAlignment.NoAlignment,
                view.right_arm.end_effector,
            ),
        ),
        context=context,
    )
    targets = [
        node.goal_pose
        for node in motion_nodes_of(plan)
        if isinstance(node, CartesianPose)
    ]
    positions_before = [
        world.transform(target, world.root).to_position().to_np().flatten()[:3]
        for target in targets
    ]

    displacement = np.array([0.25, -0.4, 0.1])
    milk_body.parent_connection.origin = HomogeneousTransformationMatrix.from_xyz_rpy(
        *(milk_body.global_pose.to_position().to_np().flatten()[:3] + displacement),
        reference_frame=world.root,
    )

    assert targets
    assert all(target.reference_frame is milk_body for target in targets)
    for target, position_before in zip(targets, positions_before):
        np.testing.assert_allclose(
            world.transform(target, world.root).to_position().to_np().flatten()[:3],
            position_before + displacement,
            atol=1e-9,
        )
