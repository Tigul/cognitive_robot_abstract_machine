from dataclasses import dataclass, field
from uuid import UUID, uuid4

from typing_extensions import Dict, List, Optional

from krrood.entity_query_language.backends import (
    EntityQueryLanguageGenerativeBackend,
    ProbabilisticBackend,
)
from krrood.entity_query_language.factories import a, variable_from
from cramph.composites import Sequence
from cramph.data_types import LifeCycleValues
from cramph.node import StatechartNode

from coraplex.datastructures.enums import (
    Arms,
    ApproachDirection,
    VerticalAlignment,
)
from coraplex.datastructures.grasp import GraspDescription

from coraplex.execution_environment import simulated_robot
from coraplex.plans.failures import PlanFailure
from coraplex.plans.function_call import FunctionCall
from coraplex.robot_plans.actions.base import Action
from coraplex.robot_plans.actions.core.navigation import NavigateAction
from coraplex.robot_plans.actions.core.pick_up import PickUpAction
from semantic_digital_twin.robots.robot_parts import AbstractRobot
from semantic_digital_twin.spatial_types.spatial_types import Pose
from semantic_digital_twin.world import World
from coraplex.plans.plan_execution import PlanExecutor
from coraplex.plans.underspecified import UnderspecifiedNode

# %% mimics for testing candidate trials without depending on real motion physics


@dataclass
class TrialCall:
    """
    One recorded attempt of a `RecordingAction`.
    """

    world: World
    """
    The world the attempt actually ran against, so a test can tell a trial copy
    from the real world by identity.
    """

    position_at_entry: float
    """
    The value of the probed degree of freedom when this attempt started, so a test can
    tell whether a later trial copy reflects an earlier real failure's state.
    """


class TrialProbe:
    """
    Records every attempt of a `RecordingAction` across both trial and real execution.
    """

    def __init__(self):
        self.calls: List[TrialCall] = []


_registered_probes: Dict[UUID, TrialProbe] = {}
"""
`TrialProbe` instances, keyed by the id a `RecordingAction` carries as `probe_key`.

`World.rebind_world_entities` deep-copies every value it does not recognize as a world entity, so a
rebound `RecordingAction` cannot share a probe handed to it directly as a field - the
same isolation that makes a trial safe for real designators. Reaching the probe
out-of-band by an id, whose value survives copying even though its identity need not,
is what lets a test observe a candidate's trial and its later real attempt as one
sequence.
"""


def register_probe() -> UUID:
    """
    Register a fresh `TrialProbe` and return the id a `RecordingAction` should carry to
    record into it.
    """
    key = uuid4()
    _registered_probes[key] = TrialProbe()
    return key


@dataclass(eq=False, repr=False)
class RecordingAction(Action):
    """
    An action whose execution deterministically records itself and can be made to fail
    on a specific attempt, for testing the trial-then-real handling of an underspecified
    node's candidates without depending on real motion physics.
    """

    probe_key: UUID = field(kw_only=True)
    """
    Id of the `TrialProbe` this action's attempts are recorded to.
    """

    dof_id: UUID = field(kw_only=True)
    """
    Id of the degree of freedom this action mutates on every attempt, to make world
    state changes observable.
    """

    fail_on_attempt_number: Optional[int] = field(kw_only=True, default=None)
    """
    Raise a `PlanFailure` once the probe has recorded this many calls; never raise if
    None.
    """

    @property
    def _sub_nodes(self) -> List[StatechartNode]:
        return [FunctionCall(function=self._record_attempt)]

    def _record_attempt(self) -> None:
        """
        Record the world this attempt runs against and mutate the probed degree of
        freedom, failing on the configured attempt.
        """
        world = self.world
        probe = _registered_probes[self.probe_key]
        probe.calls.append(
            TrialCall(
                world=world,
                position_at_entry=world.state[self.dof_id].position,
            )
        )
        world.state[self.dof_id].position = len(probe.calls)
        world.notify_state_change()
        if len(probe.calls) == self.fail_on_attempt_number:
            raise PlanFailure()


def test_underspecified_action(apartment_world_pr2_copy_with_context):
    """
    Test that an underspecified action resolves to a concrete candidate, which runs as
    the node's child.
    """
    world, robot, context = apartment_world_pr2_copy_with_context
    action = a(NavigateAction)(
        target_location=variable_from(
            [
                Pose.from_xyz_quaternion(1, -1, 0, reference_frame=world.root),
                Pose.from_xyz_quaternion(2, -1, 0, reference_frame=world.root),
            ]
        ),
    )

    plan = UnderspecifiedNode(statement=action)
    with simulated_robot:
        executor = PlanExecutor(context)
        executor.compile(plan)
        executor.execute()

    assert plan.life_cycle_state == LifeCycleValues.SUCCEEDED
    assert isinstance(plan.latest_child, NavigateAction)


def test_underspecified_action_with_ellipsis(apartment_world_pr2_copy_with_context):
    """
    Test that an underspecified action resolves when a factory for a spatial type is
    used with ellipsis.
    """
    world, robot, context = apartment_world_pr2_copy_with_context
    context.query_backend = ProbabilisticBackend()
    action = a(NavigateAction)(
        target_location=a(Pose.from_xyz_rpy)(
            x=...,
            y=...,
            z=0.0,
            roll=0.0,
            pitch=0.0,
            yaw=0.0,
            reference_frame=context.robot.root,
        ),
    )

    plan = UnderspecifiedNode(statement=action)
    with simulated_robot:
        executor = PlanExecutor(context)
        executor.compile(plan)
        executor.execute()

    assert plan.life_cycle_state == LifeCycleValues.SUCCEEDED
    assert isinstance(plan.latest_child, NavigateAction)


def test_underspecified_language(apartment_world_pr2_copy_with_context):
    """
    Test that entire plans can be underspecified.
    """
    world, robot, context = apartment_world_pr2_copy_with_context
    grasp_description = GraspDescription(
        ApproachDirection.FRONT,
        VerticalAlignment.NoAlignment,
        robot.left_arm.end_effector,
    )
    plan_generator = a(Sequence)(
        nodes=[
            a(NavigateAction)(
                target_location=(
                    target_locations := variable_from(
                        [
                            Pose.from_xyz_quaternion(
                                1, 0, 0, reference_frame=world.root
                            ),
                            Pose.from_xyz_quaternion(
                                2, 0, 0, reference_frame=world.root
                            ),
                        ]
                    )
                ),
            ),
            a(PickUpAction)(
                arm=...,
                grasp_description=grasp_description,
                object_designator=world.get_body_by_name("milk.stl"),
            ),
        ],
    )
    plans = list(EntityQueryLanguageGenerativeBackend().evaluate(plan_generator))
    assert len(plans) == len(list(target_locations._domain_)) * len(list(Arms))


# %% candidate trials


def test_isolation_rejected_candidate_never_touches_real_world(
    apartment_world_pr2_copy_with_context,
):
    """
    A candidate that only ever fails must be rejected during its trial, against a
    disposable copy of the world, and never attached to the plan or executed against the
    real world; only the candidate that survives its trial is executed for real.
    """
    world, robot, context = apartment_world_pr2_copy_with_context
    dof = world.degrees_of_freedom[0]
    probe_key = register_probe()

    action = a(RecordingAction)(
        probe_key=probe_key,
        dof_id=dof.id,
        fail_on_attempt_number=variable_from([1, None]),
    )
    plan = UnderspecifiedNode(statement=action)
    with simulated_robot:
        executor = PlanExecutor(context)
        executor.compile(plan)
        executor.execute()

    assert plan.life_cycle_state == LifeCycleValues.SUCCEEDED
    assert len(plan.children) == 1
    assert plan.children[0].fail_on_attempt_number is None

    probe = _registered_probes[probe_key]
    assert len(probe.calls) == 3
    # candidate 1's trial: the only attempt it ever gets, and it is a copy.
    assert probe.calls[0].world is not world
    # candidate 2's trial: still a copy, restored to how candidate 1 found it.
    assert probe.calls[1].world is not world
    assert probe.calls[1].position_at_entry == probe.calls[0].position_at_entry
    # candidate 2's real attempt: the actual world.
    assert probe.calls[2].world is world

    # candidate 1's rejected trial only ever mutated its own throwaway copy.
    assert world.state[dof.id].position == 3


def test_rejected_candidates_are_tried_against_one_copy(
    apartment_world_pr2_copy_with_context,
):
    """
    Candidates that follow a rejected one are tried against the copy that rejection was
    made in, rather than each candidate copying the world again.

    Nothing has changed the real world between them, so the copy still describes it and
    rolling it back is enough to give the next candidate the same starting point.
    """
    world, robot, context = apartment_world_pr2_copy_with_context
    dof = world.degrees_of_freedom[0]
    probe_key = register_probe()

    action = a(RecordingAction)(
        probe_key=probe_key,
        dof_id=dof.id,
        fail_on_attempt_number=variable_from([1, 2, None]),
    )
    plan = UnderspecifiedNode(statement=action)
    with simulated_robot:
        executor = PlanExecutor(context)
        executor.compile(plan)
        executor.execute()

    probe = _registered_probes[probe_key]
    # every call but the last is a trial; the last is the accepted candidate's real
    # attempt, which runs against the real world.
    trials = probe.calls[:-1]
    assert len(trials) == 3
    assert trials[0].world is not world
    assert trials[1].world is trials[0].world
    assert trials[2].world is trials[0].world


def test_real_failure_keeps_state_and_next_trial_reflects_it(
    apartment_world_pr2_copy_with_context,
):
    """
    A candidate that passes its trial but then fails for real must leave the real
    world's state exactly as the failed attempt left it; the next candidate's trial copy
    must be taken from that post-failure state, not the original one.
    """
    world, robot, context = apartment_world_pr2_copy_with_context
    dof = world.degrees_of_freedom[0]
    initial_position = world.state[dof.id].position
    probe_key = register_probe()

    action = a(RecordingAction)(
        probe_key=probe_key,
        dof_id=dof.id,
        fail_on_attempt_number=variable_from([2, None]),
    )
    plan = UnderspecifiedNode(statement=action)
    with simulated_robot:
        executor = PlanExecutor(context)
        executor.compile(plan)
        executor.execute()

    assert plan.life_cycle_state == LifeCycleValues.SUCCEEDED
    # Both the failed and the accepted candidate are attached to the tree - a real
    # failure is not undone, only worked around by trying the next candidate.
    assert [child.fail_on_attempt_number for child in plan.children] == [
        2,
        None,
    ]

    probe = _registered_probes[probe_key]
    assert len(probe.calls) == 4
    # candidate 1's trial: a copy, starting from the untouched world.
    assert probe.calls[0].world is not world
    assert probe.calls[0].position_at_entry == initial_position
    # candidate 1's real attempt: the actual world, still untouched by the trial,
    # mutated and then failed.
    assert probe.calls[1].world is world
    assert probe.calls[1].position_at_entry == initial_position
    # candidate 2's trial: a fresh copy, taken *after* candidate 1's real failure -
    # it must already carry that mutation.
    assert probe.calls[2].world is not world
    assert probe.calls[2].position_at_entry == 2
    # candidate 2's real attempt: the actual world, still carrying candidate 1's
    # failed-attempt mutation, since nothing rolled it back.
    assert probe.calls[3].world is world
    assert probe.calls[3].position_at_entry == 2

    assert world.state[dof.id].position == 4
