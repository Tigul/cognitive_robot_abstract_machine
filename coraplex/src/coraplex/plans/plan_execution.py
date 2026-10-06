from __future__ import annotations

import logging
from copy import deepcopy
from dataclasses import dataclass, field, replace

from typing_extensions import Dict, Iterator, Optional, Tuple

from coraplex.datastructures.dataclasses import Context
from coraplex.datastructures.enums import ExecutionType
from coraplex.exceptions import (
    MotionDidNotFinish,
    NotAnUnderspecifiedNode,
    UnknownExecutionType,
)
from coraplex.execution_environment import ExecutionEnvironment
from coraplex.plans.designator import DesignatorParameters
from coraplex.plans.failures import EmptyUnderspecified, PlanFailure
from coraplex.plans.underspecified import UnderspecifiedNode
from cramph.candidate_generator import CandidateGenerator
from cramph.composites import (
    Sequence,
    ChildChooser,
    ChildChooserAccess,
    CompositeNodeChoosingItsChild,
)
from cramph.context import StatechartContext
from cramph.data_types import LifeCycleValues
from cramph.executor import StatechartExecutor
from cramph.node import StatechartNode
from cramph.statechart import Statechart
from giskardpy.motion_control import MotionControl
from giskardpy.motion_statechart.goals.collision_avoidance import (
    ExternalCollisionAvoidance,
    SelfCollisionAvoidance,
)
from giskardpy.motion_statechart.graph_node import EndMotion
from giskardpy.motion_statechart.ros_context import RosNodeAccess
from giskardpy.qp.qp_controller_config import QPControllerConfig

logger = logging.getLogger(__name__)


# %% running the statechart of a plan


@dataclass
class PlanExecution:
    """
    Runs a plan as one statechart, in the way the current
    :class:`~coraplex.execution_environment.ExecutionEnvironment` asks for.

    The statechart holds the plan's root, the collision avoidance when the environment
    asks for it, and one :class:`~giskardpy.motion_statechart.graph_node.EndMotion`
    once the root succeeded. Underspecified actions are grounded while it runs, see
    :class:`UnderspecifiedChildChooser`.
    """

    root: StatechartNode
    """
    The node the plan consists of.
    """

    context: Context
    """
    The context the plan is executed in.
    """

    def run(self) -> None:
        """
        Run the plan until its root succeeded.

        :raises MotionDidNotFinish: If the root did not succeed.
        :raises UnknownExecutionType: If no environment says how to execute.
        """
        execution_type = ExecutionEnvironment.current_execution_type
        match execution_type:
            case ExecutionType.NO_EXECUTION:
                return
            case ExecutionType.SIMULATED:
                self._run_in_simulation()
            case ExecutionType.REAL:
                self._run_on_the_robot()
            case _:
                raise UnknownExecutionType(execution_type)

    @staticmethod
    def create_executor(context: Context) -> StatechartExecutor:
        """
        :param context: The plan context whose world and ROS node the executor uses.
        :return: An executor that runs the statechart of a plan in simulation.
        """
        executor = StatechartExecutor(
            context=context.create_statechart_context(),
            extensions=[
                RosNodeAccess(context.ros_node),
                MotionControl(
                    qp_controller_config=QPControllerConfig(
                        target_frequency=50, prediction_horizon=4, verbose=False
                    )
                ),
            ],
        )
        executor.context.add_extension(
            ChildChooserAccess(chooser=UnderspecifiedChildChooser(context=context))
        )
        return executor

    def create_statechart(self, statechart_context: StatechartContext) -> Statechart:
        """
        :param statechart_context: The context the statechart is built in.
        :return: The statechart running the plan, ending once its root succeeded.
        """
        statechart = Statechart(context=statechart_context)
        statechart.add_node(self.root)
        if ExecutionEnvironment.current_collision_avoidance:
            statechart.add_node(ExternalCollisionAvoidance())
            statechart.add_node(SelfCollisionAvoidance())
        statechart.add_node(EndMotion.when_true(self.root))
        return statechart

    def _run_in_simulation(self) -> None:
        """
        Tick the statechart in the world of the context until it ended, its root ended
        without succeeding, or the tick budget ran out.

        :raises MotionDidNotFinish: If the statechart did not end.
        """
        executor = self.create_executor(self.context)
        statechart = self.create_statechart(executor.context)
        try:
            executor.compile(statechart)
            while not self._is_over(statechart) and (
                executor.tick_count < self._tick_budget()
            ):
                executor.tick()
        finally:
            MotionControl.set_velocity_acceleration_jerk_to_zero(executor.context.world)
            statechart.cleanup_nodes()
            executor.context.cleanup()
        if statechart.is_ended():
            return
        self._raise_if_out_of_actions(statechart)
        motion_did_not_finish = MotionDidNotFinish(
            [
                node
                for node in statechart.nodes
                if node.life_cycle_state
                not in [LifeCycleValues.SUCCEEDED, LifeCycleValues.NOT_STARTED]
            ]
        )
        logger.error(motion_did_not_finish.error_message())
        raise motion_did_not_finish

    @staticmethod
    def _raise_if_out_of_actions(statechart: Statechart) -> None:
        """
        :raises EmptyUnderspecified: If an underspecified node ran out of actions to
            try, which is why the statechart did not end.
        """
        for node in statechart.get_nodes_by_type(UnderspecifiedNode):
            if node.ran_out_of_children:
                raise EmptyUnderspecified(node=node)

    def _is_over(self, statechart: Statechart) -> bool:
        """
        :return: Whether the statechart ended or its root can no longer succeed.
        """
        return statechart.is_ended() or self.root.life_cycle_state in (
            LifeCycleValues.FAILED,
            LifeCycleValues.INTERRUPTED,
        )

    def _tick_budget(self) -> int:
        """
        The ticks the run may take,
        :attr:`~coraplex.datastructures.dataclasses.Context.ticks_per_motion` for every
        motion of the plan.

        It grows with the plan, since a grounded action joins it while it runs.
        """
        motion_count = len(
            [
                node
                for node in [self.root, *self.root.descendants]
                if self._is_motion(node)
            ]
        )
        return max(motion_count, 1) * self.context.ticks_per_motion

    @staticmethod
    def _is_motion(node: StatechartNode) -> bool:
        """
        :return: Whether `node` is one step of a sequence that does something itself,
            rather than an action, a sequence or an underspecified node arranging other
            steps. A motion held alongside speed caps or collision rules is one step.
        """
        if not isinstance(node.parent_node, Sequence):
            return False
        return not isinstance(
            node, (DesignatorParameters, Sequence, CompositeNodeChoosingItsChild)
        )

    def _run_on_the_robot(self) -> None:
        """
        Send the statechart to giskard and wait until it ended, grounding every
        underspecified action giskard reaches against the world as it is then.
        """
        statechart_context = self.context.create_statechart_context()
        self.context.giskard_wrapper.execute(
            self.create_statechart(statechart_context),
            child_chooser=UnderspecifiedChildChooser(context=self.context),
        )


# %% trying a grounded action out before it is executed for real


@dataclass
class ActionTrial:
    """
    Tries grounded actions against a disposable copy of the world, to check that a
    candidate can succeed before it is attempted for real.

    One copy serves every candidate: after an attempt the copy is rolled back to the
    model version it was at and its state is restored, so the next candidate starts from
    the same point without another copy having to be made. A fresh copy is taken
    whenever `context.world` has itself moved on, so a trial always reflects the state
    and model changes actually in it.

    The copy is never connected to a synchronizer, so nothing a trial does is published,
    and a trial always runs simulated, whatever the real attempt will use.
    """

    context: Context
    """
    The context the candidates were grounded in.

    Only ever read from: a trial never mutates it or the world it points at, and the
    candidates themselves are left untouched too, so they can still run for real
    afterwards.
    """

    _copied_context: Optional[Context] = field(default=None, init=False, repr=False)
    """
    The context pointing at the copy candidates are tried against, kept until that copy
    no longer matches the world it was taken from.
    """

    _source_versions: Optional[Tuple[int, int]] = field(
        default=None, init=False, repr=False
    )
    """
    The model and state versions `context.world` had when the copy was taken, used to
    notice that it has moved on and the copy has to be replaced.
    """

    def succeeds(self, action: DesignatorParameters) -> bool:
        """
        Run `action` against the copy and restore the copy afterwards.

        The action is rebuilt from its own parameters, rebound onto the copy, because an
        action that modifies the model (attaching a grasped body, say) requires the
        entities it is given to belong to the world being modified, and because a
        statechart node belongs to one statechart only.

        The version to roll back to is read here rather than when the copy is taken, so
        each attempt undoes only its own modifications.

        :param action: The grounded action to try out.
        :return: True if `action` runs to completion without raising a `PlanFailure`.
        """
        context = self._copy()
        world = context.world
        candidate = type(action)(
            **world.rebind_world_entities(action.designator_parameter)
        )
        version = world.get_world_model_manager().version

        with (
            world.reset_state_context(),
            ExecutionEnvironment(
                ExecutionType.SIMULATED,
                collision_avoidance=ExecutionEnvironment.current_collision_avoidance,
            ),
        ):
            try:
                PlanExecution(root=candidate, context=context).run()
                return True
            except PlanFailure as failure:
                logger.info("%s failed its trial: %s", action, failure)
                return False
            finally:
                # Undo the model changes before leaving the reset context restores the
                # state, which needs the degrees of freedom it was snapshotted with.
                world.rollback_to_version(version)

    def _copy(self) -> Context:
        """
        :return: The context pointing at the copy to try candidates against, taken again
            if `context.world` has changed since the current one was made.
        """
        versions = (
            self.context.world.get_world_model_manager().version,
            self.context.world.state.version,
        )
        if self._copied_context is None or self._source_versions != versions:
            world = deepcopy(self.context.world)
            self._copied_context = replace(
                self.context,
                world=world,
                robot=world.get_semantic_annotation_by_id(self.context.robot.id),
            )
            self._source_versions = versions
        return self._copied_context

    def discard(self) -> None:
        """
        Release the copy, so the next trial takes a fresh one.
        """
        self._copied_context = None
        self._source_versions = None


# %% grounding underspecified actions while the statechart runs


@dataclass(eq=False, repr=False)
class UnderspecifiedCandidates(
    CandidateGenerator[DesignatorParameters, StatechartNode]
):
    """
    The actions an :class:`~coraplex.plans.underspecified.UnderspecifiedNode` may run,
    grounded one at a time against the world at the moment it asks, each tried in an
    :class:`ActionTrial` first.
    """

    node: UnderspecifiedNode
    """
    The node the actions are grounded for.
    """

    trial: ActionTrial
    """
    The trial every candidate of :attr:`node` is tried against, shared so they share one
    copy of the world.
    """

    def _generate_proposals(self) -> Iterator[DesignatorParameters]:
        return self.trial.context.query_backend.evaluate(self.node.statement)

    def _is_viable(self, proposal: DesignatorParameters) -> bool:
        """
        A proposal that fails its trial is discarded without ever touching the real
        world, so a bad parameterization cannot poison a later attempt.
        """
        return self.trial.succeeds(proposal)

    def _create_candidate(self, proposal: DesignatorParameters) -> StatechartNode:
        return proposal

    def stop_generating(self) -> None:
        """
        Release the action iterator and the trial's copy of the world.
        """
        super().stop_generating()
        self.trial.discard()


@dataclass
class UnderspecifiedChildChooser(ChildChooser):
    """
    Grounds the statement of every
    :class:`~coraplex.plans.underspecified.UnderspecifiedNode` of a statechart into the
    action it runs next.
    """

    context: Context
    """
    The context the statements are grounded in.
    """

    _candidates: Dict[UnderspecifiedNode, UnderspecifiedCandidates] = field(
        default_factory=dict, init=False, repr=False
    )
    """
    The candidates of every node that asked so far.
    """

    def choose_child(
        self, node: CompositeNodeChoosingItsChild, context: StatechartContext
    ) -> Optional[StatechartNode]:
        """
        :raises NotAnUnderspecifiedNode: If `node` carries no statement to ground.
        """
        if not isinstance(node, UnderspecifiedNode):
            raise NotAnUnderspecifiedNode(node=node)
        candidates = self._candidates_of(node)
        if candidates.advance():
            return candidates.current_candidate
        candidates.stop_generating()
        return None

    def _candidates_of(self, node: UnderspecifiedNode) -> UnderspecifiedCandidates:
        """
        :return: The candidates of `node`, created when it first asks.
        """
        if node not in self._candidates:
            self._candidates[node] = UnderspecifiedCandidates(
                node=node, trial=ActionTrial(context=self.context)
            )
        return self._candidates[node]

    def cleanup(self) -> None:
        """
        Release every node's action iterator and world copy.
        """
        for candidates in self._candidates.values():
            candidates.stop_generating()
