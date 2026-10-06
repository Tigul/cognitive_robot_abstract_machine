from __future__ import annotations

import logging
from copy import deepcopy
from abc import ABC, abstractmethod
from dataclasses import dataclass, field, replace
from datetime import timedelta

from typing_extensions import ClassVar, Dict, Iterator, List, Optional, Tuple

from coraplex.datastructures.dataclasses import Context
from coraplex.datastructures.enums import ActionTrialVisualization, ExecutionType
from coraplex.exceptions import (
    MotionDidNotFinish,
    NotAnUnderspecifiedNode,
    PlanNotCompiled,
    UnknownExecutionType,
)
from coraplex.execution_environment import ExecutionEnvironment
from coraplex.plans.designator import DesignatorParameters
from coraplex.plans.failures import (
    CandidateLimitReached,
    EmptyUnderspecified,
    MotionExceededSimulationTimeLimit,
    MotionMadeNoProgress,
    MotionViolatedCollisionAvoidance,
    PlanFailure,
)
from coraplex.plans.plan_callbacks import PlanCallback, PlanCallbackDispatcher
from coraplex.plans.plan_transformation import PlanRewriting
from coraplex.plans.underspecified import UnderspecifiedNode
from coraplex.robot_plans.actions.base import Action
from coraplex.visualization import RvizVisualization
from cramph.candidate_generator import CandidateGenerator
from cramph.composites import (
    Attempt,
    ChildChooser,
    Sequence,
    ChildChooserAccess,
    CompositeNodeChoosingItsChild,
)
from cramph.context import StatechartContext
from cramph.data_types import LifeCycleValues
from cramph.executor import StatechartExecutor
from cramph.node import StatechartNode
from cramph.statechart import Statechart
from giskardpy.motion_control import MotionControl
from giskardpy.motion_statechart.exceptions import (
    CollisionViolatedError,
    NoProgressError,
)
from giskardpy.motion_statechart.goals.collision_avoidance import (
    ExternalCollisionAvoidance,
    SelfCollisionAvoidance,
)
from giskardpy.motion_statechart.graph_node import ConvergingTask, EndMotion
from giskardpy.motion_statechart.monitors.progress_monitors import (
    Stalled,
    StillProgressing,
)
from giskardpy.motion_statechart.ros_context import RosNodeAccess
from giskardpy.qp.qp_controller_config import QPControllerConfig
from semantic_digital_twin.world import World

logger = logging.getLogger(__name__)


# %% compiling and executing a plan


@dataclass
class PlanExecutor:
    """
    Compiles a plan into one statechart and executes it, in the way the
    :class:`~coraplex.execution_environment.ExecutionEnvironment` in force at
    :meth:`compile` asks for.

    A plan is any statechart node, usually a cramph composite holding actions. The
    statechart holds the plan, the collision avoidance when the environment asks for it,
    and one :class:`~giskardpy.motion_statechart.graph_node.EndMotion` once the plan
    succeeded. Underspecified actions are grounded while it runs, see
    :class:`UnderspecifiedChildChooser`.
    """

    context: Context
    """
    The context plans are compiled and executed in.
    """

    callbacks: List[PlanCallback] = field(default_factory=list, kw_only=True)
    """
    The callbacks observing every plan this executor runs.
    """

    _run: Optional[PlanRun] = field(default=None, init=False, repr=False)
    """
    The run of the plan compiled last.
    """

    def compile(self, plan: StatechartNode) -> None:
        """
        Build the statechart running `plan` and compile it, so that :meth:`execute` can
        run it.

        :param plan: The node to execute.
        :raises UnknownExecutionType: If no environment says how to execute.
        """
        self._run = PlanRun.for_execution_type(
            ExecutionEnvironment.current_execution_type,
            plan=plan,
            context=self.context,
        )
        self._run.compile(self.callbacks)

    def execute(self) -> None:
        """
        Run the compiled plan until it succeeded.

        :raises PlanNotCompiled: If no plan was compiled.
        :raises MotionDidNotFinish: If the plan did not succeed.
        :raises EmptyUnderspecified: If an underspecified action ran out of actions to
            try.
        :raises MotionMadeNoProgress: When the plan stops approaching its goal.
        :raises MotionExceededSimulationTimeLimit: When a simulated plan runs for longer
            than :attr:`SimulatedPlanRun.simulation_time_limit`.
        :raises MotionViolatedCollisionAvoidance: When the plan brings bodies closer to
            each other than collision avoidance allows.
        """
        if self._run is None:
            raise PlanNotCompiled()
        try:
            self._run.execute()
        except NoProgressError as stalled:
            raise MotionMadeNoProgress(stalled) from stalled
        except CollisionViolatedError as violation:
            raise MotionViolatedCollisionAvoidance(violation) from violation


@dataclass
class PlanRun(ABC):
    """
    One plan, compiled and executed in one way.
    """

    plan: StatechartNode
    """
    The node to execute.
    """

    context: Context
    """
    The context the plan is executed in.
    """

    @staticmethod
    def for_execution_type(
        execution_type: ExecutionType, plan: StatechartNode, context: Context
    ) -> PlanRun:
        """
        :param execution_type: How the plan is to be executed.
        :param plan: The node to execute.
        :param context: The context the plan is executed in.
        :return: The run executing `plan` that way.
        :raises UnknownExecutionType: If `execution_type` has no run.
        """
        match execution_type:
            case ExecutionType.NO_EXECUTION:
                return SkippedPlanRun(plan=plan, context=context)
            case ExecutionType.SIMULATED:
                return SimulatedPlanRun(plan=plan, context=context)
            case ExecutionType.REAL:
                return RobotPlanRun(plan=plan, context=context)
            case _:
                raise UnknownExecutionType(execution_type)

    def create_statechart(self, statechart_context: StatechartContext) -> Statechart:
        """
        :param statechart_context: The context the statechart is built in.
        :return: The statechart running the plan, ending once the plan succeeded and
            giving up on it once it stops approaching its goal.
        """
        statechart = Statechart(context=statechart_context)
        statechart.add_node(self.plan)
        self._rewrite(statechart_context)
        if ExecutionEnvironment.current_collision_avoidance:
            statechart.add_node(ExternalCollisionAvoidance())
            statechart.add_node(SelfCollisionAvoidance())
        statechart.add_node(EndMotion.when_true(self.plan))
        self._add_stall_detection(statechart)
        return statechart

    def _rewrite(self, statechart_context: StatechartContext) -> None:
        """
        Rewrite the plan with the plan transformations of the context, and keep them in
        `statechart_context` for the actions that join the plan while it runs.

        :param statechart_context: The context of the statechart the plan was added to.
        """
        rewriting = PlanRewriting(
            transformations=self.context.plan_transformations,
            offered_types=(Action, UnderspecifiedNode),
        )
        statechart_context.add_extension(rewriting)
        rewriting.rewrite(self.plan)

    def _add_stall_detection(self, statechart: Statechart) -> None:
        """
        Cancel the statechart once nothing the plan holds approaches its goal.

        Only the motions the plan holds when it is compiled are watched; an action an
        underspecified node chooses later is watched by its own stall monitor, see
        :class:`UnderspecifiedChildChooser`. A plan holding no motion yet is not watched
        at all, since it would read as stalled from its first tick.

        :param statechart: The statechart the plan was added to, which expanded it.
        """
        if not any(
            isinstance(node, ConvergingTask)
            for node in [self.plan, *self.plan.descendants]
        ):
            return
        still_progressing = StillProgressing(monitored_node=self.plan)
        statechart.add_node(still_progressing)
        statechart.add_node(still_progressing.cancel_motion())

    @abstractmethod
    def compile(self, callbacks: List[PlanCallback]) -> None:
        """
        Build the statechart running the plan and prepare it for :meth:`execute`.

        :param callbacks: The callbacks observing the plan while it runs.
        """

    @abstractmethod
    def execute(self) -> None:
        """
        Run the plan until it succeeded.
        """


@dataclass
class SkippedPlanRun(PlanRun):
    """
    Builds the statechart running the plan without running it, for an environment that
    asks for nothing to be executed, so the plan can be inspected as it would run.
    """

    statechart: Optional[Statechart] = field(default=None, init=False)
    """
    The statechart running the plan, set by :meth:`compile`.
    """

    def compile(self, callbacks: List[PlanCallback]) -> None:
        """
        Build the statechart, which expands the plan, without compiling it.
        """
        self.statechart = self.create_statechart(
            self.context.create_statechart_context()
        )

    def execute(self) -> None:
        """
        Run nothing.
        """


@dataclass
class SimulatedPlanRun(PlanRun):
    """
    Ticks the plan's statechart in the world of the context.
    """

    simulation_time_limit: ClassVar[timedelta] = timedelta(minutes=2)
    """
    The simulated time after which a plan is given up on, however it is progressing.
    """

    qp_controller_config: QPControllerConfig = field(
        default_factory=lambda: QPControllerConfig(
            target_frequency=50, prediction_horizon=4, verbose=False
        ),
        kw_only=True,
    )
    """
    The controller configuration the plan's motions are simulated with.
    """

    executor: StatechartExecutor = field(init=False)
    """
    The executor ticking the statechart.
    """

    child_chooser: UnderspecifiedChildChooser = field(init=False)
    """
    Grounds the underspecified actions of the plan while it runs.
    """

    def __post_init__(self):
        self.child_chooser = UnderspecifiedChildChooser(context=self.context)
        self.executor = StatechartExecutor(
            context=self.context.create_statechart_context(),
            extensions=[
                RosNodeAccess(self.context.ros_node),
                MotionControl(qp_controller_config=self.qp_controller_config),
            ],
        )
        self.executor.context.add_extension(
            ChildChooserAccess(chooser=self.child_chooser)
        )

    def compile(self, callbacks: List[PlanCallback]) -> None:
        """
        Compile the statechart against the executor, which ticks it once, reporting the
        plan's progress to `callbacks` from then on.
        """
        statechart = self.create_statechart(self.executor.context)
        for callback in callbacks:
            callback.on_compile(self.plan, statechart)
        statechart.history.add_observer(
            PlanCallbackDispatcher(plan=self.plan, callbacks=callbacks)
        )
        self.executor.compile(statechart)

    def execute(self) -> None:
        """
        Tick the statechart until it or the plan ended.

        The statechart's own stall monitor decides when a plan is hopeless, so a plan
        that keeps converging is never cut off for taking many ticks.

        :raises MotionDidNotFinish: If the statechart did not end.
        :raises NoProgressError: When the plan stops approaching its goal.
        :raises MotionExceededSimulationTimeLimit: When the plan runs for longer than
            :attr:`simulation_time_limit`.
        """
        statechart = self.executor.statechart
        maximum_ticks = (
            self.simulation_time_limit.total_seconds()
            / self.qp_controller_config.control_dt
        )
        try:
            while not self._is_over(statechart):
                if self.executor.tick_count >= maximum_ticks:
                    raise MotionExceededSimulationTimeLimit(self.simulation_time_limit)
                self.executor.tick()
        finally:
            MotionControl.set_velocity_acceleration_jerk_to_zero(
                self.executor.context.world
            )
            statechart.cleanup_nodes()
            self.executor.context.cleanup()
        if (
            statechart.is_ended()
            or self.plan.life_cycle_state == LifeCycleValues.SUCCEEDED
        ):
            return
        self._raise_if_out_of_candidates(statechart)
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

    def _raise_if_out_of_candidates(self, statechart: Statechart) -> None:
        """
        :raises CandidateLimitReached: If an underspecified node tried as many actions
            as it may, which is why the statechart did not end.
        :raises EmptyUnderspecified: If an underspecified node ran out of actions to
            try, which is why the statechart did not end.
        """
        for node in statechart.get_nodes_by_type(UnderspecifiedNode):
            if not node.ran_out_of_children:
                continue
            candidates = self.child_chooser.candidates_of(node)
            if candidates.reached_candidate_limit:
                raise CandidateLimitReached(
                    node=node, candidate_limit=candidates.candidate_limit
                )
            raise EmptyUnderspecified(node=node)

    def _is_over(self, statechart: Statechart) -> bool:
        """
        :return: Whether the statechart or the plan ended.
        """
        return statechart.is_ended() or self.plan.life_cycle_state.is_terminal


@dataclass
class RobotPlanRun(PlanRun):
    """
    Sends the plan's statechart to giskard, grounding every underspecified action
    giskard reaches against the world as it is then.
    """

    statechart: Optional[Statechart] = field(default=None, init=False)
    """
    The statechart sent to giskard, set by :meth:`compile`.
    """

    def compile(self, callbacks: List[PlanCallback]) -> None:
        """
        Build the statechart; giskard compiles it once it receives it, and ticks it
        where no callback can observe it.
        """
        self.statechart = self.create_statechart(
            self.context.create_statechart_context()
        )

    def execute(self) -> None:
        """
        Send the statechart to giskard and wait until it ended.
        """
        self.context.giskard_wrapper.execute(
            self.statechart,
            child_chooser=UnderspecifiedChildChooser(context=self.context),
        )


# %% trying a grounded action out before it is executed for real


@dataclass
class ActionTrial:
    """
    Tries grounded actions against a copy of the world, to check that a candidate can
    succeed before it is attempted for real.

    One copy serves every candidate: after each attempt its model is rolled back and its
    state restored, and when `context.world` has changed since, the copy replays those
    model and state changes instead of being taken anew. Collision rules changed after
    the copy was taken are not carried over.

    Trials never publish to a synchronizer, always run simulated, and always evaluate
    pre- and postconditions. While the context is debugging, the copy is shown in RViz
    under its own frame prefix and marker topic.
    """

    context: Context
    """
    The context the candidates were grounded in.

    Only ever read from: a trial never mutates it or the world it points at, and the
    candidates themselves are left untouched too, so they can still run for real
    afterwards.
    """

    copy_marker_alpha: float = field(default=0.9, kw_only=True)
    """
    The opacity the copy is drawn with while debugging, so it can be told apart from the
    world it copies where the two overlap.
    """

    _copied_context: Optional[Context] = field(default=None, init=False, repr=False)
    """
    The context pointing at the copy candidates are tried against.
    """

    _source_versions: Optional[Tuple[int, int]] = field(
        default=None, init=False, repr=False
    )
    """
    The model and state versions `context.world` had when the copy last matched it, used
    to notice that it has moved on and the copy has to be caught up.
    """

    _replayed_modification_blocks: int = field(default=0, init=False, repr=False)
    """
    How many of the modification blocks of `context.world` the copy already holds.
    """

    _visualization: Optional[RvizVisualization] = field(
        default=None, init=False, repr=False
    )
    """
    The RViz publishing of the current copy, while the context is debugging.
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
        candidate = self._on_the_copy(action, world)
        version = world.get_world_model_manager().version

        with (
            world.reset_state_context(),
            ExecutionEnvironment(
                ExecutionType.SIMULATED,
                collision_avoidance=ExecutionEnvironment.current_collision_avoidance,
            ),
        ):
            try:
                executor = PlanExecutor(context)
                # The candidate runs in a sequence of its own, the way it runs for real,
                # so the nodes a plan transformation puts beside it are tried with it.
                executor.compile(Sequence(nodes=[candidate]))
                executor.execute()
                return True
            except PlanFailure as failure:
                logger.info(f"{action} failed its trial: {failure}")
                return False
            finally:
                # Undo the model changes before leaving the reset context restores the
                # state, which needs the degrees of freedom it was snapshotted with.
                world.rollback_to_version(version)

    @classmethod
    def _on_the_copy(
        cls, action: DesignatorParameters, world: World
    ) -> DesignatorParameters:
        """
        :param action: The grounded action to try out.
        :param world: The copy to try it against.
        :return: A new action with the parameters of `action`, referring to `world`.
            The actions among those parameters are built anew the same way, since each
            of them becomes a node of the statechart the trial runs.
        """
        return type(action)(
            **{
                name: (
                    cls._on_the_copy(value, world)
                    if isinstance(value, DesignatorParameters)
                    else world.rebind_world_entities(value)
                )
                for name, value in action.designator_parameter.items()
            }
        )

    def _copy(self) -> Context:
        """
        :return: The context pointing at the copy to try candidates against, caught up
            with `context.world` if that has changed since the copy last matched it.
        """
        versions = (
            self.context.world.get_world_model_manager().version,
            self.context.world.state.version,
        )
        if self._copied_context is None:
            self._take_copy()
        elif self._source_versions != versions:
            self._catch_up()
        self._source_versions = versions
        return self._copied_context

    def _take_copy(self) -> None:
        """
        Copy `context.world` and, while the context is debugging, start publishing the
        copy.
        """
        world = deepcopy(self.context.world)
        self._replayed_modification_blocks = len(
            self.context.world.get_world_model_manager().model_modification_blocks
        )
        self._copied_context = replace(
            self.context,
            world=world,
            robot=world.get_semantic_annotation_by_id(self.context.robot.id),
            evaluate_conditions=True,
        )
        if self.context.debug:
            self._visualization = RvizVisualization(
                world,
                ros_node=self.context.ros_node,
                collision_visualization=True,
                frame_prefix=ActionTrialVisualization.FRAME_PREFIX,
                marker_topic=ActionTrialVisualization.MARKER_TOPIC,
                marker_alpha=self.copy_marker_alpha,
            ).start()

    def _catch_up(self) -> None:
        """
        Bring the copy up to date with `context.world`: replay the modifications made to
        it since, the way copying it replays all of them, and take over its state.

        The copy's own modifications are all rolled back by then, so it still matches
        the world as it was when it last caught up.
        """
        modification_blocks = (
            self.context.world.get_world_model_manager().model_modification_blocks
        )
        world = self._copied_context.world
        with world.modify_world():
            for block in modification_blocks[self._replayed_modification_blocks :]:
                block.update_references_for_world_and_apply(world=world)
            world.state.merge_state(self.context.world.state)
        self._replayed_modification_blocks = len(modification_blocks)

    def discard(self) -> None:
        """
        Release the copy, so the next trial takes a fresh one.
        """
        self._stop_visualization()
        self._copied_context = None
        self._source_versions = None

    def _stop_visualization(self) -> None:
        """
        Stop publishing the current copy, if it is being published.
        """
        if self._visualization is None:
            return
        self._visualization.stop()
        self._visualization = None


# %% grounding underspecified actions while the statechart runs


@dataclass(eq=False, repr=False)
class UnderspecifiedCandidates(
    CandidateGenerator[DesignatorParameters, StatechartNode]
):
    """
    The actions an :class:`~coraplex.plans.underspecified.UnderspecifiedNode` may run,
    grounded one at a time against the world at the moment it asks, each tried in an
    :class:`ActionTrial` first.

    At most :attr:`candidate_limit` actions are grounded.
    """

    node: UnderspecifiedNode
    """
    The node the actions are grounded for.
    """

    trial: ActionTrial
    """
    The trial every candidate is tried against, shared by every underspecified node of a
    plan so they all try their candidates in one copy of the world.
    """

    _candidates_pulled: int = field(default=0, init=False, repr=False)
    """
    How many actions the current run through the statement has grounded.
    """

    @property
    def candidate_limit(self) -> int:
        """
        :return: How many actions are grounded: the statement's own limit, or the
            context's if it has none.
        """
        return self.node.statement._limit_ or self.trial.context.candidates_to_try

    @property
    def reached_candidate_limit(self) -> bool:
        """
        :return: Whether the last run through the statement stopped because it grounded
            :attr:`candidate_limit` actions.
        """
        return self._candidates_pulled == self.candidate_limit

    def _pull_next_proposal(self) -> Optional[DesignatorParameters]:
        """
        :return: The next grounded action, or None once the statement is exhausted or
            :attr:`candidate_limit` actions were grounded.
        """
        if self._proposals is None:
            self._candidates_pulled = 0
        if self.reached_candidate_limit:
            self.stop_generating()
            return None
        proposal = super()._pull_next_proposal()
        if proposal is not None:
            self._candidates_pulled += 1
        return proposal

    def _generate_proposals(self) -> Iterator[DesignatorParameters]:
        return self.trial.context.query_backend.evaluate(self.node.statement)

    def _is_viable(self, proposal: DesignatorParameters) -> bool:
        """
        A proposal that fails its trial is discarded without ever touching the real
        world, so a bad parameterization cannot poison a later attempt.
        """
        return self.trial.succeeds(proposal)

    def _create_candidate(self, proposal: DesignatorParameters) -> StatechartNode:
        """
        :return: `proposal`, in a sequence of its own for the nodes a plan
            transformation puts beside it, given up on once it stops approaching its
            goal, so the node can try the next action instead.
        """
        steps = Sequence(name=f"{proposal.name}/steps", nodes=[proposal])
        return Attempt(
            name=f"{proposal.name}/attempt",
            task=steps,
            failure_monitors=[Stalled(monitored_node=steps)],
        )


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

    trial: ActionTrial = field(init=False)
    """
    The trial every node tries its candidates against.
    """

    _candidates: Dict[UnderspecifiedNode, UnderspecifiedCandidates] = field(
        default_factory=dict, init=False, repr=False
    )
    """
    The candidates of every node that asked so far.
    """

    def __post_init__(self):
        self.trial = ActionTrial(context=self.context)

    def choose_child(
        self, node: CompositeNodeChoosingItsChild, context: StatechartContext
    ) -> Optional[StatechartNode]:
        """
        :raises NotAnUnderspecifiedNode: If `node` carries no statement to ground.
        """
        if not isinstance(node, UnderspecifiedNode):
            raise NotAnUnderspecifiedNode(node=node)
        candidates = self.candidates_of(node)
        if candidates.advance():
            return candidates.current_candidate
        candidates.stop_generating()
        return None

    def candidates_of(self, node: UnderspecifiedNode) -> UnderspecifiedCandidates:
        """
        :return: The candidates of `node`, created when it first asks.
        """
        if node not in self._candidates:
            self._candidates[node] = UnderspecifiedCandidates(
                node=node, trial=self.trial
            )
        return self._candidates[node]

    def cleanup(self) -> None:
        """
        Release every node's action iterator and the trial's world copy.
        """
        for candidates in self._candidates.values():
            candidates.stop_generating()
        self.trial.discard()
