from __future__ import annotations

import logging
from abc import ABC
from copy import deepcopy
from dataclasses import dataclass, field, fields

from typing_extensions import (
    TYPE_CHECKING,
    Any,
    Callable,
)

from coraplex.datastructures.enums import ActionTrialVisualization
from coraplex.plans.context_extensions import StatementGrounding
from coraplex.plans.plan_transformation import PlanRewriting, PlanTransformation
from coraplex.robot_plans.actions.base import Action
from coraplex.robot_plans.plan_transformations import WriteTheOdometryInsteadOfDriving
from coraplex.robot_plans.robot_specific_plan_transformations import (
    KeepStretchInFrontOfTheContainerItCloses,
)
from cramph.visualization import RvizVisualization
from cramph.context import StatechartContext
from cramph.composites import (
    ChildChooser,
    ChildChooserAccess,
    CompositeNodeChoosingItsChild,
    ControlStructureNode,
    ControlStructureNodeRunningItsChildrenInTurn,
    GenericChoosingNode,
    Parallel,
    Sequence,
    StatementNode,
    UnderspecifiedParallel,
)
from cramph.executor import ExecutorExtension
from cramph.exceptions import ExecutionFailure
from cramph.node import StatechartNode
from cramph.statechart import Statechart
from giskardpy.motion_statechart.monitors.progress_monitors import Stalled

if TYPE_CHECKING:
    from coraplex.plans.executors import PlanExecutor
    from semantic_digital_twin.world import World

logger = logging.getLogger(__name__)

# %% preparing a chosen child


@dataclass
class PlanChildChooser(ChildChooser[GenericChoosingNode], ABC):
    """
    Chooses children that are part of a plan: each is rewritten by the plan
    transformations of its statechart, and given up on once it stops approaching its
    goal, so the node that chose it can choose the next one.
    """

    def prepare_chosen_child(
        self,
        node: GenericChoosingNode,
        child: StatechartNode,
    ) -> None:
        """
        Rewrites `child` with the plan transformations of `context`, before the monitor
        watching it for a stall collects what it watches, see
        :meth:`~cramph.composites.ChildChooser.prepare_chosen_child`.
        """
        rewriting = node.context.get_extension(PlanRewriting)
        if rewriting is not None:
            rewriting.rewrite(child)

    def failure_monitors_for(
        self,
        node: GenericChoosingNode,
        child: StatechartNode,
    ) -> list[StatechartNode]:
        """
        Differs from :meth:`~cramph.composites.ChildChooser.failure_monitors_for` in
        giving up on `child` once nothing below it approaches its goal.

        :return: A monitor firing once nothing below `child` approaches its goal.
        """
        return [Stalled(monitored_node=child)]

    @staticmethod
    def steps_of(candidate: Action) -> Sequence:
        """
        :param candidate: A grounded action.
        :return: The child running `candidate`: a sequence of its own, for the nodes a
            plan transformation puts beside it.
        """
        return Sequence(name=f"{candidate.name}/steps", nodes=[candidate])


# %% handing a statement a candidate grounded beforehand


@dataclass
class CandidateHandOver(PlanChildChooser[StatementNode]):
    """
    Hands a statement the candidate it was grounded to beforehand, prepared and watched
    for a stall like a candidate the statement grounds itself.

    The statement counts as grounded then, so the plan transformations rewriting
    statements leave it alone.
    """

    candidate: Action
    """
    The candidate the statement runs.
    """

    def choose_child(self, node: StatementNode) -> StatechartNode | None:
        return self.steps_of(self.candidate)

    def prepare_chosen_child(self, node: StatementNode, child: StatechartNode) -> None:
        """
        Differs from :meth:`PlanChildChooser.prepare_chosen_child` in also keeping the
        plan transformations from rewriting `node`.
        """
        super().prepare_chosen_child(node, child)
        rewriting = node.context.get_extension(PlanRewriting)
        if rewriting is not None:
            rewriting.treat_as_offered(node)


@dataclass
class NoFurtherCandidate(ChildChooser[StatementNode]):
    """
    Gives a statement no candidate beyond the one it was handed, so a trial in which a
    candidate fails fails, rather than grounding the statement anew.
    """

    def choose_child(self, node: StatementNode) -> StatechartNode | None:
        return None


# %% rebuilding a parallel onto a world


@dataclass
class BuiltParallel:
    """
    A parallel rebuilt onto a world, and the candidate each of its statements that was
    grounded beforehand runs.
    """

    parallel: Parallel
    """
    The rebuilt parallel.
    """

    candidates: dict[StatementNode, Action]
    """
    The candidate each statement of :attr:`parallel` that was grounded beforehand runs.
    """

    def hand_over_candidates(self) -> None:
        """
        Let every statement of :attr:`candidates` run its candidate, once
        :attr:`parallel` joined a statechart.
        """
        for statement, candidate in self.candidates.items():
            statement.choose_child_with(CandidateHandOver(candidate=candidate))


@dataclass
class ParallelRebuild:
    """
    Rebuilds a parallel of a plan onto a world, putting candidates in for its
    statements.

    Every node is built anew from the fields it is constructed with, an action from its
    parameters, see :meth:`~coraplex.robot_plans.actions.base.Action.rebound_to`, a
    statement as a statement of its own, running the candidate it is handed, if any,
    and any other node choosing its child as the child it runs now.
    """

    parallel: Parallel
    """
    The parallel rebuilt.
    """

    candidates: dict[StatementNode, Action]
    """
    The candidate each statement of :attr:`parallel` runs once rebuilt; a statement
    without one grounds one itself.
    """

    left_out: set[StatechartNode] = field(default_factory=set)
    """
    The nodes of :attr:`parallel` left out of the rebuilt parallel, with everything
    below them.
    """

    _world: World | None = field(default=None, init=False, repr=False)
    """
    The world the current :meth:`build` rebuilds the nodes onto.
    """

    _cuts_branches: bool = field(default=False, init=False, repr=False)
    """
    Whether the current :meth:`build` ends a branch before its first statement without a
    candidate.
    """

    _handed_over: dict[StatementNode, Action] = field(
        default_factory=dict, init=False, repr=False
    )
    """
    The candidate each statement rebuilt by the current :meth:`build` runs.
    """

    _rebuilt: dict[StatechartNode, StatechartNode | None] = field(
        default_factory=dict, init=False, repr=False
    )
    """
    Every node the current :meth:`build` rebuilt so far, so that a node referred to
    twice is rebuilt once.
    """

    _cut: bool = field(default=False, init=False, repr=False)
    """
    Whether a branch was cut below the node rebuilt last.
    """

    @classmethod
    def remainder_of(
        cls, parallel: Parallel, statement: StatementNode, proposal: Action
    ) -> ParallelRebuild:
        """
        :param parallel: A running parallel.
        :param statement: The statement of `parallel` grounding a candidate on its own.
        :param proposal: The candidate proposed for `statement`.
        :return: The rebuild of what is left to run of `parallel`: the nodes that ended
            are left out, since what they did is in the world already, a statement
            running a candidate keeps running it, and `statement` runs `proposal`.
        """
        descendants = parallel.descendants
        running_candidates = {
            node: node.grounded_nodes[-1]
            for node in descendants
            if isinstance(node, StatementNode)
            and node.latest_child is not None
            and not node.latest_child.life_cycle_state.is_terminal
        }
        return cls(
            parallel=parallel,
            candidates={**running_candidates, statement: proposal},
            left_out={
                node for node in descendants if node.life_cycle_state.is_terminal
            },
        )

    def build(self, world: World, cuts_branches: bool) -> BuiltParallel:
        """
        :param world: The world the rebuilt nodes refer to.
        :param cuts_branches: Whether every branch ends before its first statement
            without a candidate, as it does for a trial, since what follows that
            statement depends on how it is grounded.
        :return: :attr:`parallel` rebuilt onto `world`.
        """
        self._world = world
        self._cuts_branches = cuts_branches
        self._handed_over = {}
        self._rebuilt = {}
        self._cut = False
        return BuiltParallel(
            parallel=self._rebuild_node(self.parallel),
            candidates=self._handed_over,
        )

    def _rebuild_node(self, node: StatechartNode) -> StatechartNode | None:
        """
        :return: `node` rebuilt onto :attr:`_world`, or None if it is left out.
        """
        if node not in self._rebuilt:
            self._rebuilt[node] = self._rebuild_anew(node)
        return self._rebuilt[node]

    def _rebuild_anew(self, node: StatechartNode) -> StatechartNode | None:
        """
        :return: `node` rebuilt onto :attr:`_world`, or None if it is left out, if a branch is
            cut at it, or if it is a control structure node left with no child.
        """
        if node in self.left_out:
            return None
        if isinstance(node, StatementNode):
            return self._rebuild_statement(node)
        if isinstance(node, CompositeNodeChoosingItsChild):
            return self._rebuild_running_child(node)
        if isinstance(node, Action):
            return node.rebound_to(self._world)
        arguments = {
            constructor_field.name: self._rebuild_value(
                node, getattr(node, constructor_field.name)
            )
            for constructor_field in fields(node)
            if constructor_field.init
        }
        if isinstance(node, ControlStructureNode) and not arguments["nodes"]:
            return None
        return type(node)(**arguments)

    def _rebuild_statement(self, statement: StatementNode) -> StatementNode | None:
        """
        :return: A statement of its own grounding the statement of `statement`, running
            its candidate from :attr:`candidates`, or None if it has none and branches
            are cut at such a statement.
        """
        candidate = self.candidates.get(statement)
        if candidate is None and self._cuts_branches:
            self._cut = True
            return None
        rebuilt = StatementNode(statement=statement.statement)
        if candidate is not None:
            self._handed_over[rebuilt] = candidate.rebound_to(self._world)
        return rebuilt

    def _rebuild_running_child(
        self, node: CompositeNodeChoosingItsChild
    ) -> StatechartNode | None:
        """
        :return: The child `node` runs now, rebuilt, or None if it runs none, which cuts
            the branch, since what `node` will run is not chosen yet.
        """
        latest_child = node.latest_child
        if latest_child is None or latest_child.life_cycle_state.is_terminal:
            self._cut = True
            return None
        return self._rebuild_node(latest_child)

    def _rebuild_value(self, owner: StatechartNode, value: Any) -> Any:
        """
        :param owner: The node `value` is a constructor argument of.
        :return: `value` referring to :attr:`_world`, the nodes in it rebuilt.
        """
        if isinstance(value, StatechartNode):
            return self._rebuild_node(value)
        if isinstance(value, list) and any(
            isinstance(element, StatechartNode) for element in value
        ):
            return self._rebuild_children(owner, value)
        return self._world.rebind_world_entities(value)

    def _rebuild_children(
        self,
        owner: StatechartNode,
        children: list[StatechartNode],
    ) -> list[StatechartNode]:
        """
        :param owner: The node holding `children`.
        :return: `children` rebuilt, leaving out those left out, and, if `owner` runs
            its children in turn, every child after the first one a branch was cut in.
        """
        cut_before = self._cut
        cut_in_children = False
        rebuilt_children = []
        for child in children:
            self._cut = False
            rebuilt = self._rebuild_node(child)
            if rebuilt is not None:
                rebuilt_children.append(rebuilt)
            cut_in_children = cut_in_children or self._cut
            if self._cut and isinstance(
                owner, ControlStructureNodeRunningItsChildrenInTurn
            ):
                break
        self._cut = cut_before or cut_in_children
        return rebuilt_children

    def __str__(self) -> str:
        candidates = " beside ".join(map(str, self.candidates.values()))
        return f"{candidates} in {self.parallel.name}"


@dataclass
class ActionTrial:
    """
    Tries grounded actions against a copy of the world, to check that a candidate can
    succeed before it is attempted for real.

    One copy serves every candidate: after each attempt its model is rolled back and its
    state restored, and when the executor's world has changed since, the copy replays
    those model and state changes instead of being taken anew. Collision rules changed
    after the copy was taken are not carried over.

    Trials never publish to a synchronizer and always run simulated, in an executor of
    :attr:`trial_executor_type` over the copy, with the context extensions of
    :attr:`executor` rebound to the copy, and the base put where a candidate drives it
    by :attr:`plan_adaptations`.
    While the executor is debugging, the copy is shown in RViz under its own frame
    prefix and marker topic.
    """

    executor: PlanExecutor
    """
    The executor the candidates are grounded for.

    Only ever read from: a trial never mutates its world, and the candidates themselves
    are left untouched too, so they can still run for real afterwards.
    """

    trial_executor_type: type[PlanExecutor]
    """
    The executor that runs a candidate in the copy, which simulates the robot.
    """

    plan_adaptations: list[PlanTransformation] = field(
        default_factory=lambda: [
            WriteTheOdometryInsteadOfDriving(),
            KeepStretchInFrontOfTheContainerItCloses(),
        ],
        kw_only=True,
    )
    """
    The plan transformations adapting every candidate to being tried, in place of the
    plan adaptations of :attr:`trial_executor_type`.
    """

    copy_marker_alpha: float = field(default=0.9, kw_only=True)
    """
    The opacity the copy is drawn with while debugging, so it can be told apart from the
    world it copies where the two overlap.
    """

    _copied_world: World | None = field(default=None, init=False, repr=False)
    """
    The copy candidates are tried against.
    """

    _source_versions: tuple[int, int] | None = field(
        default=None, init=False, repr=False
    )
    """
    The model and state versions of the executor's world when the copy last matched it,
    used to notice that it has moved on and the copy has to be caught up.
    """

    _replayed_modification_blocks: int = field(default=0, init=False, repr=False)
    """
    How many of the modification blocks of the executor's world the copy already holds.
    """

    @property
    def world(self) -> World:
        """
        :return: The world the candidates are grounded in.
        """
        return self.executor.context.world

    def succeeds(self, action: Action) -> bool:
        """
        Run `action` against the copy and restore the copy afterwards.

        The action is rebuilt from its own parameters, rebound onto the copy, because an
        action that modifies the model (attaching a grasped body, say) requires the
        entities it is given to belong to the world being modified, and because a
        statechart node belongs to one statechart only. It runs as the child of a
        statement does, see :meth:`PlanChildChooser.steps_of`, so the nodes a plan
        transformation puts beside it are tried with it.

        :param action: The grounded action to try out.
        :return: True if `action` runs to completion without raising an
            :class:`~cramph.exceptions.ExecutionFailure`.
        """

        def build(world: World, statechart: Statechart) -> None:
            statechart.add_node(PlanChildChooser.steps_of(action.rebound_to(world)))

        return self._runs_to_completion(build, str(action))

    def parallel_succeeds(self, rebuild: ParallelRebuild) -> bool:
        """
        Run the parallel `rebuild` builds against the copy, every branch cut at its
        first statement without a candidate and the other statements running the
        candidates they are handed and no other, and restore the copy afterwards.

        :param rebuild: Builds the parallel to try out.
        :return: True if the parallel runs to completion without raising an
            :class:`~cramph.exceptions.ExecutionFailure`.
        """

        def build(world: World, statechart: Statechart) -> None:
            built = rebuild.build(world, cuts_branches=True)
            # A parallel is ended by what holds it.
            statechart.add_node(Sequence(nodes=[built.parallel]))
            statechart.context.require_extension(ChildChooserAccess).choosers = [
                NoFurtherCandidate()
            ]
            built.hand_over_candidates()

        return self._runs_to_completion(build, str(rebuild))

    def _runs_to_completion(
        self, build: Callable[[World, Statechart], None], description: str
    ) -> bool:
        """
        The version to roll back to is read here rather than when the copy is taken, so
        each attempt undoes only its own modifications.

        :param build: Adds what is tried to the statechart it is given, built over the
            copy it is given.
        :param description: What is tried, for the log.
        :return: True if what `build` adds runs to completion against the copy without
            raising an :class:`~cramph.exceptions.ExecutionFailure`.
        """
        world = self._copy()
        version = world.get_world_model_manager().version

        with world.reset_state_context():
            try:
                executor = self._trial_executor(world)
                statechart = Statechart(context=executor.context)
                build(world, statechart)
                executor.compile(statechart)
                executor.execute()
                return True
            except ExecutionFailure as failure:
                logger.info(f"{description} failed its trial: {failure}")
                return False
            finally:
                # Undo the model changes before leaving the reset context restores the
                # state, which needs the degrees of freedom it was snapshotted with.
                world.rollback_to_version(version)

    def _trial_executor(self, world: World) -> PlanExecutor:
        """
        :param world: The copy a candidate is tried in.
        :return: An executor running a candidate in `world`, configured like
            :attr:`executor`, the robot and everything else its context extensions refer
            to rebound to `world`, adapting the candidate with :attr:`plan_adaptations`,
            and showing `world` in RViz while it is debugging.
        """
        return self.trial_executor_type(
            StatechartContext.with_extensions(
                world,
                [
                    world.rebind_world_entities(extension)
                    for extension in self.executor.supplied_context_extensions
                ],
            ),
            universal_adaptations=self.executor.universal_adaptations,
            plan_adaptations=self.plan_adaptations,
            collision_avoidance=self.executor.collision_avoidance,
            extensions=self._trial_visualizations(),
        )

    def _trial_visualizations(self) -> list[ExecutorExtension]:
        """
        :return: The RViz visualization of the copy, through the ROS node of
            :attr:`executor`, under its own frame prefix and marker topic, while the
            executor is debugging; empty otherwise.
        """
        if not self.executor.debug:
            return []
        return [
            RvizVisualization(
                ros_node=self.executor.ros_node,
                collision_visualization=True,
                frame_prefix=ActionTrialVisualization.FRAME_PREFIX,
                marker_topic=ActionTrialVisualization.MARKER_TOPIC,
                marker_alpha=self.copy_marker_alpha,
            )
        ]

    def _copy(self) -> World:
        """
        :return: The copy to try candidates against, caught up with the executor's world
            if that has changed since the copy last matched it.
        """
        versions = (
            self.world.get_world_model_manager().version,
            self.world.state.version,
        )
        if self._copied_world is None:
            self._take_copy()
        elif self._source_versions != versions:
            self._catch_up()
        self._source_versions = versions
        return self._copied_world

    def _take_copy(self) -> None:
        """
        Copy the executor's world.
        """
        self._copied_world = deepcopy(self.world)
        self._replayed_modification_blocks = len(
            self.world.get_world_model_manager().model_modification_blocks
        )

    def _catch_up(self) -> None:
        """
        Bring the copy up to date with the executor's world: replay the modifications
        made to it since, the way copying it replays all of them, and take over its
        state.

        The copy's own modifications are all rolled back by then, so it still matches
        the world as it was when it last caught up.
        """
        modification_blocks = (
            self.world.get_world_model_manager().model_modification_blocks
        )
        with self._copied_world.modify_world():
            for block in modification_blocks[self._replayed_modification_blocks :]:
                block.update_references_for_world_and_apply(world=self._copied_world)
            self._copied_world.state.merge_state(self.world.state)
        self._replayed_modification_blocks = len(modification_blocks)

    def discard(self) -> None:
        """
        Release the copy, so the next trial takes a fresh one.
        """
        self._copied_world = None
        self._source_versions = None


# %% grounding underspecified actions while the statechart runs


@dataclass
class UnderspecifiedChildChooser(PlanChildChooser[StatementNode]):
    """
    Grounds the statement of every :class:`~cramph.composites.StatementNode` of a
    statechart into the action it runs next, see :meth:`PlanChildChooser.steps_of`.

    A grounded action is tried out first, so one that fails its trial is discarded
    without ever touching the real world, and a bad parameterization cannot poison a
    later attempt. A statement running in a parallel is tried beside what is left to run
    of that parallel, see :meth:`ParallelRebuild.remainder_of`.
    """

    trial: ActionTrial
    """
    The trial every node tries its candidates against, so they all try them in one copy
    of the world.
    """

    def choose_child(self, node: StatementNode) -> StatechartNode | None:
        """
        :return: The child running the next grounded action that succeeds its trial, or
            None once the statement ran out of actions that succeed their trial.
        """
        grounding = node.context.require_extension(StatementGrounding)
        parallel = self._parallel_around(node)
        proposal = node.next_candidate(
            grounding.query_backend, grounding.candidates_to_try
        )
        while proposal is not None:
            if self._succeeds(node, proposal, parallel):
                return self.steps_of(proposal)
            proposal = node.next_candidate(
                grounding.query_backend, grounding.candidates_to_try
            )
        node.stop_grounding()
        return None

    def _succeeds(
        self, node: StatementNode, proposal: Action, parallel: Parallel | None
    ) -> bool:
        """
        :return: Whether `proposal` for `node` succeeds its trial, beside what is left to
            run of `parallel`, or on its own if `node` runs in no parallel.
        """
        if parallel is None:
            return self.trial.succeeds(proposal)
        return self.trial.parallel_succeeds(
            ParallelRebuild.remainder_of(parallel, statement=node, proposal=proposal)
        )

    @staticmethod
    def _parallel_around(node: StatementNode) -> Parallel | None:
        """
        :return: The outermost parallel above `node` with no action in between, which
            holds everything that runs beside it, or None if there is none. A parallel
            that is an action, and one above an action, are left out, because a trial
            builds an action anew from its parameters, which grounds what it runs anew.
        """
        around = None
        for ancestor in node.path:
            if isinstance(ancestor, Action):
                return around
            if isinstance(ancestor, Parallel):
                around = ancestor
        return around

    def cleanup(self) -> None:
        """
        Release the trial's world copy.
        """
        self.trial.discard()


@dataclass
class UnderspecifiedParallelChildChooser(PlanChildChooser[UnderspecifiedParallel]):
    """
    Grounds the statements of every
    :class:`~cramph.composites.UnderspecifiedParallel` together, into the first
    combination of their candidates whose trial succeeds as the whole parallel, and runs
    the parallel with them, see :class:`ParallelRebuild`.

    It chooses once per run of the node: a statement whose child failed grounds a
    candidate of its own, see :class:`UnderspecifiedChildChooser`, so a branch that
    succeeded never runs again.
    """

    trial: ActionTrial
    """
    The trial every combination is tried against.
    """

    _built: dict[UnderspecifiedParallel, BuiltParallel] = field(
        default_factory=dict, init=False, repr=False
    )
    """
    The parallel built for each node, until it joined the statechart.
    """

    def choose_child(self, node: UnderspecifiedParallel) -> StatechartNode | None:
        """
        :return: The parallel running the first combination that succeeds its trial,
            held by a sequence, which ends it, or None once no combination is left, or
            once the parallel chosen for the current run of `node` failed.
        """
        if node.latest_child is not None:
            node.stop_grounding()
            return None
        grounding = node.context.require_extension(StatementGrounding)
        combination = node.next_combination(
            grounding.query_backend, grounding.candidates_to_try
        )
        while combination is not None:
            grounded = ParallelRebuild(parallel=node.parallel, candidates=combination)
            if self.trial.parallel_succeeds(grounded):
                built = grounded.build(node.context.world, cuts_branches=False)
                self._built[node] = built
                return Sequence(nodes=[built.parallel])
            combination = node.next_combination(
                grounding.query_backend, grounding.candidates_to_try
            )
        return None

    def prepare_chosen_child(
        self, node: UnderspecifiedParallel, child: StatechartNode
    ) -> None:
        """
        Differs from :meth:`PlanChildChooser.prepare_chosen_child` in first handing
        every grounded statement of the parallel its candidate, so that the parallel
        arrives wherever it runs with them chosen.
        """
        self._built.pop(node).hand_over_candidates()
        super().prepare_chosen_child(node, child)

    def failure_monitors_for(
        self, node: UnderspecifiedParallel, child: StatechartNode
    ) -> list[StatechartNode]:
        """
        Differs from :meth:`PlanChildChooser.failure_monitors_for` in handing none:

        every candidate is watched for a stall by a monitor of its own, and a branch
        grounding a candidate anew stops nothing beside it.
        """
        return []

    def cleanup(self) -> None:
        """
        Release the trial's world copy and the parallels not yet chosen.
        """
        self.trial.discard()
        self._built.clear()


# %% grounding the statements that start with a parallel together


@dataclass
class GroundTheStatementsOfAParallelTogether(PlanTransformation[Parallel]):
    """
    Puts a :class:`~cramph.composites.UnderspecifiedParallel` in the place of every
    parallel with statements that start with it, so that they are grounded together,
    each to a candidate suiting the others, before any of them runs.
    """

    def is_applicable(self, plan_node: Parallel) -> bool:
        holder = plan_node.parent_node
        return (
            not isinstance(plan_node, Action)
            and (holder is None or isinstance(holder, ControlStructureNode))
            and bool(self.statements_starting_with(plan_node))
        )

    def apply(self, plan_node: Parallel) -> None:
        plan_node.statechart.replace(
            plan_node,
            UnderspecifiedParallel(
                parallel=plan_node,
                statements=self.statements_starting_with(plan_node),
            ),
        )

    @classmethod
    def statements_starting_with(cls, node: StatechartNode) -> list[StatementNode]:
        """
        :return: The statements that start when `node` starts and have not chosen a
            child: `node` itself, or those of every child of a parallel and of the first
            child of a node running its children in turn. None in an action, which
            grounds its own.
        """
        if isinstance(node, StatementNode):
            return [] if node.chosen_children else [node]
        if isinstance(node, Action):
            return []
        if isinstance(node, Parallel):
            children = node.nodes
        elif isinstance(node, ControlStructureNodeRunningItsChildrenInTurn):
            children = node.nodes[:1]
        else:
            return []
        return [
            statement
            for child in children
            for statement in cls.statements_starting_with(child)
        ]
