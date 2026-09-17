from __future__ import division

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from itertools import combinations
from typing import List

from typing_extensions import Optional

from cramph.context import StatechartContext
from cramph.data_types import (
    LifeCyclePredicate,
    LifeCycleValues,
    ObservationStateValues,
    SuccessDecider,
)
from cramph.exceptions import AttemptCannotFailError
from cramph.node import (
    CancelStatechart,
    CompositeNode,
    StatechartNode,
    NodeArtifacts,
    TerminalNode,
)
from krrood.exceptions import DataclassException
from krrood.symbolic_math.symbolic_math import (
    Scalar,
    trinary_if_cases,
    sum,
    trinary_logic_and,
    trinary_logic_not,
    trinary_logic_or,
    logic_and,
    logic_not,
    logic_or,
)

# %% giving a node an outcome


@dataclass(repr=False, eq=False)
class Attempt(CompositeNode):
    """
    Runs a node that would never end on its own and decides it, one way or the other.

    A node that keeps holding its goal observes only whether it is at that goal right
    now, so nothing about it ever concludes. This goal concludes instead: it
    observes True once the task is at its goal, which ends it as a success, and False
    once one of :attr:`failure_monitors` fires, which is what it declares its own
    failure on. That is what lets a maintained node be one step of a plan.

    It declares that failure as well once the task ended without succeeding, which is the
    task having concluded on its own and nothing else here would notice.

    .. note:: The task is never ended from here. It keeps exerting itself after first
        reaching its goal and comes down only with this goal, so a task that was pushed
        off its goal again is still being held.
    """

    success_decided_by = SuccessDecider.ITSELF
    fails_when_observing_false = True

    task: StatechartNode = field(kw_only=True)
    """
    The node run until this goal is decided.
    """

    failure_monitors: List[StatechartNode] = field(kw_only=True)
    """
    The nodes whose observing True gives up on the task, any one of which is enough.

    An empty list states that this node cannot fail, leaving reaching its goal as the
    only way it ends. A monitor is read the way it is written, so one that observes
    being *well* has to be negated before it can be passed here.
    """

    @property
    def any_failure_monitor_fired(self) -> Scalar:
        """
        :return: True once a failure monitor fired, and false while none has or there
            are none to fire.
        """
        if not self.failure_monitors:
            return Scalar.const_false()
        return trinary_logic_or(
            *[monitor.last_observed_true for monitor in self.failure_monitors]
        )

    @property
    def can_fail(self) -> bool:
        """
        Whether this goal has a way to fail: a failure monitor, or a task that fails on
        its own.
        """
        return bool(self.failure_monitors) or self.task.can_fail_on_its_own

    @property
    def failure_reasons(self) -> List[StatechartNode]:
        """
        Which monitors gave up on the task, which is what turns a failure into a reason.

        They are read through their last observation, because ending this goal ends them
        too and a node that ended observes nothing any more.

        :return: The failure monitors that fired, in the order they were given, and
            nothing at all unless this goal declared itself failed. Empty as well for a
            failure the task reached on its own, which no monitor is the reason for.
        """
        if self.life_cycle_state != LifeCycleValues.FAILED:
            return []
        return [
            monitor
            for monitor in self.failure_monitors
            if monitor.last_observation_state == ObservationStateValues.TRUE
        ]

    def expand(self, context: StatechartContext) -> None:
        """
        Add the task and the monitors.

        A monitor that fires fails this goal on the same tick, which interrupts the
        monitor and so keeps the observation it fired on as its last observation.
        """
        self._add_child_to_statechart(self.task)
        self._add_children_to_statechart(self.failure_monitors)

    def build_artifacts(self, context: StatechartContext) -> NodeArtifacts:
        """
        Report reaching the goal, being given up on, or neither.

        Reaching the goal outranks a monitor firing on the same tick: a task
        that arrived did what it was asked, whatever else was true at that moment. The
        task is read through its last observation, which is what it observes for as long
        as this goal holds it open, and what it arrived at if it ended itself.

        A task that ended without succeeding is reported the same way a monitor giving up
        is: nothing will move it any more, and an attempt still waiting for it would never
        end. What it observed when it ended does not count as reaching its goal.
        """
        task_at_its_goal = logic_and(
            self.task.last_observed_true,
            logic_not(self.task.is_failed_or_interrupted),
        )
        return NodeArtifacts(
            observation=trinary_if_cases(
                cases=[
                    (task_at_its_goal, Scalar.const_true()),
                    (self.any_failure_monitor_fired, Scalar.const_false()),
                    (self.task.is_failed_or_interrupted, Scalar.const_false()),
                ],
                else_result=Scalar.const_trinary_unknown(),
            )
        )


# %% running a list of nodes


@dataclass(repr=False, eq=False)
class NodeListCompositeNode(CompositeNode):
    """
    A composite node that runs the list of nodes it is handed.

    The nodes join the statechart when :meth:`expand` adds them during compilation, so a
    node handed over before that is serialized once, inside this goal.
    """

    nodes: List[StatechartNode] = field(default_factory=list, init=True)
    """
    The nodes this goal runs, in the order they were handed over.
    """

    def add_node(self, node: StatechartNode) -> None:
        """
        Hands this goal one more node to run.

        :param node: The node to run as a child of this goal.
        """
        self._add_node_sanity_check(node)
        if node in self.nodes:
            return
        self.nodes.append(node)


# %% goals built from nodes that end on their own


@dataclass(repr=False, eq=False)
class CompositeNodeOverSelfDecidingNodes(CompositeNode, ABC):
    """
    Base for the goals that order or choose between children, which only works if each
    child reaches a terminal state by itself.

    Such a goal reads its children's outcomes and never their observations, and it owns
    their life cycles: what starts and ends a child is this goal's to decide. What comes
    out decides itself in turn, which is what lets one be a step of another.
    """

    success_decided_by = SuccessDecider.ITSELF
    fails_when_observing_false = True

    def _add_self_deciding(self, node: StatechartNode) -> StatechartNode:
        """
        Adds a child that ends on its own, converting the caller's node where it needs
        converting.

        A node whose owner decides its success observes whether it reached its goal, so
        one is wrapped in an :class:`Attempt` without failure monitors, which fails only
        if the node fails on its own.

        :param node: The child the caller passed.
        :return: The child to run in its place, which may be `node` itself.
        """
        self._check_caller_wired_no_transitions(node)
        self._check_node_doesnt_belong_to_different_parent(node)
        if node.success_decided_by == SuccessDecider.ITSELF:
            self._add_child_to_statechart(node)
            return node
        attempt = Attempt(name=f"{node.name}/attempt", task=node, failure_monitors=[])
        # The attempt takes the node's place among the children and becomes its parent,
        # so the children stay in the order the caller wrote them in.
        if node in self.nodes:
            self.nodes[self.nodes.index(node)] = attempt
        self._add_child_to_statechart(attempt)
        node.parent_node = attempt
        return attempt

    def _check_attempt_can_fail(self, node: StatechartNode) -> None:
        """
        Rejects a child this goal only moves on from once it failed, if it is an attempt
        that cannot fail.

        :param node: The child this goal waits on to fail.
        :raises AttemptCannotFailError: If `node` is an :class:`Attempt` that cannot
            fail.
        """
        if isinstance(node, Attempt) and not node.can_fail:
            raise AttemptCannotFailError(node=self, attempt=node)


@dataclass(repr=False, eq=False)
class Sequence(NodeListCompositeNode, CompositeNodeOverSelfDecidingNodes):
    """
    Runs a list of nodes one after another.

    Its observation turns True once the last step succeeded, and False as soon as a step
    ended without succeeding, so a step that was given up on fails the sequence rather
    than leaving it waiting forever.

    .. note:: corresponds to the RPL's SEQ. (McDermott, Drew. A reactive plan language, 1991)
    """

    _steps: List[StatechartNode] = field(default_factory=list, init=False)
    """
    The nodes actually run, which is what the caller passed with every plain task
    wrapped in an attempt.
    """

    def expand(self, context: StatechartContext) -> None:
        """
        Each step is a node that ends on its own, and the next one waits for the outcome
        it earned, because only an outcome outlasts the step that reached it.
        """
        self._check_has_children()
        previous_step: Optional[StatechartNode] = None
        for node in list(self.nodes):
            # A node that ends the statechart decides nothing and has nothing to convert.
            if isinstance(node, TerminalNode):
                self._add_child_to_statechart(node)
                step = node
            else:
                step = self._add_self_deciding(node)
            if previous_step is not None:
                step.start_condition = previous_step.is_succeeded
            self._steps.append(step)
            previous_step = step

    def build_artifacts(self, context: StatechartContext) -> NodeArtifacts:
        """
        Report success, a failed step, or neither, all read off the steps' outcomes.

        A step that is still running has not failed, it has not arrived yet, so only a
        step that ended decides anything.
        """
        return NodeArtifacts(
            observation=trinary_if_cases(
                cases=[
                    (
                        trinary_logic_or(
                            *[step.is_failed_or_interrupted for step in self._steps]
                        ),
                        Scalar.const_false(),
                    ),
                    (self._steps[-1].is_succeeded, Scalar.const_true()),
                ],
                else_result=Scalar.const_trinary_unknown(),
            )
        )


@dataclass(repr=False, eq=False)
class Parallel(NodeListCompositeNode):
    """
    Holds a list of nodes at once until enough of them are at their goals together.

    Its observation turns True once at least :attr:`minimum_success` of them are at
    their goals on the same tick.

    Unlike the goals that run steps, this one ends none of its nodes and reads what they
    observe now, because releasing a node that reached its goal would let a sibling
    undo what it reached. For the same reason its own owner decides
    when it succeeded: a plan step built from one is an attempt wrapping it.
    """

    success_decided_by = SuccessDecider.OWNER

    minimum_success: Optional[int] = field(default=None, kw_only=True)
    """
    How many nodes must have reached their goals for this goal to be achieved.

    Defaults to None, which means all of them.
    """

    @property
    def required_successes(self) -> int:
        """
        :return: How many nodes have to reach their goals, which is all of them unless
            :attr:`minimum_success` says otherwise.
        """
        if self.minimum_success is None:
            return len(self.nodes)
        return self.minimum_success

    def expand(self, context: StatechartContext) -> None:
        """
        Add the nodes, and declare this goal failed once too few of them can still reach
        their goals.

        Observing False means the nodes are not at their goals, which is not a failure
        and is left to the attempt this goal is wrapped in. A node that ended without
        succeeding is different: nothing brings it back, so once too few are left this
        goal can no longer arrive and says so rather than holding its owner open forever.
        """
        self._check_has_children()
        for node in self.nodes:
            self._add_child_to_statechart(node)
        self.fail_condition = logic_or(
            self.fail_condition, self._cannot_arrive_any_more
        )

    @property
    def _cannot_arrive_any_more(self) -> Scalar:
        """
        Asks whether so many nodes ended without succeeding that
        :attr:`required_successes` is out of reach.

        Counting would say this in one line, but a transition condition has to render
        back into the expression it was written as, which only leaves the logic
        operators: the question becomes which groups of nodes ending without succeeding
        are enough, one term per group.

        :return: True once too few nodes are left to reach :attr:`required_successes`.
        """
        nodes_that_must_end_without_succeeding = (
            len(self.nodes) - self.required_successes + 1
        )
        if nodes_that_must_end_without_succeeding <= 0:
            return Scalar.const_true()
        if nodes_that_must_end_without_succeeding > len(self.nodes):
            return Scalar.const_false()
        return logic_or(
            *[
                logic_and(*[node.is_failed_or_interrupted for node in group])
                for group in combinations(
                    self.nodes, nodes_that_must_end_without_succeeding
                )
            ]
        )

    def build_artifacts(self, context: StatechartContext) -> NodeArtifacts:
        """
        Count the nodes that are at their goals against :attr:`required_successes`.

        This goal ends none of its nodes, so a node that keeps running is counted by
        what it observes now and stops counting once it observes False again. A node that
        succeeded on its own is counted by the last observation it took, which outlasts
        it; one that ended without succeeding stops counting, because the reading it kept
        says where it was cut off rather than where it is.

        Observing False means the nodes are not at their goals, not that anything went
        wrong: whether that is worth giving up on is decided outside, by the attempt this
        goal is wrapped in.
        """
        nodes_at_their_goals = [
            trinary_logic_and(
                node.last_observed_true,
                trinary_logic_not(node.is_failed_or_interrupted),
            )
            for node in self.nodes
        ]
        return NodeArtifacts(
            observation=self.required_successes <= sum(*nodes_at_their_goals)
        )


# %% repeating a task


@dataclass(repr=False, eq=False)
class RepeatUntil(CompositeNodeOverSelfDecidingNodes):
    """
    Runs a task again from the start whenever an attempt at it fails.

    Its observation turns True once the task succeeds and False once
    :attr:`stop_retry_monitor` calls the retrying off, so a caller can tell "eventually
    worked" from "gave up".

    What counts as a failed attempt is stated on the task itself: hand it an
    :class:`Attempt` carrying the failure monitors that decide it, or see
    :class:`RepeatOnStall`, which derives that decision from the task's own progress. A
    task that never ends on its own is attempted without failure monitors, which is
    rejected unless the task fails on its own, since it would never be retried.
    """

    task: StatechartNode = field(kw_only=True)
    """
    The node to run, and to run again after every failed attempt.

    Resetting a goal resets everything below it, so a composite task starts over as a
    unit.
    """

    stop_retry_monitor: StatechartNode = field(kw_only=True)
    """
    Stops the retrying once it observes True, which makes this goal observe False.
    """

    exception: Optional[DataclassException] = field(default=None, kw_only=True)
    """
    The failure that ends the statechart once :attr:`stop_retry_monitor` calls the
    retrying off, or None to only observe False then.
    """

    _attempt: Optional[StatechartNode] = field(default=None, init=False)
    """
    The node actually run, which is :attr:`task` wrapped in an attempt if it needed one.
    """

    def expand(self, context: StatechartContext) -> None:
        """
        Wire the retry loop.

        The attempt declares its own failure, and that outcome is what starts the next
        try: a node takes at most one transition triggered by its own conditions per
        tick, so the reset lands the tick after the failure rather than on it.

        The stop monitor is asked whether its last observation is True, which outlasts a
        monitor that ends itself on reaching what it counts, and which a monitor that has
        not observed anything yet has not reached either.
        """
        self._attempt = self._add_self_deciding(self.task)
        self._add_child_to_statechart(self.stop_retry_monitor)

        retrying_stopped = self._retrying_stopped
        still_trying = logic_not(retrying_stopped)
        # Starting is gated as well as ending, because a reset task is not started and
        # ending is not considered while it is not.
        self._attempt.start_condition = still_trying
        self._attempt.reset_condition = logic_and(self._attempt.is_failed, still_trying)
        self._attempt.interrupt_condition = retrying_stopped
        self._end_statechart_once_retrying_stops()

    def check_children(self) -> None:
        """
        Rejects an attempt that cannot fail, since it would never be retried.
        """
        self._check_attempt_can_fail(self._attempt)

    @property
    def _retrying_stopped(self) -> Scalar:
        """
        :return: True once :attr:`stop_retry_monitor` observed True, even if it ended
            since; false while it has not.
        """
        return self.stop_retry_monitor.last_observed_true

    def _end_statechart_once_retrying_stops(self) -> None:
        """
        Add the node that ends the statechart with :attr:`exception` once
        :attr:`stop_retry_monitor` calls the retrying off.
        """
        if self.exception is None:
            return
        exhausted = CancelStatechart(
            name=f"{self.name}/exhausted", exception=self.exception
        )
        self._add_child_to_statechart(exhausted)
        exhausted.start_condition = self._retrying_stopped

    def build_artifacts(self, context: StatechartContext) -> NodeArtifacts:
        """
        Report success, giving up, or neither.

        Both children are read through something that outlasts them: the attempt through
        its outcome, which the reset that starts the next try clears again, and the stop
        monitor through its last observation.
        """
        return NodeArtifacts(
            observation=trinary_if_cases(
                cases=[
                    (self._attempt.is_succeeded, Scalar.const_true()),
                    (self._retrying_stopped, Scalar.const_false()),
                ],
                else_result=Scalar.const_trinary_unknown(),
            )
        )


# %% trying alternatives


@dataclass(repr=False, eq=False)
class TryAll(NodeListCompositeNode, CompositeNodeOverSelfDecidingNodes):
    """
    Runs a list of alternatives at once and takes the first one that works.

    Its observation turns True as soon as an alternative succeeded, and False only once
    every one of them ended without doing so.
    """

    _alternatives: List[StatechartNode] = field(default_factory=list, init=False)
    """
    The nodes actually run, which is what the caller passed with every plain task
    wrapped in an attempt.
    """

    def expand(self, context: StatechartContext) -> None:
        """
        Add every alternative, so they run side by side.
        """
        self._check_has_children()
        self._alternatives = [
            self._add_self_deciding(node) for node in list(self.nodes)
        ]

    def build_artifacts(self, context: StatechartContext) -> NodeArtifacts:
        """
        Report the first alternative that worked, or that none of them did.
        """
        return NodeArtifacts(
            observation=trinary_if_cases(
                cases=[
                    (
                        trinary_logic_or(
                            *[
                                alternative.is_succeeded
                                for alternative in self._alternatives
                            ]
                        ),
                        Scalar.const_true(),
                    ),
                    (
                        trinary_logic_and(
                            *[
                                alternative.is_failed_or_interrupted
                                for alternative in self._alternatives
                            ]
                        ),
                        Scalar.const_false(),
                    ),
                ],
                else_result=Scalar.const_trinary_unknown(),
            )
        )


@dataclass(repr=False, eq=False)
class TryInOrder(NodeListCompositeNode, CompositeNodeOverSelfDecidingNodes):
    """
    Tries a list of alternatives one after another, short-circuiting on the first
    success.

    The next alternative only starts once the previous one has ended without
    succeeding. Its observation turns True as soon as an alternative succeeds and False
    only once every one of them is over, so it stays unknown while any is still running.

    Each alternative decides for itself when to give up, which is why this goal reduces
    to ordering: wrap one in an :class:`Attempt` carrying the monitors that decide it.
    An attempt that cannot fail is rejected anywhere but last, since the alternatives
    after it could never start.

    .. note:: corresponds to the RPL's TRY-IN-ORDER. (McDermott, Drew. A reactive plan language, 1991)
    """

    _alternatives: List[StatechartNode] = field(default_factory=list, init=False)
    """
    The nodes actually run, which is what the caller passed with every plain task
    wrapped in an attempt.
    """

    def expand(self, context: StatechartContext) -> None:
        """
        Wire each alternative to start once the previous one ended without succeeding,
        which short-circuits on the first success.
        """
        self._check_has_children()
        previous_alternative: Optional[StatechartNode] = None
        for node in list(self.nodes):
            alternative = self._add_self_deciding(node)
            if previous_alternative is not None:
                alternative.start_condition = (
                    previous_alternative.is_failed_or_interrupted
                )
            self._alternatives.append(alternative)
            previous_alternative = alternative

    def check_children(self) -> None:
        """
        Rejects an attempt that cannot fail before the last alternative, since the
        alternatives after it could never start.
        """
        for alternative in self._alternatives[:-1]:
            self._check_attempt_can_fail(alternative)

    def build_artifacts(self, context: StatechartContext) -> NodeArtifacts:
        """
        Report the alternative that worked, or that none of them did.
        """
        return NodeArtifacts(
            observation=trinary_if_cases(
                cases=[
                    (
                        trinary_logic_or(
                            *[
                                alternative.is_succeeded
                                for alternative in self._alternatives
                            ]
                        ),
                        Scalar.const_true(),
                    ),
                    (
                        trinary_logic_and(
                            *[
                                alternative.is_failed_or_interrupted
                                for alternative in self._alternatives
                            ]
                        ),
                        Scalar.const_false(),
                    ),
                ],
                else_result=Scalar.const_trinary_unknown(),
            )
        )


# %% monitored subtrees


@dataclass(repr=False, eq=False)
class MonitoredCompositeNode(CompositeNode, ABC):
    """
    Runs a monitored node next to the monitor observing it.

    What it observes is what the monitored node has reached, so nothing here ever
    concludes either: a plan step built from one is an attempt wrapping it.

    The two are siblings, which is what lets the monitor's observation drive the
    monitored node's life cycle: a transition condition may only reference the owning
    node or a sibling of it. Neither node is chained to the other, so the monitor
    observes from the moment this goal starts.
    """

    success_decided_by = SuccessDecider.OWNER

    monitor: StatechartNode = field(kw_only=True)
    """
    The node whose observation controls the monitored node.
    """

    monitored_node: Optional[StatechartNode] = field(default=None, kw_only=True)
    """
    The node placed under the monitor's control.
    """

    def expand(self, context: StatechartContext) -> None:
        """
        Add the monitor and the monitored node, wire the monitor, and declare this goal
        failed once the monitored node ended without succeeding, because it can no
        longer arrive.
        """
        self._add_child_to_statechart(self.monitor)
        self._add_child_to_statechart(self.monitored_node)
        self.wire_monitor()
        self.fail_condition = logic_or(
            self.fail_condition, self.monitored_node.is_failed_or_interrupted
        )

    @abstractmethod
    def wire_monitor(self) -> None:
        """
        Connect the monitor's observation to the monitored node's life cycle.
        """

    def build_artifacts(self, context: StatechartContext) -> NodeArtifacts:
        """
        The monitored node is read through its last observation, which outlasts it,
        because a node that ended observes nothing any more.
        """
        return NodeArtifacts(observation=Scalar(self.monitored_node.last_observation))


@dataclass(repr=False, eq=False)
class PausedWhileTrue(MonitoredCompositeNode):
    """
    Holds the monitored node for as long as the monitor observes True, and lets it
    continue once the monitor turns False again.
    """

    def wire_monitor(self) -> None:
        self.monitored_node.pause_condition = logic_or(
            self.monitor.observes_true,
            self.monitored_node.pause_condition,
        )


@dataclass(repr=False, eq=False)
class PausedUntilTrue(MonitoredCompositeNode):
    """
    Holds the monitored node until the monitor observes True, and lets it continue from
    then on.

    A monitor that has not observed anything yet has not turned True either, so it holds
    the monitored node as well.
    """

    def wire_monitor(self) -> None:
        self.monitored_node.pause_condition = logic_or(
            self.monitored_node.pause_condition,
            logic_not(self.monitor.observes_true),
        )


@dataclass(repr=False, eq=False)
class StoppedWhenTrue(MonitoredCompositeNode):
    """
    Interrupts the monitored node as soon as the monitor observes True.

    It observes True while the monitored node observes True or once it succeeded, False
    once the monitor stopped it, whatever it observed, and Unknown otherwise. Stopping
    the monitored node interrupts it, so this goal fails the way every monitored
    composite node does once its monitored node ended without succeeding.

    The monitor is read through its last observation, which outlasts a monitor that ends
    itself on firing, unlike the pausing goals, which need the reading it takes right
    now.
    """

    def wire_monitor(self) -> None:
        self.monitored_node.interrupt_condition = logic_or(
            self.monitored_node.interrupt_condition,
            self.monitor.last_observed_true,
        )

    def build_artifacts(self, context: StatechartContext) -> NodeArtifacts:
        """
        The monitored node's observation counts only while it has not ended; after that,
        only its success does.

        A node that ended keeps the observation it ended on until the next tick, so a
        node the monitor stopped is told apart from one that succeeded by its life cycle
        rather than by that observation.
        """
        return NodeArtifacts(
            observation=trinary_if_cases(
                [
                    (
                        self._monitored_node_observing_true_or_succeeded,
                        Scalar.const_true(),
                    ),
                    (self.monitor.last_observed_true, Scalar.const_false()),
                ],
                Scalar.const_trinary_unknown(),
            )
        )

    @property
    def _monitored_node_observing_true_or_succeeded(self) -> Scalar:
        """
        :return: True while the monitored node has not ended and observes True, and once
            it succeeded; false otherwise.
        """
        has_ended = LifeCyclePredicate.IS_TERMINATED.expression(
            self.monitored_node.life_cycle_variable
        )
        observing_true_while_running = trinary_logic_and(
            trinary_logic_not(has_ended),
            self.monitored_node.observes_true,
        )
        return trinary_logic_or(
            observing_true_while_running, self.monitored_node.is_succeeded
        )


@dataclass(repr=False, eq=False)
class CancelledWhenTrue(StoppedWhenTrue):
    """
    Interrupts the monitored node as soon as the monitor observes True, and ends the
    statechart with it.

    Nothing in a plan waits for a node that failed, so a monitor that gives up on its
    subtree has to end the statechart rather than leave the rest of the plan waiting for
    a subtree that will never succeed.
    """

    exception: DataclassException = field(kw_only=True)
    """
    The failure reported once the monitor ends the statechart.
    """

    def expand(self, context: StatechartContext) -> None:
        """
        Add the monitor and the monitored node, and the node that ends the statechart
        once the monitor observes True.
        """
        super().expand(context)
        cancelled = CancelStatechart(
            name=f"{self.name}/cancelled", exception=self.exception
        )
        self._add_child_to_statechart(cancelled)
        cancelled.start_condition = self.monitor.last_observed_true
