from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum, auto

from typing_extensions import List

import krrood.symbolic_math.symbolic_math as sm
from cramph.context import StatechartContext
from cramph.data_types import LifeCycleValues, ObservationStateValues, SuccessDecider
from cramph.exceptions import StatechartError
from cramph.composites import Sequence
from cramph.node import (
    StatechartNode,
    CompositeNode,
    NodeArtifacts,
    CancelStatechart,
)
from cramph.monitors import CountTicks, Pulse
from krrood.symbolic_math.symbolic_math import FloatVariable


@dataclass
class NodeAssertionError(StatechartError):
    """
    Raised by test statechart nodes when a behaviour they assert on is violated.
    """

    reason: str
    """
    Description of the violated assertion.
    """

    def error_message(self) -> str:
        return self.reason

    def suggest_correction(self) -> str:
        return ""


@dataclass(eq=False, repr=False)
class ConstTrueNode(StatechartNode):
    """
    A node that has always reached its goal, so ending it always succeeds it.
    """

    success_decided_by = SuccessDecider.OWNER

    def build_artifacts(self, context: StatechartContext) -> NodeArtifacts:
        return NodeArtifacts(observation=sm.Scalar.const_true())


@dataclass(eq=False, repr=False)
class ConstFalseNode(StatechartNode):
    """
    A node that never reaches its goal, so nothing but being released ever ends it.
    """

    success_decided_by = SuccessDecider.OWNER

    def build_artifacts(self, context: StatechartContext) -> NodeArtifacts:
        return NodeArtifacts(observation=sm.Scalar.const_false())


@dataclass(eq=False, repr=False)
class NodeWithOwnStructureCopy(StatechartNode):
    """
    A kind of node declared outside of the statechart's own node classes, whose structure
    copy is an instance of this kind.
    """

    success_decided_by = SuccessDecider.OWNER

    def create_structure_copy(self) -> NodeWithOwnStructureCopy:
        return NodeWithOwnStructureCopy(name=self.name)


@dataclass(eq=False, repr=False)
class SpecializedNodeWithOwnStructureCopy(NodeWithOwnStructureCopy):
    """
    A specialization of :class:`NodeWithOwnStructureCopy` whose structure copy falls back to that kind.
    """

    detail: int = field(default=0, kw_only=True)
    """
    A value only the specialization has, and its structure copy does not.
    """


@dataclass(repr=False, eq=False)
class ChangeStateOnEvents(StatechartNode):
    success_decided_by = SuccessDecider.OWNER

    state: str | None = None

    def on_start(self, context: StatechartContext):
        self.state = "on_start"

    def on_pause(self, context: StatechartContext):
        self.state = "on_pause"

    def on_unpause(self, context: StatechartContext):
        self.state = "on_unpause"

    def on_end(self, context: StatechartContext):
        self.state = "on_end"

    def on_reset(self, context: StatechartContext):
        self.state = "on_reset"


@dataclass(repr=False, eq=False)
class CompositeNodeWithChainedChildren(CompositeNode):
    """
    A composite node whose second child starts once its first child observes True, and
    which observes what its second child observes.
    """

    success_decided_by = SuccessDecider.OWNER

    sub_node1: ConstTrueNode = field(init=False)
    sub_node2: ConstTrueNode = field(init=False)

    def expand(self, context: StatechartContext) -> None:
        self.sub_node1 = ConstTrueNode(name="sub muh1")
        self._add_child_to_statechart(self.sub_node1)
        self.sub_node2 = ConstTrueNode(name="sub muh2")
        self._add_child_to_statechart(self.sub_node2)
        self.sub_node1.success_condition = self.sub_node1.observes_true
        self.sub_node2.start_condition = self.sub_node1.observes_true

    def build_artifacts(self, context: StatechartContext) -> NodeArtifacts:
        return NodeArtifacts(observation=self.sub_node2.observation_variable)


@dataclass(repr=False, eq=False)
class CompositeNodeWithNestedCompositeChild(CompositeNode):
    """
    A composite node with a single composite child, observing what that child observes.
    """

    success_decided_by = SuccessDecider.OWNER

    sub_node1: CompositeNodeWithChainedChildren = field(init=False)
    sub_node2: CompositeNodeWithChainedChildren = field(init=False)
    inner: CompositeNodeWithChainedChildren = field(init=False)

    def expand(self, context: StatechartContext) -> None:
        self.inner = CompositeNodeWithChainedChildren(name="inner")
        self._add_child_to_statechart(self.inner)

    def build_artifacts(self, context: StatechartContext) -> NodeArtifacts:
        return NodeArtifacts(observation=sm.Scalar(self.inner.observation_variable))


@dataclass(repr=False, eq=False)
class CompositeNodeCancellingIfChildRunsAfterItsEnd(CompositeNode):
    """
    A composite node that cancels the statechart if one of its children runs after it
    has ended.

    Uses a CancelStatechart node to raise an exception if the child node runs after the
    parent has stopped.
    """

    success_decided_by = SuccessDecider.ITSELF

    ticking1: CountTicks = field(init=False)
    ticking2: CountTicks = field(init=False)
    cancel: CancelStatechart = field(init=False)

    def expand(self, context: StatechartContext) -> None:
        self.ticking1 = CountTicks(name="3ticks", ticks=3)
        self.ticking2 = CountTicks(name="2ticks", ticks=2)
        self.cancel = CancelStatechart(
            name="Cancel_on_tick_after_done",
            exception=NodeAssertionError(reason="Node ticked after template stopped"),
        )

        self._add_children_to_statechart(
            nodes=[
                self.ticking1,
                self.ticking2,
                self.cancel,
            ]
        )
        self.cancel.start_condition = self.ticking1.observes_true

    def build_artifacts(self, context: StatechartContext) -> NodeArtifacts:
        return NodeArtifacts(observation=sm.Scalar(self.ticking2.observation_variable))


@dataclass(repr=False, eq=False)
class CompositeNodeWithChildSucceedingBeforeItStarts(CompositeNode):
    """
    A composite node whose child has its success condition met before it starts.

    node1 waits 1 tick, then starts node 3. node2 fulfills the success condition of node
    3 immediately. node3 should start when node1 is True and transition to RUNNING with
    Observationstate UNKNOWN. On the next tick, node3 should be ended because its end
    condition is already fulfilled by node2.
    """

    success_decided_by = SuccessDecider.OWNER

    node1: CountTicks = field(init=False)
    node2: ConstTrueNode = field(init=False)
    node3: ConstTrueNode = field(init=False)

    def expand(self, context: StatechartContext) -> None:
        self.node1 = CountTicks(ticks=1)
        self.node2 = ConstTrueNode()
        self.node3 = ConstTrueNode()

        self._add_children_to_statechart(nodes=[self.node1, self.node2, self.node3])

        self.node3.start_condition = self.node1.observes_true
        self.node3.success_condition = self.node2.observes_true

    def build_artifacts(self, context: StatechartContext) -> NodeArtifacts:
        return NodeArtifacts(observation=sm.Scalar(self.node3.observation_variable))


@dataclass(repr=False, eq=False)
class CompositeNodeCancellingIfPausedChildResumesAfterItsEnd(CompositeNode):
    """
    A composite node that cancels the statechart if a paused child resumes after it has
    ended.

    Uses a CancelStatechart node to raise an exception if the child node runs after the
    parent has stopped.
    """

    success_decided_by = SuccessDecider.ITSELF

    ticking1: CountTicks = field(init=False)
    ticking2: CountTicks = field(init=False)
    ticking3: CountTicks = field(init=False)
    pulse: Pulse = field(init=False)
    cancel: CancelStatechart = field(init=False)

    def expand(self, context: StatechartContext) -> None:
        self.ticking1 = CountTicks(name="3ticks", ticks=3)
        self.ticking2 = CountTicks(name="trigger_cancel_after_unpause", ticks=4)
        self.ticking3 = CountTicks(name="2ticks", ticks=2)
        self.pulse = Pulse()
        self.cancel = CancelStatechart(
            name="Cancel_on_tick_after_done",
            exception=NodeAssertionError(reason="Node ticked after template stopped"),
        )

        self._add_children_to_statechart(
            nodes=[self.ticking1, self.ticking2, self.ticking3, self.cancel, self.pulse]
        )
        self.pulse.start_condition = self.ticking3.observes_true
        self.ticking2.pause_condition = self.pulse.observes_true
        self.cancel.start_condition = self.ticking2.observes_true

    def build_artifacts(self, context: StatechartContext) -> NodeArtifacts:
        return NodeArtifacts(observation=sm.Scalar(self.ticking1.observation_variable))


@dataclass(repr=False, eq=False)
class CompositeNodeResumingItsPausedChildren(CompositeNode):
    """
    A composite node whose children, paused along with it, resume once it resumes while
    their own pause conditions do not hold.
    """

    success_decided_by = SuccessDecider.ITSELF

    count_ticks1: CountTicks = field(init=False)
    count_ticks2: CountTicks = field(init=False)
    cancel: CancelStatechart = field(init=False)

    def expand(self, context: StatechartContext) -> None:
        self.count_ticks1 = CountTicks(ticks=2)
        self.count_ticks2 = CountTicks(ticks=5)
        self.cancel = CancelStatechart(
            name="check_unpause_failed",
            exception=NodeAssertionError(reason="Node did not unpause correctly"),
        )

        self._add_child_to_statechart(self.count_ticks1)
        self._add_child_to_statechart(Sequence(nodes=[self.count_ticks2, self.cancel]))

    def build_artifacts(self, context: StatechartContext) -> NodeArtifacts:
        """
        :attr:`count_ticks1` is read through its observation, which is what it has
        counted while it runs.
        """
        return NodeArtifacts(
            observation=sm.Scalar(self.count_ticks1.observation_variable)
        )


# %% nodes that differ in what they can be judged by


@dataclass(eq=False, repr=False)
class NodeObservingNothingYet(StatechartNode):
    """
    A node that runs without ever deciding what it observes, so ending it can only
    interrupt it.
    """

    success_decided_by = SuccessDecider.OWNER

    def build_artifacts(self, context: StatechartContext) -> NodeArtifacts:
        return NodeArtifacts(observation=sm.Scalar.const_trinary_unknown())


@dataclass(eq=False, repr=False)
class NodeObservingAPredicate(StatechartNode):
    """
    A node whose observation reads a life cycle predicate, which only a transition
    condition may do.
    """

    success_decided_by = SuccessDecider.OWNER

    watched_node: StatechartNode = field(default=None, kw_only=True)
    """
    The node whose outcome this node tries to observe.
    """

    def build_artifacts(self, context: StatechartContext) -> NodeArtifacts:
        return NodeArtifacts(observation=sm.Scalar(self.watched_node.is_succeeded))


@dataclass(eq=False, repr=False)
class NodeObservingLastObservation(StatechartNode):
    """
    A node whose observation reads the observation another node took most recently.
    """

    success_decided_by = SuccessDecider.OWNER

    watched_node: StatechartNode = field(default=None, kw_only=True)
    """
    The node whose most recent observation this node observes.
    """

    def build_artifacts(self, context: StatechartContext) -> NodeArtifacts:
        return NodeArtifacts(observation=sm.Scalar(self.watched_node.last_observation))


@dataclass(eq=False, repr=False)
class NodeObservingAnObservationPredicate(StatechartNode):
    """
    A node that observes whether another node observed True on the previous
    tick.
    """

    success_decided_by = SuccessDecider.OWNER

    watched_node: StatechartNode = field(default=None, kw_only=True)
    """
    The node whose observation this node asks about.
    """

    def build_artifacts(self, context: StatechartContext) -> NodeArtifacts:
        return NodeArtifacts(observation=sm.Scalar(self.watched_node.observes_true))


@dataclass(eq=False, repr=False)
class NodeObservingTheOppositeOfAnObservationPredicate(StatechartNode):
    """
    A node that observes True while another node does not observe True.
    """

    success_decided_by = SuccessDecider.OWNER

    watched_node: StatechartNode = field(default=None, kw_only=True)
    """
    The node whose observation this node contradicts.
    """

    def build_artifacts(self, context: StatechartContext) -> NodeArtifacts:
        return NodeArtifacts(
            observation=sm.logic_not(sm.Scalar(self.watched_node.observes_true))
        )


@dataclass(eq=False, repr=False)
class NodeObservingAFixedValue(StatechartNode):
    """
    A node that observes the same value on every tick and leaves ending it to its
    owner.
    """

    success_decided_by = SuccessDecider.OWNER

    observation: ObservationStateValues = field(kw_only=True)
    """
    What this node observes on every tick.
    """

    def build_artifacts(self, context: StatechartContext) -> NodeArtifacts:
        return NodeArtifacts(observation=sm.Scalar(float(self.observation)))


@dataclass(eq=False, repr=False)
class NodeFailingOnObservingFalse(NodeObservingAFixedValue):
    """
    A node that fails itself once it observes False and leaves succeeding to its owner.
    """

    fails_when_observing_false = True


@dataclass(eq=False, repr=False)
class NodeSucceedingOnObservingTrue(NodeObservingAFixedValue):
    """
    A node that succeeds once it observes True and declares no failure of its own.
    """

    success_decided_by = SuccessDecider.ITSELF


@dataclass(eq=False, repr=False)
class NodeDeclaringNoSuccessDecider(StatechartNode):
    """
    A node class that leaves open who decides that it succeeded.
    """


@dataclass(repr=False, eq=False)
class NodeDeclaringItsOwnFailure(CompositeNode):
    """
    A node short of its goal that declares it cannot continue, which is what a node may
    decide about itself where succeeding is left to its owner.

    Being a composite node is what gives it an :meth:`expand` hook to declare
    the failure in; it runs no children of its own.
    """

    success_decided_by = SuccessDecider.OWNER

    def expand(self, context: StatechartContext) -> None:
        self.fail_condition = sm.Scalar.const_true()

    def build_artifacts(self, context: StatechartContext) -> NodeArtifacts:
        return NodeArtifacts(observation=sm.Scalar.const_false())


# %% goals that end their child


@dataclass(repr=False, eq=False)
class CompositeNodeCuttingOffItsChildAtItsGoal(CompositeNode):
    """
    Composite node whose child has reached its goal but is never ended on its
    own terms, so the child is only ever taken down by this node ending.
    """

    success_decided_by = SuccessDecider.OWNER

    child: ConstTrueNode = field(init=False)
    """
    The child that sits at its goal until it is cut off.
    """

    def expand(self, context: StatechartContext) -> None:
        self.child = ConstTrueNode()
        self._add_child_to_statechart(self.child)

    def build_artifacts(self, context: StatechartContext) -> NodeArtifacts:
        return NodeArtifacts(observation=sm.Scalar.const_true())


@dataclass(repr=False, eq=False)
class CompositeNodeCuttingOffItsChild(CompositeNode):
    """
    Composite node whose child is short of its goal and is never ended on its
    own terms, so the child is only ever taken down by this node ending.
    """

    success_decided_by = SuccessDecider.OWNER

    child: ConstFalseNode = field(init=False)
    """
    The child that keeps running until it is cut off.
    """

    def expand(self, context: StatechartContext) -> None:
        self.child = ConstFalseNode()
        self._add_child_to_statechart(self.child)

    def build_artifacts(self, context: StatechartContext) -> NodeArtifacts:
        return NodeArtifacts(observation=sm.Scalar.const_true())


@dataclass(repr=False, eq=False)
class CompositeNodeWithChildInterruptedBySibling(CompositeNode):
    """
    Composite node whose child is interrupted by a sibling on the first tick,
    so that a caller ending this node on that same tick makes the two ways of being
    interrupted compete.
    """

    success_decided_by = SuccessDecider.OWNER

    trigger: ConstTrueNode = field(init=False)
    """
    Turns true on the first tick, which is what interrupts the child.
    """

    child: ConstFalseNode = field(init=False)
    """
    The child that is interrupted while its observation is false.
    """

    def expand(self, context: StatechartContext) -> None:
        self.trigger = ConstTrueNode()
        self.child = ConstFalseNode()
        self._add_children_to_statechart(nodes=[self.trigger, self.child])
        self.child.interrupt_condition = self.trigger.observes_true

    def build_artifacts(self, context: StatechartContext) -> NodeArtifacts:
        return NodeArtifacts(observation=sm.Scalar.const_true())


@dataclass(repr=False, eq=False)
class CompositeNodeWithChildFailingOnItsOwn(CompositeNode):
    """
    Composite node whose child declares its own failure on the first tick, so
    that a caller ending this node on that same tick makes the child's own outcome
    compete with being cut off.
    """

    success_decided_by = SuccessDecider.OWNER

    trigger: ConstTrueNode = field(init=False)
    """
    Turns true on the first tick, which is what makes the child declare its failure.
    """

    child: ConstFalseNode = field(init=False)
    """
    The child that gives up while its observation is false.
    """

    def expand(self, context: StatechartContext) -> None:
        self.trigger = ConstTrueNode()
        self.child = ConstFalseNode()
        self._add_children_to_statechart(nodes=[self.trigger, self.child])
        self.child.fail_condition = self.trigger.observes_true

    def build_artifacts(self, context: StatechartContext) -> NodeArtifacts:
        return NodeArtifacts(observation=sm.Scalar.const_true())


@dataclass(repr=False, eq=False)
class CompositeNodeWithChildSucceedingOnItsOwn(CompositeNode):
    """
    Composite node whose child declares its own success on the first tick, so
    that a caller ending this node on that same tick makes the child's own outcome
    compete with being cut off.
    """

    success_decided_by = SuccessDecider.OWNER

    trigger: ConstTrueNode = field(init=False)
    """
    Turns true on the first tick, which is what makes the child declare its success.
    """

    child: ConstFalseNode = field(init=False)
    """
    The child that declares its success while its observation is false.
    """

    def expand(self, context: StatechartContext) -> None:
        self.trigger = ConstTrueNode()
        self.child = ConstFalseNode()
        self._add_children_to_statechart(nodes=[self.trigger, self.child])
        self.child.success_condition = self.trigger.observes_true

    def build_artifacts(self, context: StatechartContext) -> NodeArtifacts:
        return NodeArtifacts(observation=sm.Scalar.const_true())


@dataclass(repr=False, eq=False)
class CompositeNodeWithChildStartingLate(CompositeNode):
    """
    Composite node whose child waits for a delay before it starts, so the
    child's start is decided while this node is already running and its ending
    conditions have a settled value.
    """

    success_decided_by = SuccessDecider.OWNER

    delay_in_ticks: int = field(default=2, kw_only=True)
    """
    How many ticks pass before the child's start condition turns true.
    """

    child: ConstFalseNode = field(init=False)
    """
    The child whose start is being observed.
    """

    def expand(self, context: StatechartContext) -> None:
        delay = CountTicks(ticks=self.delay_in_ticks)
        self.child = ConstFalseNode()
        self._add_children_to_statechart(nodes=[delay, self.child])
        self.child.start_condition = delay.observes_true

    def build_artifacts(self, context: StatechartContext) -> NodeArtifacts:
        return NodeArtifacts(observation=sm.Scalar.const_false())


@dataclass(repr=False, eq=False)
class CompositeNodeCuttingOffItsUndecidedChild(CompositeNode):
    """
    Composite node whose child never decides what it observes, so this node
    ending is the only thing that ever ends it.
    """

    success_decided_by = SuccessDecider.OWNER

    child: NodeObservingNothingYet = field(init=False)
    """
    The child that observes nothing until it is ended.
    """

    def expand(self, context: StatechartContext) -> None:
        self.child = NodeObservingNothingYet()
        self._add_child_to_statechart(self.child)

    def build_artifacts(self, context: StatechartContext) -> NodeArtifacts:
        return NodeArtifacts(observation=sm.Scalar.const_true())


@dataclass(repr=False, eq=False)
class CompositeNodeCuttingOffItsGrandchild(CompositeNode):
    """
    Composite node holding another composite node, so that ending
    it reaches a node more than one level below it.
    """

    success_decided_by = SuccessDecider.OWNER

    inner_node: CompositeNodeCuttingOffItsChild = field(init=False)
    """
    The node between this one and the grandchild.
    """

    def expand(self, context: StatechartContext) -> None:
        self.inner_node = CompositeNodeCuttingOffItsChild()
        self._add_child_to_statechart(self.inner_node)

    @property
    def grandchild(self) -> ConstFalseNode:
        """
        :return: The node two levels below this node, which is short of its goal until
            this node ends.
        """
        return self.inner_node.child

    def build_artifacts(self, context: StatechartContext) -> NodeArtifacts:
        return NodeArtifacts(observation=sm.Scalar.const_true())


# %% nodes that record what the statechart does to them


class LifeCycleCallback(Enum):
    """
    A callback the statechart runs on a node when its life cycle state changes.
    """

    START = auto()
    """
    :meth:`~cramph.node.StatechartNode.on_start`.
    """

    PAUSE = auto()
    """
    :meth:`~cramph.node.StatechartNode.on_pause`.
    """

    UNPAUSE = auto()
    """
    :meth:`~cramph.node.StatechartNode.on_unpause`.
    """

    END = auto()
    """
    :meth:`~cramph.node.StatechartNode.on_end`.
    """

    RESET = auto()
    """
    :meth:`~cramph.node.StatechartNode.on_reset`.
    """


@dataclass(eq=False, repr=False)
class NodeRecordingItsCallbacks(StatechartNode):
    """
    A node that never reaches its goal and records every life cycle callback run on it.
    """

    success_decided_by = SuccessDecider.OWNER

    callbacks: List[LifeCycleCallback] = field(default_factory=list, init=False)
    """
    The callbacks run on this node since :meth:`take_callbacks` was last called, in the
    order they ran.
    """

    def build_artifacts(self, context: StatechartContext) -> NodeArtifacts:
        return NodeArtifacts(observation=sm.Scalar.const_false())

    def take_callbacks(self) -> List[LifeCycleCallback]:
        """
        :return: The callbacks recorded so far, which are forgotten afterwards.
        """
        callbacks = self.callbacks
        self.callbacks = []
        return callbacks

    def on_start(self, context: StatechartContext):
        self.callbacks.append(LifeCycleCallback.START)

    def on_pause(self, context: StatechartContext):
        self.callbacks.append(LifeCycleCallback.PAUSE)

    def on_unpause(self, context: StatechartContext):
        self.callbacks.append(LifeCycleCallback.UNPAUSE)

    def on_end(self, context: StatechartContext):
        self.callbacks.append(LifeCycleCallback.END)

    def on_reset(self, context: StatechartContext):
        self.callbacks.append(LifeCycleCallback.RESET)


@dataclass(eq=False, repr=False)
class NodeObservingTrueOnlyOnTick(StatechartNode):
    """
    A node whose observation expression is False but whose
    :meth:`~cramph.node.StatechartNode.on_tick` overrides
    it with True, counting how often it is ticked.
    """

    success_decided_by = SuccessDecider.OWNER

    on_tick_calls: int = field(default=0, init=False)
    """
    How often :meth:`on_tick` was called.
    """

    def build_artifacts(self, context: StatechartContext) -> NodeArtifacts:
        return NodeArtifacts(observation=sm.Scalar.const_false())

    def on_tick(self, context: StatechartContext) -> ObservationStateValues:
        self.on_tick_calls += 1
        return ObservationStateValues.TRUE


@dataclass(eq=False, repr=False)
class NodeWritingAVariableOnStart(StatechartNode):
    """
    A node that sets a float variable to True when it starts, so what its start callback
    wrote can be observed by another node.
    """

    success_decided_by = SuccessDecider.OWNER

    variable: FloatVariable = field(init=False)
    """
    The variable written when this node starts, False until then.
    """

    def build_artifacts(self, context: StatechartContext) -> NodeArtifacts:
        self.variable = FloatVariable(f"{self.name}/written_on_start")
        context.float_variable_data.register_expression(self.variable)
        return NodeArtifacts(observation=sm.Scalar.const_trinary_unknown())

    def on_start(self, context: StatechartContext):
        context.float_variable_data.set_value(
            self.variable, float(ObservationStateValues.TRUE)
        )


@dataclass(eq=False, repr=False)
class NodeObservingAWrittenVariable(StatechartNode):
    """
    A node that observes True once another node's start callback wrote its variable.
    """

    success_decided_by = SuccessDecider.OWNER

    writer: NodeWritingAVariableOnStart = field(kw_only=True)
    """
    The node whose written variable this node observes.
    """

    @property
    def prerequisite_nodes(self) -> List[StatechartNode]:
        """
        :return: :attr:`writer`, which creates the variable this node reads.
        """
        return [self.writer]

    def build_artifacts(self, context: StatechartContext) -> NodeArtifacts:
        return NodeArtifacts(
            observation=sm.if_eq(
                self.writer.variable,
                float(ObservationStateValues.TRUE),
                sm.Scalar.const_true(),
                sm.Scalar.const_false(),
            )
        )


@dataclass(repr=False, eq=False)
class CompositeNodeObservingItsSecondChildRun(CompositeNode):
    """
    Composite node that observes True once its second child is running, which
    starts once its first child succeeded, so the second child is started and then cut off
    by this node ending.
    """

    success_decided_by = SuccessDecider.OWNER

    first: ConstTrueNode = field(init=False)
    """
    The child that succeeds as soon as it observes its goal.
    """

    second: NodeRecordingItsCallbacks = field(init=False)
    """
    The child that starts once :attr:`first` succeeded and is cut off by this node.
    """

    def expand(self, context: StatechartContext) -> None:
        self.first = ConstTrueNode()
        self.second = NodeRecordingItsCallbacks()
        self._add_children_to_statechart(nodes=[self.first, self.second])
        self.first.success_condition = self.first.observes_true
        self.second.start_condition = self.first.is_succeeded

    def build_artifacts(self, context: StatechartContext) -> NodeArtifacts:
        return NodeArtifacts(
            observation=sm.if_eq(
                self.second.life_cycle_variable,
                int(LifeCycleValues.RUNNING),
                sm.Scalar.const_true(),
                sm.Scalar.const_false(),
            )
        )


@dataclass(repr=False, eq=False)
class CompositeNodeWithARecordingChild(CompositeNode):
    """
    Composite node holding one child that records its callbacks and would start
    whenever it may.
    """

    success_decided_by = SuccessDecider.OWNER

    child: NodeRecordingItsCallbacks = field(init=False)
    """
    The child whose callbacks are recorded.
    """

    def expand(self, context: StatechartContext) -> None:
        self.child = NodeRecordingItsCallbacks()
        self._add_child_to_statechart(self.child)


@dataclass(repr=False, eq=False)
class CompositeNodeObservingItsCancellingChildRun(CompositeNode):
    """
    Composite node that observes True once its :class:`CancelStatechart` child is
    running, which starts once its other child observes True, so the cancelling node is
    started and then cut off by this node ending.
    """

    success_decided_by = SuccessDecider.OWNER

    trigger: ConstTrueNode = field(init=False)
    """
    The child whose observation starts :attr:`cancel`.
    """

    cancel: CancelStatechart = field(init=False)
    """
    The child that is cut off right after starting.
    """

    def expand(self, context: StatechartContext) -> None:
        self.trigger = ConstTrueNode()
        self.cancel = CancelStatechart(
            exception=NodeAssertionError(reason="cancelled right after starting")
        )
        self._add_children_to_statechart(nodes=[self.trigger, self.cancel])
        self.cancel.start_condition = self.trigger.observes_true

    def build_artifacts(self, context: StatechartContext) -> NodeArtifacts:
        return NodeArtifacts(
            observation=sm.if_eq(
                self.cancel.life_cycle_variable,
                int(LifeCycleValues.RUNNING),
                sm.Scalar.const_true(),
                sm.Scalar.const_false(),
            )
        )
