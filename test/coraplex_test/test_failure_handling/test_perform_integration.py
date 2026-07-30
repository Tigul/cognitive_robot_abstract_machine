from dataclasses import dataclass, field

import pytest
from semantic_digital_twin.spatial_types.spatial_types import Pose

from coraplex.datastructures.dataclasses import Context
from coraplex.datastructures.enums import TaskStatus
from coraplex.failure_handling.failure_handler import FailureHandler
from coraplex.failure_handling.failure_handling_strategy import (
    FailureHandlingStrategy,
    FailureResolution,
    Propagate,
    RetryNode,
)
from coraplex.failure_handling.failure_refiner import FailureRefiner
from coraplex.language import CodeNode, SequentialNode
from coraplex.plans.factories import code, execute_single
from coraplex.plans.failures import PlanFailure
from coraplex.robot_plans.actions.core.navigation import NavigateAction

from .test_failure_handler import (
    ExhaustibleRetryStrategy,
    HandledFailure,
    RefinedHandledFailure,
    RefiningDetector,
    RetryingStrategy,
)

# %% stub handlers and strategies


@dataclass
class ConsultationCountingHandler(FailureHandler):
    """
    Counts how often the performing frames consult the handler.
    """

    consultations: int = field(default=0, init=False)
    """
    How many failures this handler was consulted for.
    """

    def handle(self, failure: PlanFailure) -> FailureResolution:
        self.consultations += 1
        return super().handle(failure)


@dataclass
class SequenceRetryingStrategy(FailureHandlingStrategy):
    """
    Retries the nearest enclosing sequence frame instead of the failing frame.
    """

    def resolve(self, failure: PlanFailure) -> FailureResolution:
        for ancestor in failure.node.path:
            if isinstance(ancestor, SequentialNode):
                return RetryNode(failure=failure, target_node=ancestor)
        return Propagate(failure=failure)


# %% helpers


def context_with(handler: FailureHandler) -> Context:
    return Context(world=None, robot=None, failure_handler=handler)


# %% baseline handler on the context


def test_a_context_carries_a_baseline_handler_by_default():
    context = Context(world=None, robot=None)

    assert isinstance(context.failure_handler, FailureHandler)


def test_the_baseline_handler_propagates_a_plain_failure_like_today():
    node = code(lambda: None, context=Context(world=None, robot=None))
    failure = PlanFailure(node=node)

    def raise_failure():
        raise failure

    node.code = raise_failure

    with pytest.raises(PlanFailure) as raised:
        node.perform()

    assert raised.value is failure
    assert node.status == TaskStatus.FAILED
    assert node.reason is failure
    assert isinstance(failure.resolution, Propagate)
    assert node.end_time is not None


# %% refinement at the chokepoint


def test_a_configured_detector_refines_the_raised_failure():
    handler = FailureHandler(
        refiner=FailureRefiner(failure_detectors=[RefiningDetector()])
    )
    action_node = execute_single(
        NavigateAction(target_location=Pose()), context=context_with(handler)
    )
    failing_child = CodeNode(code=lambda: None)
    action_node.add_child(failing_child)
    original = HandledFailure(node=failing_child)

    def raise_original():
        raise original

    failing_child.code = raise_original

    with pytest.raises(RefinedHandledFailure) as raised:
        failing_child.perform()

    assert raised.value.refined_from is original
    assert failing_child.status == TaskStatus.FAILED
    assert failing_child.reason is raised.value


# %% retrying frames


def test_a_retrying_strategy_reruns_the_frame_until_success():
    handler = FailureHandler(strategies=[RetryingStrategy()])
    node = code(lambda: None, context=context_with(handler))
    executions = []

    def fail_twice():
        executions.append(len(executions))
        if len(executions) < 3:
            raise HandledFailure(node=node)

    node.code = fail_twice

    node.perform()

    assert len(executions) == 3
    assert node.status == TaskStatus.SUCCEEDED


def test_exhausted_retries_propagate():
    handler = FailureHandler(strategies=[ExhaustibleRetryStrategy(maximum_attempts=2)])
    node = code(lambda: None, context=context_with(handler))
    executions = []

    def always_fail():
        executions.append(len(executions))
        raise HandledFailure(node=node)

    node.code = always_fail

    with pytest.raises(HandledFailure):
        node.perform()

    assert len(executions) == 3
    assert node.status == TaskStatus.FAILED


def test_a_targeted_retry_reruns_the_sequence_frame_not_the_outer_frame():
    handler = FailureHandler(strategies=[SequenceRetryingStrategy()])
    outer = code(lambda: None, context=context_with(handler))
    sequence = SequentialNode()
    outer.add_child(sequence)

    executions = []
    first = CodeNode(code=lambda: executions.append("first"))
    failing = CodeNode(code=lambda: None)
    sequence.add_child(first)
    sequence.add_child(failing)

    def fail_once():
        executions.append("failing")
        if executions.count("failing") < 2:
            raise HandledFailure(node=failing)

    failing.code = fail_once

    outer_runs = []

    def run_sequence():
        outer_runs.append(True)
        sequence.perform()

    outer.code = run_sequence

    outer.perform()

    assert outer_runs == [True]
    assert executions == ["first", "failing", "first", "failing"]
    assert sequence.status == TaskStatus.SUCCEEDED
    assert outer.status == TaskStatus.SUCCEEDED


# %% handler consultation across nested frames


def test_the_handler_is_consulted_once_per_failure_object():
    handler = ConsultationCountingHandler()
    outer = code(lambda: None, context=context_with(handler))
    inner = CodeNode(code=lambda: None)
    outer.add_child(inner)

    def always_fail():
        raise HandledFailure(node=inner)

    inner.code = always_fail
    outer.code = inner.perform

    with pytest.raises(HandledFailure):
        outer.perform()

    assert handler.consultations == 1
    assert inner.status == TaskStatus.FAILED
    assert outer.status == TaskStatus.FAILED
    assert inner.reason is outer.reason


# %% interruption guard placement


def test_an_interrupted_ancestor_short_circuits_before_execution():
    parent = code(lambda: None, context=Context(world=None, robot=None))
    executions = []
    child = CodeNode(code=lambda: executions.append(True))
    parent.add_child(child)
    parent.interrupt()

    child.perform()

    assert child.status == TaskStatus.INTERRUPTED
    assert executions == []
