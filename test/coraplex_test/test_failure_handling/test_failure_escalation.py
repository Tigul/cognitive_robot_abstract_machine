from dataclasses import dataclass

import pytest

from coraplex.datastructures.dataclasses import Context
from coraplex.datastructures.enums import TaskStatus
from coraplex.failure_handling.failure_handler import FailureHandler
from coraplex.failure_handling.failure_handling_strategy import (
    FailureHandlingStrategy,
    FailureResolution,
    Propagate,
    RetryNode,
)
from coraplex.language import CodeNode, SequentialNode
from coraplex.plans.factories import sequential
from coraplex.plans.failures import PlanFailure

from .test_failure_handler import HandledFailure
from .test_perform_integration import ConsultationCountingHandler, context_with

# %% stub strategies


@dataclass
class SequenceRetryingStrategy(FailureHandlingStrategy):
    """
    Retries the nearest enclosing sequence, which never runs in a perform frame of its
    own.
    """

    def resolve(self, failure: PlanFailure) -> FailureResolution:
        for ancestor in failure.node.path:
            if isinstance(ancestor, SequentialNode):
                return RetryNode(failure=failure, target_node=ancestor)
        return Propagate(failure=failure)


# %% helpers


def sequence_over_a_leaf(context: Context) -> tuple[SequentialNode, CodeNode]:
    """
    :param context: The context the plan is built in.
    :return: A sequence and the leaf below it, neither of which the other performs in a
        frame of its own.
    """
    leaf = CodeNode(code=lambda: None)
    root = sequential([leaf], context)
    return root, leaf


# %% escalation along the plan tree


def test_escalating_hands_the_failure_to_the_parent_node():
    handler = ConsultationCountingHandler()
    root, leaf = sequence_over_a_leaf(context_with(handler))
    failure = HandledFailure(node=leaf)

    with pytest.raises(HandledFailure):
        leaf.escalate(failure)

    assert handler.consultations == 1


def test_escalating_at_the_root_raises_the_failure():
    root, leaf = sequence_over_a_leaf(Context(world=None, robot=None))
    failure = PlanFailure(node=root)

    with pytest.raises(PlanFailure) as raised:
        root.escalate(failure)

    assert raised.value is failure


def test_a_propagated_failure_is_recorded_along_the_whole_chain():
    root, leaf = sequence_over_a_leaf(Context(world=None, robot=None))
    failure = PlanFailure(node=leaf)

    with pytest.raises(PlanFailure):
        leaf.handle_failure(failure)

    assert leaf.status == TaskStatus.FAILED
    assert root.status == TaskStatus.FAILED
    assert root.reason is failure


# %% targets that never get a perform frame


def test_a_resolution_reaches_a_target_that_has_no_perform_frame():
    """
    A sequence runs its children inside one merged execution list, so it never gets a
    perform frame.

    Escalation walks the plan tree instead of the call stack, so the resolution still
    reaches it.
    """
    handler = FailureHandler(strategies=[SequenceRetryingStrategy()])
    root, leaf = sequence_over_a_leaf(context_with(handler))
    failure = HandledFailure(node=leaf)

    leaf.handle_failure(failure)

    assert failure.resolution is None
    assert root.status != TaskStatus.FAILED


def test_the_handler_is_consulted_once_while_a_failure_escalates():
    handler = ConsultationCountingHandler()
    root, leaf = sequence_over_a_leaf(context_with(handler))
    failure = HandledFailure(node=leaf)

    with pytest.raises(HandledFailure):
        leaf.handle_failure(failure)

    assert handler.consultations == 1


# %% perform routes to the node that raised


def test_perform_hands_a_failure_to_the_node_that_raised_it():
    handler = FailureHandler(strategies=[SequenceRetryingStrategy()])
    context = context_with(handler)
    executions = []
    leaf = CodeNode(code=lambda: None)
    root = sequential([leaf], context)

    def fail_once():
        executions.append(len(executions))
        if len(executions) < 2:
            raise HandledFailure(node=leaf)

    leaf.code = fail_once

    root.perform()

    assert executions == [0, 1]
    assert root.status == TaskStatus.SUCCEEDED
