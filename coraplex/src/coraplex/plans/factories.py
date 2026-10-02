from __future__ import annotations

from typing import Callable

from typing_extensions import List, Optional, TYPE_CHECKING, Union, assert_never

from coraplex.plans.failures import PlanCancelled, RepetitionsExhausted
from coraplex.plans.function_call import FunctionCall
from coraplex.plans.plan import Plan
from coraplex.plans.underspecified import UnderspecifiedNode
from cramph.composites import (
    Attempt,
    CancelledWhenTrue,
    Parallel,
    PausedUntilTrue,
    PausedWhileTrue,
    RepeatUntil,
    Sequence,
    TryAll,
    TryInOrder,
)
from cramph.monitors import CountNodeResets
from cramph.node import StatechartNode
from giskardpy.motion_statechart.goals.templates import RepeatOnStall
from krrood.entity_query_language.query.match import Match

if TYPE_CHECKING:
    from coraplex.datastructures.dataclasses import Context

ActionLike = Union[Plan, Match, StatechartNode]
"""
Anything a plan can be built from: a plan, whose root is taken over, an underspecified
statement, which is grounded while the plan runs, or a statechart node, such as an
action or a motion.
"""


def as_statechart_node(action_like: ActionLike) -> StatechartNode:
    """
    :param action_like: What a step of a plan is described by.
    :return: The statechart node running that step.
    """
    if isinstance(action_like, Plan):
        return action_like.root
    if isinstance(action_like, Match):
        return UnderspecifiedNode(statement=action_like)
    if isinstance(action_like, StatechartNode):
        return action_like
    assert_never(action_like)


def _as_statechart_nodes(children: List[ActionLike]) -> List[StatechartNode]:
    """
    :return: The statechart node of every step in `children`, in order.
    """
    return [as_statechart_node(child) for child in children]


def execute_single(
    action_like: ActionLike,
    context: Optional[Context] = None,
) -> Plan:
    """
    Executes the given action like on its own.

    :param action_like: The action like to execute
    :param context: Plan context to pass to the action
    :return: The plan running the action like.
    """
    return Plan(as_statechart_node(action_like), context=context)


def sequential(
    children: List[ActionLike],
    context: Optional[Context] = None,
) -> Plan:
    """
    Executes the given actions or motions in order, one after the other.

    If any of them fails, the plan fails.

    :param children: The actions to be executed
    :param context: The plan context to pass to the actions
    :return: The plan running the actions in order.
    """
    return Plan(Sequence(_as_statechart_nodes(children)), context=context)


def parallel(
    children: List[ActionLike],
    context: Optional[Context] = None,
) -> Plan:
    """
    Executes the given actions in parallel as far as possible.

    .. warning:: If there are model changes during the execution unexpected behaviour can occur, so avoid using manipulation actions such as Pick and Place in this

    :param children: Actions or motions to be executed in parallel
    :param context: The plan context to pass to the actions
    :return: The plan running the actions at the same time.
    """
    return Plan(Parallel(_as_statechart_nodes(children)), context=context)


def try_in_order(
    children: List[ActionLike],
    context: Optional[Context] = None,
) -> Plan:
    """
    Executes the given actions or motions one after another, if one fails the next one
    is executed until one succeeds or all fail.

    :param children: The actions or motions to be executed
    :param context: The plan context that should be passed to the build plan
    :return: The plan trying the actions in order.
    """
    return Plan(TryInOrder(_as_statechart_nodes(children)), context=context)


def try_all(
    children: List[ActionLike],
    context: Optional[Context] = None,
) -> Plan:
    """
    Tries all given actions or motions, similar to try_in_order but executes them in
    parallel.

    Succeeded if at least one child succeeds.

    :param children: Actions or motions to be executed
    :param context: the plan context that should be passed to the children
    :return: The plan trying the actions at the same time.
    """
    return Plan(TryAll(_as_statechart_nodes(children)), context=context)


def pause_while(
    children: List[ActionLike],
    monitor: StatechartNode,
    context: Optional[Context] = None,
) -> Plan:
    """
    Hold `children` for as long as `monitor` observes True.

    :param children: Actions or motions that should be under the monitor, meaning they
        are paused when the monitor is true
    :param monitor: The node observed while the children run.
    :param context: The context of the plan to be passed to the children
    """
    return Plan(
        PausedWhileTrue(
            monitor=monitor, monitored_node=Sequence(_as_statechart_nodes(children))
        ),
        context=context,
    )


def pause_until(
    children: List[ActionLike],
    monitor: StatechartNode,
    context: Optional[Context] = None,
) -> Plan:
    """
    Hold `children` until `monitor` observes True.

    .. warning:: A monitor that never turns True holds the children forever, so the
        plan runs out of ticks and fails with
        :class:`~coraplex.exceptions.MotionDidNotFinish`. Use :func:`cancel_when` to
        give up on the plan instead.

    :param children: Actions or motions that should be paused until the monitor turns
        true
    :param monitor: The node observed while the children run.
    :param context: The plan context to be passed to the children
    """
    return Plan(
        PausedUntilTrue(
            monitor=monitor, monitored_node=Sequence(_as_statechart_nodes(children))
        ),
        context=context,
    )


def cancel_when(
    children: List[ActionLike],
    monitor: StatechartNode,
    context: Optional[Context] = None,
) -> Plan:
    """
    Stop `children` once `monitor` observes True and cancel the rest of the plan,
    raising :class:`~coraplex.plans.failures.PlanCancelled`, because the state the rest
    of the plan assumed no longer holds.

    :param monitor: The node observed while the children run.
    :param children: Actions or motions that should be cancelled when the monitor turns
        true
    :param context: The plan context to be passed to the children
    """
    return Plan(
        CancelledWhenTrue(
            monitor=monitor,
            monitored_node=Sequence(_as_statechart_nodes(children)),
            exception=PlanCancelled(monitor=monitor),
        ),
        context=context,
    )


def repeat(
    children: List[ActionLike],
    maximum_repetitions: int,
    context: Optional[Context] = None,
    repeat_template: Callable[..., RepeatUntil] = RepeatOnStall,
    failure_monitor: Optional[StatechartNode] = None,
) -> Plan:
    """
    Attempt `children` until they succeed, at most `maximum_repetitions` times.

    Running out of attempts is a failure, raising
    :class:`~coraplex.plans.failures.RepetitionsExhausted`.

    .. note:: The default template treats an attempt as failed once it stops making
        progress, which needs at least one converging task among the children.

    :param children: Actions or motions that should be repeated.
    :param maximum_repetitions: How many attempts before the repeating is given up on.
    :param context: The plan context to be passed to the children
    :param repeat_template: Builds the goal that runs the children over and over, for a
        decision derived from the children, such as a stall. It is called with the
        children's goal, the attempt counter and the failure reported once the attempts
        run out, so a template needing more configuration is passed pre-configured, for
        instance ``partial(RepeatOnStall, timeout=timedelta(seconds=1))``.
    :param failure_monitor: Node whose True observation means an attempt failed, for a
        decision that stands on its own, such as a force spike. It turns the children's
        goal into an attempt that gives up when this monitor fires.
    """
    children_goal = Sequence(_as_statechart_nodes(children))
    counter = CountNodeResets(
        name="repeat/attempts", node=children_goal, target=maximum_repetitions
    )
    attempted_children = (
        Attempt(
            name="repeat/attempt",
            task=children_goal,
            failure_monitors=[failure_monitor],
        )
        if failure_monitor is not None
        else children_goal
    )
    return Plan(
        repeat_template(
            name="repeat",
            task=attempted_children,
            stop_retry_monitor=counter,
            exception=RepetitionsExhausted(
                repeated_node=children_goal, maximum_repetitions=maximum_repetitions
            ),
        ),
        context=context,
    )


def code(function: Callable, context: Optional[Context] = None) -> Plan:
    """
    :param function: The function to call as a step of a plan, in a thread of its own.
    :param context: The plan context.
    :return: The plan calling the function.
    """
    return Plan(FunctionCall(function=function), context=context)
