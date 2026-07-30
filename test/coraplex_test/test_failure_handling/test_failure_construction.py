import pytest
from semantic_digital_twin.spatial_types.spatial_types import Pose

from coraplex.datastructures.dataclasses import Context
from coraplex.exceptions import ConditionNotSatisfied
from coraplex.language import CodeNode
from coraplex.plans.condition_nodes import ConditionNode
from coraplex.plans.executables import ConditionExecutable
from coraplex.plans.factories import code, execute_single, sequential, try_in_order
from coraplex.plans.failures import AllChildrenFailed, PlanFailure
from coraplex.plans.plan_node import ActionNode
from coraplex.robot_plans.actions.core.navigation import NavigateAction

# %% refined_from provenance chain


def test_refined_from_defaults_to_none():
    failure = PlanFailure(node=None)

    assert failure.refined_from is None


def test_refined_from_links_to_the_failure_it_was_refined_from():
    original = PlanFailure(node=None)
    refined = PlanFailure(node=None, refined_from=original)

    assert refined.refined_from is original


def test_refined_from_chain_can_be_walked_across_multiple_links():
    first = PlanFailure(node=None)
    second = PlanFailure(node=None, refined_from=first)
    third = PlanFailure(node=None, refined_from=second)

    assert third.refined_from is second
    assert third.refined_from.refined_from is first


# %% resolution bookkeeping


def test_resolution_defaults_to_none():
    failure = PlanFailure(node=None)

    assert failure.resolution is None


# %% action_node resolution


def test_action_node_returns_the_node_itself_when_it_is_an_action_node():
    action_node = execute_single(NavigateAction(target_location=Pose()))

    failure = PlanFailure(node=action_node)

    assert failure.action_node is action_node


def test_action_node_finds_the_nearest_ancestor_action_node():
    action_node = execute_single(NavigateAction(target_location=Pose()))
    child_node = CodeNode(code=lambda: None)
    action_node.add_child(child_node)

    failure = PlanFailure(node=child_node)

    assert isinstance(action_node, ActionNode)
    assert failure.action_node is action_node


def test_action_node_is_none_when_no_action_node_is_in_the_path():
    root_node = code(lambda: None)

    failure = PlanFailure(node=root_node)

    assert failure.action_node is None


# %% context resolution


def test_context_returns_the_failing_plan_context(immutable_model_world):
    world, robot, context = immutable_model_world
    root_node = sequential([NavigateAction(target_location=Pose())], context)

    failure = PlanFailure(node=root_node)

    assert failure.context is context


# %% raise-site construction


def test_a_try_in_order_of_failing_children_raises_a_well_formed_all_children_failed():
    first_failing = CodeNode(code=lambda: None)
    second_failing = CodeNode(code=lambda: None)

    def fail_first():
        raise PlanFailure(node=first_failing)

    def fail_second():
        raise PlanFailure(node=second_failing)

    first_failing.code = fail_first
    second_failing.code = fail_second
    root = try_in_order(
        [first_failing, second_failing], Context(world=None, robot=None)
    )

    with pytest.raises(AllChildrenFailed) as raised:
        root.perform()

    assert raised.value.node is root
    assert raised.value.language_node is root


def test_an_unsatisfied_condition_raises_a_well_formed_condition_not_satisfied():
    action_node = execute_single(NavigateAction(target_location=Pose()))
    condition_node = ConditionNode(
        condition=False, pre_condition=True, action_node=action_node
    )
    executable = ConditionExecutable(condition_node=condition_node, context=None)

    with pytest.raises(ConditionNotSatisfied) as raised:
        executable.execute()

    assert raised.value.node is condition_node
    assert raised.value.action is NavigateAction
    assert raised.value.pre_condition is True
