"""
Coverage for how the action console finds the ways to build actions.
"""

from __future__ import annotations

import inspect
from dataclasses import dataclass

from typing_extensions import Self

from coraplex.robot_plans.actions.base import Action
from coraplex.robot_plans.actions.composite.carrying import CarryAction
from coraplex.robot_plans.actions.composite.transporting import TransportAction
from coraplex.robot_plans.actions.core.container import (
    CloseAction,
    MovesContainerByItsHandle,
    OpenAction,
)
from coraplex.robot_plans.actions.core.navigation import NavigateAction
from cramph.context import StatechartContext
from krrood.utils import recursive_subclasses
from semantic_digital_twin.spatial_types.spatial_types import Pose

# %% constructors


def test_every_concrete_action_is_built_by_its_constructor(action_console):
    catalog = action_console.ActionCatalog.discover()

    constructed_types = {
        builder.action_type
        for builder in catalog.builders
        if isinstance(builder, action_console.ActionConstructor)
    }

    assert constructed_types == {
        action_type
        for action_type in recursive_subclasses(Action)
        if action_type.__module__.startswith("coraplex.robot_plans.actions")
        and not inspect.isabstract(action_type)
    }


def test_a_constructor_takes_the_parameters_of_its_action(action_console):
    constructor = action_console.ActionConstructor(NavigateAction)

    assert [parameter.name for parameter in constructor.parameters()] == [
        discovered.field.name for discovered in NavigateAction.parameter_fields()
    ]


# %% factories


def test_a_factory_inherited_from_another_action_builds_the_inheriting_action(
    action_console,
):
    catalog = action_console.ActionCatalog.discover()

    handle_factories = [
        builder
        for builder in catalog.builders
        if isinstance(builder, action_console.ActionFactory)
        and builder.method.__func__ is MovesContainerByItsHandle.from_handle.__func__
    ]

    assert [factory.action_type for factory in handle_factories] == [
        CloseAction,
        OpenAction,
    ]


def test_a_factory_hides_the_context_it_is_given(action_console):
    factory = action_console.ActionFactory(
        TransportAction, TransportAction.from_graspable_by_closest_grasps
    )

    assert [parameter.name for parameter in factory.parameters()] == [
        name
        for name in inspect.signature(
            TransportAction.from_graspable_by_closest_grasps
        ).parameters
        if name not in factory.context_parameter_names
    ]
    assert factory.context_parameter_names == ["context"]


@dataclass
class ContextRecordingFactory:
    """
    Records what a factory taking a context was handed.
    """

    target: Pose
    """
    The parameter a caller chooses.
    """

    context: StatechartContext
    """
    The context the factory was handed.
    """

    @classmethod
    def towards(cls, target: Pose, context: StatechartContext) -> Self:
        """
        :param target: The parameter a caller chooses.
        :param context: The context the action runs in.
        :return: The record of both.
        """
        return cls(target, context)


def test_a_factory_is_handed_the_context_the_action_runs_in(
    action_console, pr2_statechart_context
):
    factory = action_console.ActionFactory(
        NavigateAction, ContextRecordingFactory.towards
    )
    target = Pose()

    record = factory.build({"target": target}, pr2_statechart_context)

    assert record.target is target
    assert record.context is pr2_statechart_context


# %% requests


def test_a_nested_request_builds_the_action_its_parent_is_built_from(
    action_console, pr2_statechart_context
):
    target = Pose()
    request = action_console.ActionRequest(
        action_console.ActionConstructor(CarryAction),
        {
            "navigate": action_console.ActionRequest(
                action_console.ActionConstructor(NavigateAction),
                {"target_location": target},
            )
        },
    )

    carry = request.build(pr2_statechart_context)

    assert type(carry.navigate) is NavigateAction
    assert carry.navigate.target_location is target


def test_builders_of_a_kind_of_action_build_it_or_a_subclass_of_it(action_console):
    catalog = action_console.ActionCatalog.discover()

    built_types = {
        builder.action_type
        for builder in catalog.builders_of(MovesContainerByItsHandle)
    }

    assert built_types == {OpenAction, CloseAction}
