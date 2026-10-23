"""
Coverage for how the action console enters the value of each parameter of an action.
"""

from __future__ import annotations

import pytest
from textual.widgets import Input, Select, SelectionList

from coraplex.datastructures.enums import DetectionTechnique
from coraplex.robot_plans.actions.composite.carrying import CarryAction
from coraplex.robot_plans.actions.core.misc import DetectAction, MoveToReach
from coraplex.robot_plans.actions.core.navigation import LookAtAction, NavigateAction
from coraplex.robot_plans.actions.core.pick_up import PickUpAction, ReachAction
from coraplex.robot_plans.actions.core.robot_body import (
    ParkArmsAction,
    SetGripperAction,
)
from semantic_digital_twin.spatial_types.spatial_types import Pose

# %% choosing a field


def parameter_named(action_console, action_type: type, name: str):
    """
    :return: The parameter ``name`` of the constructor of ``action_type``.
    """
    [parameter] = [
        parameter
        for parameter in action_console.ActionConstructor(action_type).parameters()
        if parameter.name == name
    ]
    return parameter


def test_each_kind_of_parameter_is_entered_in_its_own_field(action_console):
    registry = action_console.ParameterFieldRegistry()
    expected_field_types = {
        (NavigateAction, "base_pose_threshold"): action_console.NumberField,
        (NavigateAction, "target_location"): action_console.PoseField,
        (MoveToReach, "target_pose_offset_robot"): action_console.PlanarPoseField,
        (SetGripperAction, "motion"): action_console.EnumerationField,
        (ReachAction, "reverse_reach_order"): action_console.BooleanField,
        (SetGripperAction, "gripper"): action_console.WorldEntityField,
        (LookAtAction, "camera"): action_console.WorldEntityField,
        (ParkArmsAction, "arms"): action_console.WorldEntityListField,
        (DetectAction, "object_sem_annotation"): (
            action_console.SemanticAnnotationTypeField
        ),
        (PickUpAction, "grasp"): action_console.GraspCandidateField,
        (CarryAction, "navigate"): action_console.NestedActionField,
        (DetectAction, "detection_box"): action_console.UnsupportedField,
    }

    field_types = {
        (action_type, name): registry.field_type_for(
            parameter_named(action_console, action_type, name)
        )
        for action_type, name in expected_field_types
    }

    assert field_types == expected_field_types


# %% numbers


@pytest.mark.asyncio
async def test_a_number_is_required_unless_none_is_allowed(
    action_console, console_session, host_widgets
):
    required = action_console.NumberField(
        action_console.ActionParameter("width", float, has_default=False),
        console_session,
    )
    optional = action_console.NumberField(
        action_console.ActionParameter("width", float | None, has_default=False),
        console_session,
    )

    async with host_widgets(required, optional).run_test():
        assert required.problem() is not None
        assert optional.problem() is None
        assert optional.value() is None


@pytest.mark.asyncio
async def test_a_number_is_read_from_its_text(
    action_console, console_session, host_widgets
):
    number_field = action_console.NumberField(
        action_console.ActionParameter("width", float, has_default=False),
        console_session,
    )

    async with host_widgets(number_field).run_test() as pilot:
        number_field.query_one(Input).value = "0.25"
        await pilot.pause()
        assert number_field.problem() is None
        assert number_field.value() == 0.25


@pytest.mark.asyncio
async def test_text_that_is_no_number_is_a_problem(
    action_console, console_session, host_widgets
):
    number_field = action_console.NumberField(
        action_console.ActionParameter("width", float, has_default=False),
        console_session,
    )

    async with host_widgets(number_field).run_test() as pilot:
        number_field.query_one(Input).value = "-"
        await pilot.pause()
        assert number_field.problem() is not None


# %% enumerations and defaults


@pytest.mark.asyncio
async def test_a_field_starts_at_the_default_of_its_parameter(
    action_console, console_session, host_widgets
):
    parameter = action_console.ActionParameter(
        "technique",
        DetectionTechnique,
        has_default=True,
        default=DetectionTechnique.ALL,
    )
    enumeration_field = action_console.EnumerationField(parameter, console_session)

    async with host_widgets(enumeration_field).run_test():
        assert enumeration_field.value() is parameter.default


@pytest.mark.asyncio
async def test_a_default_of_none_needs_no_choice(
    action_console, console_session, host_widgets
):
    annotation_type_field = action_console.SemanticAnnotationTypeField(
        parameter_named(action_console, DetectAction, "object_sem_annotation"),
        console_session,
    )

    async with host_widgets(annotation_type_field).run_test():
        assert annotation_type_field.problem() is None
        assert annotation_type_field.value() is None


@pytest.mark.asyncio
async def test_a_choice_without_a_default_must_be_made(
    action_console, console_session, host_widgets
):
    gripper_field = action_console.WorldEntityField(
        parameter_named(action_console, SetGripperAction, "gripper"), console_session
    )

    async with host_widgets(gripper_field).run_test():
        assert gripper_field.problem() is not None


@pytest.mark.asyncio
async def test_an_enumeration_without_a_default_must_be_chosen(
    action_console, console_session, host_widgets
):
    motion_field = action_console.EnumerationField(
        parameter_named(action_console, SetGripperAction, "motion"), console_session
    )

    async with host_widgets(motion_field).run_test() as pilot:
        assert motion_field.problem() is not None
        [first_member, *_] = motion_field.parameter.value_type
        motion_field.query_one(Select).value = first_member
        await pilot.pause()
        assert motion_field.value() is first_member


# %% poses


@pytest.mark.asyncio
async def test_a_pose_is_entered_relative_to_the_world_root_by_default(
    action_console, console_session, host_widgets
):
    pose_field = action_console.PoseField(
        parameter_named(action_console, NavigateAction, "target_location"),
        console_session,
    )
    coordinates = {"x": 1.0, "y": 2.0, "z": 0.5, "roll": 0.0, "pitch": 0.0, "yaw": 1.5}

    async with host_widgets(pose_field).run_test() as pilot:
        for coordinate, coordinate_input in zip(
            pose_field.coordinates, pose_field.query(".coordinate").results(Input)
        ):
            coordinate_input.value = str(coordinates[coordinate.value])
        await pilot.pause()
        pose = pose_field.value()

    world = console_session.world
    assert pose.reference_frame is world.root
    assert pose.to_np() == pytest.approx(
        Pose.from_xyz_rpy(**coordinates, reference_frame=world.root).to_np()
    )


@pytest.mark.asyncio
async def test_a_pose_that_allows_none_is_none_until_a_frame_is_chosen(
    action_console, console_session, host_widgets
):
    parameter = action_console.ActionParameter(
        "target_pose", Pose | None, has_default=True
    )
    pose_field = action_console.PoseField(parameter, console_session)

    async with host_widgets(pose_field).run_test():
        assert pose_field.problem() is None
        assert pose_field.value() is None


# %% lists of entities


@pytest.mark.asyncio
async def test_the_selected_entities_of_a_list_are_its_value(
    action_console, console_session, host_widgets, pr2_apartment_context
):
    _, pr2, _ = pr2_apartment_context
    arms_field = action_console.WorldEntityListField(
        parameter_named(action_console, ParkArmsAction, "arms"), console_session
    )
    chosen_arm = pr2.all_arms[0]

    async with host_widgets(arms_field).run_test() as pilot:
        [chosen_index] = [
            index
            for index, choice in enumerate(arms_field.choices())
            if choice.value is chosen_arm
        ]
        arms_field.query_one(SelectionList).select(chosen_index)
        await pilot.pause()
        arms = arms_field.value()

    assert arms == [chosen_arm]
