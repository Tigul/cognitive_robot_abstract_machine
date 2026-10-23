"""
Coverage for choosing, entering and running actions on the screens of the action
console.
"""

from __future__ import annotations

import pytest
from textual.app import App
from textual.widgets import Button, Collapsible, Input, SelectionList

from coraplex.plans.executors import RealRobotPlanExecutor
from coraplex.robot_plans.actions.composite.carrying import CarryAction
from coraplex.robot_plans.actions.core.navigation import NavigateAction
from coraplex.robot_plans.actions.core.pick_up import PickUpAction
from coraplex.robot_plans.actions.core.robot_body import ParkArmsAction
from semantic_digital_twin.robots.armar7 import Armar7
from semantic_digital_twin.robots.pr2 import PR2
from semantic_digital_twin.spatial_types.spatial_types import Pose

# %% helpers


async def choose_builder(pilot, form, typed_name: str) -> None:
    """
    Type ``typed_name`` into the list of builders of ``form`` and choose the best match.
    """
    form.query_one(".builder-choice").query_one(Input).focus()
    await pilot.press(*typed_name, "enter")
    await pilot.pause()


async def enter_coordinates(pilot, pose_field, coordinates: dict[str, float]) -> None:
    """
    Enter ``coordinates`` into ``pose_field``, by the name of each coordinate.
    """
    for coordinate, coordinate_input in zip(
        pose_field.coordinates, pose_field.query(".coordinate").results(Input)
    ):
        coordinate_input.value = str(coordinates.get(coordinate.value, 0.0))
    await pilot.pause()


# %% choosing actions


@pytest.mark.asyncio
async def test_typing_the_name_of_an_action_chooses_its_constructor(
    action_console, console_session, host_widgets
):
    form = action_console.ActionForm(console_session.catalog.builders, console_session)

    async with host_widgets(form).run_test(size=(160, 50)) as pilot:
        await choose_builder(pilot, form, "NavigateAction")

        assert form.parameter_form.builder == action_console.ActionConstructor(
            NavigateAction
        )


@pytest.mark.asyncio
async def test_a_nested_action_is_entered_in_a_form_of_its_own(
    action_console, console_session, host_widgets, pr2_statechart_context
):
    form = action_console.ActionForm(console_session.catalog.builders, console_session)
    target = {"x": 1.0, "y": 2.0}

    async with host_widgets(form).run_test(size=(160, 80)) as pilot:
        await choose_builder(pilot, form, "CarryAction")
        nested_form = form.parameter_form.query_one(action_console.ActionForm)
        await choose_builder(pilot, nested_form, "NavigateAction")
        await enter_coordinates(
            pilot, nested_form.query_one(action_console.PoseField), target
        )
        request = form.request()

    carry = request.build(pr2_statechart_context)
    world = console_session.world
    assert type(carry) is CarryAction
    assert type(carry.navigate) is NavigateAction
    assert carry.navigate.target_location.to_np() == pytest.approx(
        Pose.from_xyz_rpy(**target, reference_frame=world.root).to_np()
    )


# %% keyboard navigation


def letters_choice(action_console, choose_first: bool = False):
    """
    :param choose_first: Whether the first letter is chosen from the start.
    :return: A choice among values labelled ``a``, ``b`` and ``c``.
    """
    options = [action_console.LabelledValue(letter, object()) for letter in "abc"]
    return action_console.FuzzyChoice(
        options, initial_value=options[0].value if choose_first else None
    )


@pytest.mark.asyncio
async def test_the_arrow_keys_move_the_highlight_of_an_open_list(
    action_console, host_widgets
):
    choice = letters_choice(action_console)

    async with host_widgets(choice).run_test() as pilot:
        choice.query_one(Input).focus()
        await pilot.press("enter", "down", "down", "up", "enter")
        await pilot.pause()

        assert choice.chosen_value is choice.options[1].value


@pytest.mark.asyncio
async def test_deleting_everything_typed_lists_every_option_again(
    action_console, host_widgets
):
    choice = letters_choice(action_console)

    async with host_widgets(choice).run_test() as pilot:
        choice.query_one(Input).focus()
        await pilot.pause()
        await pilot.press("c", "backspace", "backspace")
        await pilot.pause()

        assert choice.listed_options == choice.options


@pytest.mark.asyncio
async def test_a_choice_without_options_says_there_is_nothing_to_choose(
    action_console, host_widgets
):
    choice = action_console.FuzzyChoice([])

    async with host_widgets(choice).run_test():
        assert (
            choice.query_one(Input).placeholder
            == action_console.ChoicePrompt.NOTHING_TO_CHOOSE
        )


@pytest.mark.asyncio
async def test_escape_closes_a_list_without_choosing(action_console, host_widgets):
    choice = letters_choice(action_console, choose_first=True)
    [first_option, *_] = choice.options

    async with host_widgets(choice).run_test() as pilot:
        choice.query_one(Input).focus()
        await pilot.press("c", "escape")
        await pilot.pause()

        assert choice.chosen_value is first_option.value
        assert choice.query_one(Input).value == first_option.label
        assert not choice.is_open


@pytest.mark.asyncio
async def test_choosing_an_action_moves_on_to_its_first_parameter(
    action_console, console_session, pr2_console_demonstration, console_script_directory
):
    screen = action_console.ActionScreen(console_session, pr2_console_demonstration)
    app = App(css_path=console_script_directory / "action_console.tcss")

    async with app.run_test(size=(200, 60)) as pilot:
        await app.push_screen(screen)
        form = screen.query_one("#action-form", action_console.ActionForm)
        await choose_builder(pilot, form, "NavigateAction")
        [first_field, *_] = form.parameter_form.fields

        assert first_field in app.focused.ancestors


@pytest.mark.asyncio
async def test_the_arrow_keys_move_between_fields_while_no_list_is_open(
    action_console, console_session, pr2_console_demonstration, console_script_directory
):
    screen = action_console.ActionScreen(console_session, pr2_console_demonstration)
    app = App(css_path=console_script_directory / "action_console.tcss")

    async with app.run_test(size=(200, 60)) as pilot:
        await app.push_screen(screen)
        form = screen.query_one("#action-form", action_console.ActionForm)
        await choose_builder(pilot, form, "NavigateAction")
        await pilot.press("down")
        await pilot.pause()
        pose_field = form.parameter_form.query_one(action_console.PoseField)

        assert app.focused.parent is pose_field.query_one(Collapsible)


@pytest.mark.asyncio
async def test_the_arrow_keys_leave_a_selection_list_past_its_first_entry(
    action_console, console_session, pr2_console_demonstration, console_script_directory
):
    screen = action_console.ActionScreen(console_session, pr2_console_demonstration)
    app = App(css_path=console_script_directory / "action_console.tcss")

    async with app.run_test(size=(200, 60)) as pilot:
        await app.push_screen(screen)
        form = screen.query_one("#action-form", action_console.ActionForm)
        await choose_builder(pilot, form, ParkArmsAction.__name__)
        fields = form.parameter_form.fields
        [arms_index] = [
            index
            for index, parameter_field in enumerate(fields)
            if isinstance(parameter_field, action_console.WorldEntityListField)
        ]
        arms_list = fields[arms_index].query_one(SelectionList)
        arms_list.focus()
        arms_list.highlighted = 0
        await pilot.press("up")
        await pilot.pause()

        assert fields[arms_index - 1] in app.focused.ancestors


# %% running actions


@pytest.mark.asyncio
async def test_a_chosen_action_runs_against_the_world_of_the_session(
    action_console,
    console_session,
    pr2_console_demonstration,
    pr2_apartment_context,
    console_script_directory,
):
    _, pr2, _ = pr2_apartment_context
    screen = action_console.ActionScreen(console_session, pr2_console_demonstration)
    app = App(css_path=console_script_directory / "action_console.tcss")
    target = {"x": 1.6, "y": 2.2}

    async with app.run_test(size=(200, 60)) as pilot:
        await app.push_screen(screen)
        form = screen.query_one("#action-form", action_console.ActionForm)
        await choose_builder(pilot, form, "NavigateAction")
        await enter_coordinates(
            pilot, form.parameter_form.query_one(action_console.PoseField), target
        )
        await pilot.press("ctrl+r")
        await app.workers.wait_for_complete()
        await pilot.pause()

    base_pose_threshold = pr2_console_demonstration.action_request.arguments[
        "base_pose_threshold"
    ]
    assert pr2.root.global_pose.position.to_np()[:2] == pytest.approx(
        [target["x"], target["y"]], abs=base_pose_threshold
    )


@pytest.mark.asyncio
async def test_an_action_with_a_problem_is_not_run(
    action_console,
    console_session,
    pr2_console_demonstration,
    console_script_directory,
):
    screen = action_console.ActionScreen(console_session, pr2_console_demonstration)
    app = App(css_path=console_script_directory / "action_console.tcss")

    async with app.run_test(size=(200, 60)) as pilot:
        await app.push_screen(screen)
        form = screen.query_one("#action-form", action_console.ActionForm)
        await choose_builder(pilot, form, "PickUpAction.from_above")
        await pilot.press("ctrl+r")
        await pilot.pause()

    assert form.parameter_form.builder.action_type is PickUpAction
    assert pr2_console_demonstration.action_request is None


# %% setup


async def run_setup(
    action_console, console_script_directory, robot_types, robot_name, run_mode
) -> list:
    """
    Choose ``robot_name`` and ``run_mode`` in the setup, and start.

    :return: What the setup dismissed with, if it did.
    """
    dismissed = []
    app = App(css_path=console_script_directory / "action_console.tcss")
    async with app.run_test(size=(120, 60)) as pilot:
        screen = action_console.SetupScreen(robot_types)
        await app.push_screen(screen, callback=dismissed.append)
        screen.query_one("#robot").query_one(Input).focus()
        await pilot.press(*robot_name, "enter")
        screen.query_one("#run-mode").query_one(Input).focus()
        await pilot.press(*run_mode.value, "enter")
        screen.query_one("#start-x", Input).value = "1.5"
        await pilot.pause()
        screen.query_one("#start", Button).press()
        await pilot.pause()
    return dismissed


@pytest.mark.asyncio
async def test_the_setup_describes_the_chosen_demonstration(
    action_console, console_script_directory
):
    [demonstration] = await run_setup(
        action_console,
        console_script_directory,
        [Armar7, PR2],
        "PR2",
        action_console.RunMode.GISKARD_STARTED_BY_CONSOLE,
    )

    assert demonstration.used_robot is PR2
    assert demonstration.executor_type is RealRobotPlanExecutor
    assert (
        demonstration.controller_launcher
        == action_console.GiskardStandaloneModule.PR2.launcher_path
    )
    assert demonstration.robot_start.to_np() == pytest.approx(
        action_console.HomogeneousTransformationMatrix.from_xyz_rpy(x=1.5).to_np()
    )


@pytest.mark.asyncio
async def test_the_setup_does_not_start_giskard_for_a_robot_it_has_none_for(
    action_console, console_script_directory
):
    dismissed = await run_setup(
        action_console,
        console_script_directory,
        [Armar7, PR2],
        "Armar7",
        action_console.RunMode.GISKARD_STARTED_BY_CONSOLE,
    )

    assert dismissed == []
