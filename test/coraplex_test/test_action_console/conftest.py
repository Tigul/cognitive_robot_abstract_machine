"""
Fixtures for the action console, which is a script rather than a module of a package.
"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path
from types import ModuleType

import pytest
from textual.app import App, ComposeResult
from textual.widget import Widget

from cramph.context import StatechartContext
from semantic_digital_twin.robots.pr2 import PR2

# %% the script


@pytest.fixture(scope="session")
def console_script_directory() -> Path:
    """
    The directory the action console and its style sheet are read from.
    """
    return Path(__file__).resolve().parents[3] / "coraplex" / "scripts"


@pytest.fixture(scope="session")
def action_console(console_script_directory) -> ModuleType:
    """
    The action console script, imported as the module ``action_console``.
    """
    specification = importlib.util.spec_from_file_location(
        "action_console", console_script_directory / "action_console.py"
    )
    module = importlib.util.module_from_spec(specification)
    sys.modules["action_console"] = module
    specification.loader.exec_module(module)
    return module


# %% hosting widgets


class WidgetHostApp(App):
    """
    An application showing nothing but the widgets it is given, styled as the console
    styles them.
    """

    def __init__(self, style_sheet: Path, *widgets: Widget) -> None:
        """
        :param style_sheet: The style sheet of the console.
        :param widgets: The widgets to show.
        """
        super().__init__(css_path=style_sheet)
        self.hosted_widgets = widgets

    def compose(self) -> ComposeResult:
        yield from self.hosted_widgets


@pytest.fixture
def host_widgets(console_script_directory):
    """
    Builds an application showing the given widgets.
    """

    def host(*widgets: Widget) -> WidgetHostApp:
        return WidgetHostApp(console_script_directory / "action_console.tcss", *widgets)

    return host


# %% sessions


@pytest.fixture
def console_session(action_console, pr2_apartment_context):
    """
    A session of the console acting on the PR2 apartment world.
    """
    world, _, _ = pr2_apartment_context
    return action_console.ConsoleSession(
        world=world,
        catalog=action_console.ActionCatalog.discover(),
        field_registry=action_console.ParameterFieldRegistry(),
    )


@pytest.fixture
def pr2_statechart_context(pr2_apartment_context) -> StatechartContext:
    """
    The context a plan of the PR2 in the apartment runs in.
    """
    world, _, extensions = pr2_apartment_context
    return StatechartContext.with_extensions(world, extensions)


@pytest.fixture
def pr2_console_demonstration(action_console):
    """
    A simulated demonstration of the PR2 performing what the console asks for.
    """
    return action_console.ConsoleDemonstration(used_robot=PR2)
