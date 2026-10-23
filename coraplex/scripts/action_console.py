"""
A terminal console that runs single coraplex actions against a robot in a scene.

Choose a robot, a scene and how actions are executed, then pick an action, or a factory
building one, from a fuzzy-searchable list, fill in its parameters with choices taken
from the world, and run it. Every action runs against the same world, so each one
continues where the previous one left off.

Requires the ``tui`` extra of coraplex: ``pip install -e coraplex[tui]``.
"""

from __future__ import annotations

import inspect
import logging
from abc import ABC, ABCMeta, abstractmethod
from dataclasses import MISSING, Field, dataclass, field
from enum import Enum, StrEnum
from importlib.util import find_spec
from inspect import isclass
from pathlib import Path
from types import NoneType, UnionType

from textual import on, work
from textual.app import App, ComposeResult
from textual.binding import Binding
from textual.containers import Horizontal, VerticalScroll
from textual.fuzzy import Matcher
from textual.message import Message
from textual.screen import Screen
from textual.validation import Integer, Number
from textual.widget import Widget
from textual.widgets import (
    Button,
    Collapsible,
    Footer,
    Header,
    Input,
    Label,
    LoadingIndicator,
    OptionList,
    RichLog,
    Select,
    SelectionList,
    Static,
    Switch,
)
from textual.worker import Worker, WorkerState
from typing_extensions import Any, Callable, Self, Union, get_args, get_origin

import coraplex.robot_plans.actions as actions_package
import semantic_digital_twin.robots as robots_package
from coraplex.demonstrations import RobotDemonstration
from coraplex.plans.context_extensions import RobotAccess
from coraplex.plans.executors import (
    KinematicallySimulatedPlanExecutor,
    PlanExecutor,
    RealRobotPlanExecutor,
)
from coraplex.robot_plans.actions.base import Action
from coraplex.standalone_process import StandaloneControllerProcess
from cramph.composites import Sequence
from cramph.context import ContextExtension, StatechartContext
from cramph.statechart import Statechart
from krrood.class_diagrams.utils import get_type_hints_of_object
from krrood.entity_query_language.query.match import Match
from krrood.ormatic.utils import classes_of_package
from krrood.patterns.path_enumeration import PathEnumeration
from krrood.utils import recursive_subclasses
from semantic_digital_twin.grasping.grasp_candidates import GraspCandidate
from semantic_digital_twin.robots.robot_part_mixins import RobotPartMixin
from semantic_digital_twin.robots.robot_parts import AbstractRobot
from semantic_digital_twin.semantic_annotations.mixins import HasGraspCandidates
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix
from semantic_digital_twin.spatial_types.spatial_types import Point2, Pose, Pose2D
from semantic_digital_twin.specifications.robots import RobotSpecification
from semantic_digital_twin.specifications.worlds import WorldSpecification
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.world_entity import (
    SemanticAnnotation,
    WorldEntity,
)

# %% scenes


class SceneFileFormat(StrEnum):
    """
    The formats a scene file is written in, by the suffix of its file.
    """

    URDF = ".urdf"
    MJCF = ".xml"


class ConsoleScene(PathEnumeration):
    """
    The maps the console can put a robot into, none of which holds a robot.
    """

    APARTMENT = (
        Path(__file__).resolve().parents[1] / "resources" / "worlds" / "apartment.urdf"
    )
    KITCHEN = (
        Path(__file__).resolve().parents[1] / "resources" / "worlds" / "kitchen.urdf"
    )
    IAI_KITCHEN = (
        Path(__file__).resolve().parents[1]
        / "resources"
        / "worlds"
        / "IAI_kitchen.urdf"
    )
    APARTMENT_MJCF = (
        Path(__file__).resolve().parents[2]
        / "semantic_digital_twin"
        / "resources"
        / "mjcf"
        / "apartment.xml"
    )

    def world_specification(self) -> WorldSpecification:
        """
        :return: The specification of the world this scene file describes.
        """
        if SceneFileFormat(self.suffix) is SceneFileFormat.MJCF:
            return WorldSpecification.from_mjcf(
                str(self), use_visual_as_collision_backup=True
            )
        return WorldSpecification.from_urdf(str(self))


# %% run modes


class GiskardStandaloneModule(StrEnum):
    """
    The modules running a standalone Giskard controller, which simulates its robot, by
    the robot they control.
    """

    PR2 = "giskardpy.middleware.ros2.scripts.iai_robots.pr2.pr2_standalone"
    HSRB = "giskardpy.middleware.ros2.scripts.iai_robots.hsr.hsr_standalone"
    STRETCH = "giskardpy.middleware.ros2.scripts.iai_robots.stretch.stretch_standalone"
    TRACY = "giskardpy.middleware.ros2.scripts.iai_robots.tracy.tracy_standalone"
    DAISY = "giskardpy.middleware.ros2.scripts.iai_robots.daisy.daisy_standalone"
    GARMI = "giskardpy.middleware.ros2.scripts.other_robots.garmi.garmi_standalone"

    @classmethod
    def for_robot(cls, robot_type: type[AbstractRobot]) -> Self | None:
        """
        :param robot_type: The robot to control.
        :return: The module controlling ``robot_type``, named after it, or ``None`` if
            there is none.
        """
        return cls.__members__.get(robot_type.__name__.upper())

    @property
    def launcher_path(self) -> Path:
        """
        :return: The file of this module, which runs the controller when executed.
        """
        return Path(find_spec(self.value).origin)


class RunMode(StrEnum):
    """
    How the actions of the console are executed.
    """

    SIMULATED = "simulated in this process"
    GISKARD_STARTED_BY_CONSOLE = "standalone Giskard started by the console"
    GISKARD_RUNNING = "Giskard already running"

    @property
    def executor_type(self) -> type[PlanExecutor]:
        """
        :return: The executor running the actions in this mode.
        """
        if self is RunMode.SIMULATED:
            return KinematicallySimulatedPlanExecutor
        return RealRobotPlanExecutor

    def controller_launcher(self, robot_type: type[AbstractRobot]) -> Path | None:
        """
        :param robot_type: The robot the actions are performed by.
        :return: The controller the console starts in this mode, or ``None`` if it
            starts none.
        """
        if self is not RunMode.GISKARD_STARTED_BY_CONSOLE:
            return None
        return GiskardStandaloneModule.for_robot(robot_type).launcher_path


# %% action catalog


@dataclass
class ActionParameter:
    """
    A parameter an action is built from.
    """

    name: str
    """
    The keyword the parameter is passed by.
    """

    type_hint: Any
    """
    The type the parameter is declared with.
    """

    has_default: bool
    """
    Whether the parameter may be left out.
    """

    default: Any = None
    """
    The value the parameter takes when it is left out, if :attr:`has_default`.
    """

    @classmethod
    def from_field(cls, action_field: Field, type_hint: Any) -> Self:
        """
        :param action_field: A dataclass field of an action.
        :param type_hint: The resolved type of ``action_field``.
        :return: The parameter ``action_field`` declares.
        """
        if action_field.default is not MISSING:
            return cls(action_field.name, type_hint, True, action_field.default)
        if action_field.default_factory is not MISSING:
            return cls(
                action_field.name, type_hint, True, action_field.default_factory()
            )
        return cls(action_field.name, type_hint, False)

    @classmethod
    def from_signature_parameter(
        cls, parameter: inspect.Parameter, type_hint: Any
    ) -> Self:
        """
        :param parameter: A parameter of a factory's signature.
        :param type_hint: The resolved type of ``parameter``.
        :return: The parameter ``parameter`` declares.
        """
        if parameter.default is inspect.Parameter.empty:
            return cls(parameter.name, type_hint, False)
        return cls(parameter.name, type_hint, True, parameter.default)

    @property
    def declares_none(self) -> bool:
        """
        :return: Whether the type of this parameter admits ``None``.
        """
        return get_origin(self.type_hint) in (
            Union,
            UnionType,
        ) and NoneType in get_args(self.type_hint)

    @property
    def allows_none(self) -> bool:
        """
        :return: Whether ``None`` is a valid value of this parameter, because its type
            admits it or because it is its default.
        """
        return self.declares_none or (self.has_default and self.default is None)

    @property
    def value_type(self) -> Any:
        """
        :return: The type of the values of this parameter other than ``None``.
        """
        if not self.declares_none:
            return self.type_hint
        remaining = tuple(
            argument
            for argument in get_args(self.type_hint)
            if argument is not NoneType
        )
        if len(remaining) == 1:
            return remaining[0]
        return Union[remaining]

    @property
    def description(self) -> str:
        """
        :return: The name of this parameter together with the type of its values.
        """
        if isclass(self.value_type):
            return f"{self.name}: {self.value_type.__name__}"
        return f"{self.name}: {self.value_type}"


@dataclass
class ActionBuilder(ABC):
    """
    A way to build an action of :attr:`action_type` from parameters.
    """

    action_type: type[Action]
    """
    The action this builder builds.
    """

    @property
    @abstractmethod
    def label(self) -> str:
        """
        :return: How the builder is listed.
        """

    @abstractmethod
    def parameters(self) -> list[ActionParameter]:
        """
        :return: The parameters a caller chooses the values of.
        """

    @abstractmethod
    def build(
        self, arguments: dict[str, Any], context: StatechartContext
    ) -> Action | Match:
        """
        :param arguments: The value of every parameter of :meth:`parameters` that is
            not left out, by its name.
        :param context: The context the action runs in.
        :return: The action, or the statement of one that is grounded when it runs.
        """


@dataclass
class ActionConstructor(ActionBuilder):
    """
    Builds an action by passing every parameter to its class.
    """

    @property
    def label(self) -> str:
        return self.action_type.__name__

    def parameters(self) -> list[ActionParameter]:
        type_hints = get_type_hints_of_object(self.action_type)
        return [
            ActionParameter.from_field(
                discovered.field, type_hints[discovered.field.name]
            )
            for discovered in self.action_type.parameter_fields()
        ]

    def build(self, arguments: dict[str, Any], context: StatechartContext) -> Action:
        return self.action_type(**arguments)


@dataclass
class ActionFactory(ActionBuilder):
    """
    Builds an action with a classmethod of its class, which derives the action's
    parameters from its own.

    A parameter of the factory that takes a :class:`StatechartContext` is given the
    context the action runs in, and is not chosen by the caller.
    """

    method: Callable[..., Action | Match]
    """
    The classmethod, bound to :attr:`action_type`.
    """

    @classmethod
    def all_of(cls, action_type: type[Action]) -> list[Self]:
        """
        :param action_type: A kind of action.
        :return: A factory for each classmethod of ``action_type`` that builds an action,
            whether declared on it or inherited from another kind of action.
        """
        seen_names = set()
        factories = []
        for declaring_type in action_type.__mro__:
            if not issubclass(declaring_type, Action):
                continue
            for method_name, member in vars(declaring_type).items():
                if method_name in seen_names:
                    continue
                seen_names.add(method_name)
                if isinstance(member, classmethod) and cls.builds_action(
                    member.__func__
                ):
                    factories.append(
                        cls(action_type, member.__get__(None, action_type))
                    )
        return factories

    @staticmethod
    def builds_action(function: Callable[..., Any]) -> bool:
        """
        :param function: The function of a classmethod of an action.
        :return: Whether ``function`` returns an action of its class, or a statement
            of one.
        """
        return_type = get_type_hints_of_object(function).get("return")
        if return_type is Self or return_type is Match:
            return True
        return isclass(return_type) and issubclass(return_type, Action)

    @property
    def label(self) -> str:
        return f"{self.action_type.__name__}.{self.method.__name__}"

    @property
    def context_parameter_names(self) -> list[str]:
        """
        :return: The names of the parameters given the context the action runs in.
        """
        return [
            name
            for name, type_hint in get_type_hints_of_object(
                self.method.__func__
            ).items()
            if isclass(type_hint) and issubclass(type_hint, StatechartContext)
        ]

    def parameters(self) -> list[ActionParameter]:
        type_hints = get_type_hints_of_object(self.method.__func__)
        context_parameter_names = self.context_parameter_names
        return [
            ActionParameter.from_signature_parameter(parameter, type_hints[name])
            for name, parameter in inspect.signature(self.method).parameters.items()
            if name not in context_parameter_names
        ]

    def build(
        self, arguments: dict[str, Any], context: StatechartContext
    ) -> Action | Match:
        contexts = {name: context for name in self.context_parameter_names}
        return self.method(**arguments, **contexts)


@dataclass
class ActionRequest:
    """
    A builder together with the values chosen for its parameters.
    """

    builder: ActionBuilder
    """
    The builder the action is built with.
    """

    arguments: dict[str, Any]
    """
    The values chosen for the parameters of :attr:`builder`, by name. A request as a
    value stands for the action it builds.
    """

    def build(self, context: StatechartContext) -> Action | Match:
        """
        :param context: The context the action runs in.
        :return: The action :attr:`builder` builds from :attr:`arguments`.
        """
        arguments = {
            name: value.build(context) if isinstance(value, ActionRequest) else value
            for name, value in self.arguments.items()
        }
        return self.builder.build(arguments, context)


@dataclass
class ActionCatalog:
    """
    Every way the console knows to build an action.
    """

    builders: list[ActionBuilder]
    """
    The builders, grouped by the action they build, in the order of its name.
    """

    @classmethod
    def discover(cls) -> Self:
        """
        :return: The constructor and the factories of every concrete action defined
            in :mod:`coraplex.robot_plans.actions`.
        """
        action_types = sorted(
            (
                action_type
                for action_type in classes_of_package(actions_package)
                if issubclass(action_type, Action)
                and not inspect.isabstract(action_type)
            ),
            key=lambda action_type: action_type.__name__,
        )
        builders = []
        for action_type in action_types:
            builders.append(ActionConstructor(action_type))
            builders.extend(ActionFactory.all_of(action_type))
        return cls(builders)

    def builders_of(self, action_type: type[Action]) -> list[ActionBuilder]:
        """
        :param action_type: A kind of action.
        :return: The builders of ``action_type`` and of its subclasses.
        """
        return [
            builder
            for builder in self.builders
            if issubclass(builder.action_type, action_type)
        ]


# %% console demonstration


@dataclass(kw_only=True)
class ConsoleDemonstration(RobotDemonstration):
    """
    A demonstration whose plan is the action the console was last asked to run.
    """

    scene: ConsoleScene = ConsoleScene.APARTMENT
    """
    The map the robot acts in.
    """

    robot_start: HomogeneousTransformationMatrix | None = None
    """
    Where a simulated robot starts in its ``odom`` frame; ``None`` starts it at its
    origin.
    """

    ros_node_name: str = "action_console"

    controller_launcher: Path | None = None
    """
    The standalone controller this demonstration starts before it acquires its world;
    ``None`` starts none.
    """

    action_request: ActionRequest | None = None
    """
    The action the next :meth:`perform` runs.
    """

    controller: StandaloneControllerProcess | None = field(init=False, default=None)
    """
    The controller started from :attr:`controller_launcher`, while it runs.
    """

    @classmethod
    def available_robot_types(cls) -> list[type[AbstractRobot]]:
        """
        :return: The concrete robots whose description is installed on this machine,
            in the order of their name.
        """
        return sorted(
            (
                robot_type
                for robot_type in classes_of_package(robots_package)
                if issubclass(robot_type, AbstractRobot)
                and not inspect.isabstract(robot_type)
                and robot_type.is_description_installed()
            ),
            key=lambda robot_type: robot_type.__name__,
        )

    def scene_specification(self) -> WorldSpecification:
        return self.scene.world_specification()

    def robot_specification(self) -> RobotSpecification:
        return RobotSpecification(
            semantic_annotation_type=self.used_robot,
            odom_T_robot_start=self.robot_start,
        )

    def is_scene_populated(self, world: World) -> bool:
        return world.is_kinematic_structure_entity_in_world_by_name(
            self.build_scene().root.name.name
        )

    def build_context_extensions(self, world: World) -> list[ContextExtension]:
        return [RobotAccess(world.get_semantic_annotations_by_type(self.used_robot)[0])]

    def build_statechart(self, context: StatechartContext) -> Statechart:
        statechart = Statechart(context=context)
        statechart.add_node(Sequence(nodes=[self.action_request.build(context)]))
        return statechart

    def acquire_world(self) -> World:
        """
        Differs from :meth:`RobotDemonstration.acquire_world` in first starting the
        controller of :attr:`controller_launcher`, if there is one.
        """
        if self.controller_launcher is not None:
            self.controller = StandaloneControllerProcess(
                launcher_path=self.controller_launcher
            )
            self.controller.start()
        return super().acquire_world()

    def tear_down(self) -> None:
        """
        Differs from :meth:`RobotDemonstration.tear_down` in also stopping the
        controller this demonstration started.
        """
        super().tear_down()
        if self.controller is None:
            return
        self.controller.stop()
        self.controller = None


@dataclass
class ConsoleSession:
    """
    The world the console acts on, and the ways it knows to build actions in it.
    """

    world: World
    """
    The world every action is performed against.
    """

    catalog: ActionCatalog
    """
    The ways to build an action.
    """

    field_registry: ParameterFieldRegistry
    """
    Chooses how the value of each parameter is entered.
    """


# %% choosing among values


@dataclass
class LabelledValue:
    """
    A value offered for choice, together with how it is listed.
    """

    label: str
    """
    How the value is listed.
    """

    value: Any
    """
    The value chosen by choosing :attr:`label`.
    """


class AbstractWidgetMeta(type(Widget), ABCMeta):
    """
    The metaclass of widgets that declare abstract methods.
    """


class FuzzyChoice(Widget):
    """
    A list of values to choose from, narrowed and ranked by fuzzy matching what is
    typed above it.

    Pressing enter in the search field chooses the best match, pressing it on the list
    or clicking chooses the highlighted value. The chosen value is shown below the list.
    """

    @dataclass
    class Chosen(Message):
        """
        Posted when a value is chosen.
        """

        choice: FuzzyChoice
        """
        The widget the value was chosen in.
        """

        value: Any
        """
        The chosen value.
        """

        @property
        def control(self) -> FuzzyChoice:
            return self.choice

    def __init__(
        self,
        options: list[LabelledValue],
        *,
        allows_none: bool = False,
        initial_value: Any = None,
        id: str | None = None,
        classes: str | None = None,
    ) -> None:
        """
        :param options: The values to choose from, in the order they are listed in
            while nothing is typed.
        :param allows_none: Whether ``None`` is offered first, as ``none``.
        :param initial_value: The value chosen before anything is chosen.
        :param id: The id of the widget.
        :param classes: The style classes of the widget.
        """
        super().__init__(id=id, classes=classes)
        self.options = list(options)
        if allows_none:
            self.options.insert(0, LabelledValue("none", None))
        self.listed_options = list(self.options)
        self.chosen_value = initial_value

    def compose(self) -> ComposeResult:
        yield Input(placeholder="type to search, enter to choose")
        yield OptionList(*(option.label for option in self.options), markup=False)
        yield Static(self.chosen_description(), classes="chosen", markup=False)

    def on_mount(self) -> None:
        chosen_indices = [
            index
            for index, option in enumerate(self.options)
            if option.value is self.chosen_value
        ]
        if chosen_indices:
            self.query_one(OptionList).highlighted = chosen_indices[0]

    def chosen_description(self) -> str:
        """
        :return: The line telling which value is chosen.
        """
        chosen_labels = [
            option.label for option in self.options if option.value is self.chosen_value
        ]
        if not chosen_labels:
            return "nothing chosen"
        return f"chosen: {chosen_labels[0]}"

    def choose(self, option: LabelledValue) -> None:
        """
        Choose the value of ``option``, and announce it.

        :param option: One of the options of this widget.
        """
        self.chosen_value = option.value
        self.query_one(".chosen", Static).update(self.chosen_description())
        self.post_message(self.Chosen(self, self.chosen_value))

    @on(Input.Changed)
    def narrow_options(self, event: Input.Changed) -> None:
        """
        List only the options matching what is typed, best match first and
        highlighted.
        """
        event.stop()
        matcher = Matcher(event.value)
        scored_options = [
            (matcher.match(option.label), option) for option in self.options
        ]
        self.listed_options = [
            option
            for score, option in sorted(
                scored_options, key=lambda scored: scored[0], reverse=True
            )
            if score > 0 or not event.value
        ]
        option_list = self.query_one(OptionList)
        option_list.set_options(option.label for option in self.listed_options)
        if self.listed_options:
            option_list.highlighted = 0

    @on(Input.Submitted)
    def choose_best_match(self, event: Input.Submitted) -> None:
        """
        Choose the highlighted option, which is the best match of what is typed.
        """
        event.stop()
        highlighted = self.query_one(OptionList).highlighted
        if highlighted is None:
            return
        self.choose(self.listed_options[highlighted])

    @on(OptionList.OptionSelected)
    def choose_selected(self, event: OptionList.OptionSelected) -> None:
        """
        Choose the option selected in the list.
        """
        event.stop()
        self.choose(self.listed_options[event.option_index])


# %% parameter fields


class ParameterField(Widget, metaclass=AbstractWidgetMeta):
    """
    Where the value of one parameter of an action is entered.
    """

    def __init__(self, parameter: ActionParameter, session: ConsoleSession) -> None:
        """
        :param parameter: The parameter whose value is entered.
        :param session: The session the value is chosen in.
        """
        super().__init__(classes="parameter-field")
        self.parameter = parameter
        self.session = session

    @classmethod
    @abstractmethod
    def accepts(cls, parameter: ActionParameter) -> bool:
        """
        :param parameter: A parameter of an action.
        :return: Whether the value of ``parameter`` can be entered in this field.
        """

    @abstractmethod
    def problem(self) -> str | None:
        """
        :return: Why the entered value cannot be used, or ``None`` if it can.
        """

    @abstractmethod
    def value(self) -> Any:
        """
        :return: The entered value, which has no :meth:`problem`.
        """


class EnumerationField(ParameterField):
    """
    Chooses a member of an enumeration.
    """

    @classmethod
    def accepts(cls, parameter: ActionParameter) -> bool:
        return isclass(parameter.value_type) and issubclass(parameter.value_type, Enum)

    def compose(self) -> ComposeResult:
        initial_value = (
            self.parameter.default
            if self.parameter.has_default and self.parameter.default is not None
            else Select.NULL
        )
        yield Select(
            [(member._name_, member) for member in self.parameter.value_type],
            value=initial_value,
        )

    def problem(self) -> str | None:
        if self.query_one(Select).is_blank() and not self.parameter.allows_none:
            return "choose a value"
        return None

    def value(self) -> Enum | None:
        selection = self.query_one(Select)
        if selection.is_blank():
            return None
        return selection.value


class BooleanField(ParameterField):
    """
    Switches a flag on or off.
    """

    @classmethod
    def accepts(cls, parameter: ActionParameter) -> bool:
        return parameter.value_type is bool

    def compose(self) -> ComposeResult:
        yield Switch(value=self.parameter.has_default and bool(self.parameter.default))

    def problem(self) -> str | None:
        return None

    def value(self) -> bool:
        return self.query_one(Switch).value


class NumberField(ParameterField):
    """
    Enters an integer or a real number.
    """

    @classmethod
    def accepts(cls, parameter: ActionParameter) -> bool:
        return parameter.value_type in (int, float)

    def compose(self) -> ComposeResult:
        initial_text = (
            str(self.parameter.default)
            if self.parameter.has_default and self.parameter.default is not None
            else ""
        )
        if self.parameter.value_type is int:
            yield Input(initial_text, type="integer", validators=[Integer()])
            return
        yield Input(initial_text, type="number", validators=[Number()])

    def problem(self) -> str | None:
        number_input = self.query_one(Input)
        if not number_input.value:
            return None if self.parameter.allows_none else "enter a number"
        if not number_input.validate(number_input.value).is_valid:
            return f"{number_input.value!r} is not a number"
        return None

    def value(self) -> int | float | None:
        text = self.query_one(Input).value
        if not text:
            return None
        return self.parameter.value_type(text)


class Coordinate(StrEnum):
    """
    The coordinates a pose is entered in, valued by the keyword
    :meth:`~semantic_digital_twin.spatial_types.spatial_types.Pose.from_xyz_rpy` takes
    them by.
    """

    X = "x"
    Y = "y"
    Z = "z"
    ROLL = "roll"
    PITCH = "pitch"
    YAW = "yaw"


class ReferencedCoordinatesField(ParameterField):
    """
    Enters a pose as coordinates relative to a chosen entity of the world. A parameter
    that allows ``None`` is ``None`` while no entity is chosen.
    """

    @property
    @abstractmethod
    def coordinates(self) -> list[Coordinate]:
        """
        :return: The coordinates entered, in the order they are listed.
        """

    @abstractmethod
    def build_value(
        self, reference_frame: WorldEntity, coordinates: dict[Coordinate, float]
    ) -> Any:
        """
        :param reference_frame: The entity the coordinates are relative to.
        :param coordinates: The value entered for each of :attr:`coordinates`.
        :return: The pose the coordinates describe.
        """

    def compose(self) -> ComposeResult:
        world = self.session.world
        yield FuzzyChoice(
            [
                LabelledValue(str(entity.name), entity)
                for entity in world.kinematic_structure_entities
            ],
            allows_none=self.parameter.allows_none,
            initial_value=None if self.parameter.allows_none else world.root,
            classes="reference-frame",
        )
        with Horizontal(classes="coordinates"):
            for coordinate in self.coordinates:
                yield Label(coordinate.value)
                yield Input(
                    "0", type="number", validators=[Number()], classes="coordinate"
                )

    def entered_coordinates(self) -> dict[Coordinate, str]:
        """
        :return: The text entered for each of :attr:`coordinates`.
        """
        return {
            coordinate: coordinate_input.value
            for coordinate, coordinate_input in zip(
                self.coordinates, self.query(".coordinate").results(Input)
            )
        }

    def problem(self) -> str | None:
        if self.query_one(FuzzyChoice).chosen_value is None:
            return None if self.parameter.allows_none else "choose a reference frame"
        invalid = [
            coordinate.value
            for coordinate, text in self.entered_coordinates().items()
            if not Number().validate(text).is_valid
        ]
        if invalid:
            return f"{', '.join(invalid)} not a number"
        return None

    def value(self) -> Any:
        reference_frame = self.query_one(FuzzyChoice).chosen_value
        if reference_frame is None:
            return None
        return self.build_value(
            reference_frame,
            {
                coordinate: float(text)
                for coordinate, text in self.entered_coordinates().items()
            },
        )


class PoseField(ReferencedCoordinatesField):
    """
    Enters a pose by its position and its roll, pitch and yaw.
    """

    @classmethod
    def accepts(cls, parameter: ActionParameter) -> bool:
        return parameter.value_type is Pose

    @property
    def coordinates(self) -> list[Coordinate]:
        return list(Coordinate)

    def build_value(
        self, reference_frame: WorldEntity, coordinates: dict[Coordinate, float]
    ) -> Pose:
        return Pose.from_xyz_rpy(
            **{coordinate.value: value for coordinate, value in coordinates.items()},
            reference_frame=reference_frame,
        )


class PlanarPoseField(ReferencedCoordinatesField):
    """
    Enters a pose in the plane by its position and its yaw.
    """

    @classmethod
    def accepts(cls, parameter: ActionParameter) -> bool:
        return parameter.value_type is Pose2D

    @property
    def coordinates(self) -> list[Coordinate]:
        return [Coordinate.X, Coordinate.Y, Coordinate.YAW]

    def build_value(
        self, reference_frame: WorldEntity, coordinates: dict[Coordinate, float]
    ) -> Pose2D:
        return Pose2D(
            Point2(
                coordinates[Coordinate.X],
                coordinates[Coordinate.Y],
                reference_frame=reference_frame,
            ),
            yaw=coordinates[Coordinate.YAW],
            reference_frame=reference_frame,
        )


class ChoiceField(ParameterField):
    """
    Chooses the value among a list of values.
    """

    @abstractmethod
    def choices(self) -> list[LabelledValue]:
        """
        :return: The values to choose from.
        """

    def compose(self) -> ComposeResult:
        yield FuzzyChoice(
            self.choices(),
            allows_none=self.parameter.allows_none,
            initial_value=self.parameter.default,
        )

    def problem(self) -> str | None:
        if self.value() is None and not self.parameter.allows_none:
            return "choose a value"
        return None

    def value(self) -> Any:
        return self.query_one(FuzzyChoice).chosen_value


class WorldEntityField(ChoiceField):
    """
    Chooses an entity of the world, such as a body, an arm or an annotated object.
    """

    @staticmethod
    def is_entity_type(value_type: Any) -> bool:
        """
        :param value_type: The type of a parameter's values.
        :return: Whether the values are entities of the world or parts of a robot in it.
        """
        return isclass(value_type) and issubclass(
            value_type, (WorldEntity, RobotPartMixin)
        )

    @classmethod
    def accepts(cls, parameter: ActionParameter) -> bool:
        return cls.is_entity_type(parameter.value_type)

    @staticmethod
    def labelled_entities(world: World, entity_type: type) -> list[LabelledValue]:
        """
        :param world: The world to look in.
        :param entity_type: The kind of entity to look for.
        :return: Every entity of ``world`` that is an ``entity_type``, labelled by its
            name and its kind.
        """
        return [
            LabelledValue(f"{entity.name} ({type(entity).__name__})", entity)
            for entity in [
                *world.semantic_annotations,
                *world.kinematic_structure_entities,
            ]
            if isinstance(entity, entity_type)
        ]

    def choices(self) -> list[LabelledValue]:
        return self.labelled_entities(self.session.world, self.parameter.value_type)


class SemanticAnnotationTypeField(ChoiceField):
    """
    Chooses a kind of semantic annotation.
    """

    @classmethod
    def accepts(cls, parameter: ActionParameter) -> bool:
        if get_origin(parameter.value_type) is not type:
            return False
        [annotation_type] = get_args(parameter.value_type)
        return isclass(annotation_type) and issubclass(
            annotation_type, SemanticAnnotation
        )

    def choices(self) -> list[LabelledValue]:
        [annotation_type] = get_args(self.parameter.value_type)
        return [
            LabelledValue(subclass.__name__, subclass)
            for subclass in sorted(
                set(recursive_subclasses(annotation_type)),
                key=lambda subclass: subclass.__name__,
            )
            if not inspect.isabstract(subclass)
        ]


class GraspCandidateField(ChoiceField):
    """
    Chooses one of the grasps the graspable objects of the world offer.
    """

    @classmethod
    def accepts(cls, parameter: ActionParameter) -> bool:
        return parameter.value_type is GraspCandidate

    def choices(self) -> list[LabelledValue]:
        return [
            LabelledValue(f"{graspable.name} grasp {index}", grasp)
            for graspable in self.session.world.semantic_annotations
            if isinstance(graspable, HasGraspCandidates)
            for index, grasp in enumerate(graspable.grasp_candidates())
        ]


class WorldEntityListField(ParameterField):
    """
    Chooses any number of entities of the world.
    """

    @classmethod
    def accepts(cls, parameter: ActionParameter) -> bool:
        if get_origin(parameter.value_type) is not list:
            return False
        [element_type] = get_args(parameter.value_type)
        return WorldEntityField.is_entity_type(element_type)

    def choices(self) -> list[LabelledValue]:
        """
        :return: The entities of the world to choose from.
        """
        [element_type] = get_args(self.parameter.value_type)
        return WorldEntityField.labelled_entities(self.session.world, element_type)

    def compose(self) -> ComposeResult:
        yield SelectionList(
            *((choice.label, index) for index, choice in enumerate(self.choices()))
        )

    def problem(self) -> str | None:
        return None

    def value(self) -> list[Any]:
        choices = self.choices()
        return [
            choices[index].value for index in self.query_one(SelectionList).selected
        ]


class NestedActionField(ParameterField):
    """
    Builds an action that is a parameter of another one, in a form of its own.
    """

    @classmethod
    def accepts(cls, parameter: ActionParameter) -> bool:
        return isclass(parameter.value_type) and issubclass(
            parameter.value_type, Action
        )

    def compose(self) -> ComposeResult:
        with Collapsible(title=self.parameter.description, collapsed=False):
            yield ActionForm(
                self.session.catalog.builders_of(self.parameter.value_type),
                self.session,
            )

    def problem(self) -> str | None:
        problems = self.query_one(ActionForm).problems()
        if not problems:
            return None
        return "; ".join(problems)

    def value(self) -> ActionRequest:
        return self.query_one(ActionForm).request()


class UnsupportedField(ParameterField):
    """
    Stands in for a parameter whose value cannot be entered in the console, which keeps
    its default.
    """

    @classmethod
    def accepts(cls, parameter: ActionParameter) -> bool:
        return True

    def compose(self) -> ComposeResult:
        if self.parameter.has_default:
            yield Static(f"cannot be entered here, keeps {self.parameter.default!r}")
            return
        yield Static("cannot be entered here")

    def problem(self) -> str | None:
        if self.parameter.has_default:
            return None
        return "cannot be entered in the console"

    def value(self) -> Any:
        return self.parameter.default


@dataclass
class ParameterFieldRegistry:
    """
    Chooses the field the value of a parameter is entered in.
    """

    field_types: list[type[ParameterField]] = field(
        default_factory=lambda: [
            EnumerationField,
            BooleanField,
            NumberField,
            PoseField,
            PlanarPoseField,
            GraspCandidateField,
            NestedActionField,
            SemanticAnnotationTypeField,
            WorldEntityListField,
            WorldEntityField,
            UnsupportedField,
        ]
    )
    """
    The kinds of field, the first of which that accepts a parameter is used for it.
    """

    def field_type_for(self, parameter: ActionParameter) -> type[ParameterField]:
        """
        :param parameter: A parameter of an action.
        :return: The first of :attr:`field_types` that accepts ``parameter``.
        """
        return next(
            field_type
            for field_type in self.field_types
            if field_type.accepts(parameter)
        )


# %% action forms


class ParameterForm(Widget):
    """
    The fields of every parameter of one builder.
    """

    def __init__(self, builder: ActionBuilder, session: ConsoleSession) -> None:
        """
        :param builder: The builder whose parameters are entered.
        :param session: The session the values are chosen in.
        """
        super().__init__()
        self.builder = builder
        self.fields = [
            session.field_registry.field_type_for(parameter)(parameter, session)
            for parameter in builder.parameters()
        ]

    def compose(self) -> ComposeResult:
        for parameter_field in self.fields:
            yield Label(
                parameter_field.parameter.description, classes="parameter-label"
            )
            yield parameter_field

    def problems(self) -> list[str]:
        """
        :return: Why the entered values cannot build the action, one line per field.
        """
        return [
            f"{parameter_field.parameter.name}: {problem}"
            for parameter_field in self.fields
            if (problem := parameter_field.problem()) is not None
        ]

    def request(self) -> ActionRequest:
        """
        :return: The request of an action with the entered values, which have no
            :meth:`problems`.
        """
        return ActionRequest(
            self.builder,
            {
                parameter_field.parameter.name: parameter_field.value()
                for parameter_field in self.fields
            },
        )


class ActionForm(Widget):
    """
    Chooses a builder and enters the values of its parameters.
    """

    def __init__(
        self,
        builders: list[ActionBuilder],
        session: ConsoleSession,
        *,
        id: str | None = None,
    ) -> None:
        """
        :param builders: The builders to choose from.
        :param session: The session the values are chosen in.
        :param id: The id of the widget.
        """
        super().__init__(id=id)
        self.builders = builders
        self.session = session
        self.parameter_form: ParameterForm | None = None

    def compose(self) -> ComposeResult:
        yield FuzzyChoice(
            [LabelledValue(builder.label, builder) for builder in self.builders],
            classes="builder-choice",
        )
        yield VerticalScroll(classes="parameters")

    @on(FuzzyChoice.Chosen)
    def show_parameters(self, event: FuzzyChoice.Chosen) -> None:
        """
        Show the parameters of the chosen builder.
        """
        if event.choice is not self.query_one(".builder-choice", FuzzyChoice):
            return
        event.stop()
        if (
            self.parameter_form is not None
            and self.parameter_form.builder is event.value
        ):
            return
        self.parameter_form = ParameterForm(event.value, self.session)
        parameters = self.query_one(".parameters", VerticalScroll)
        parameters.remove_children()
        parameters.mount(self.parameter_form)

    def problems(self) -> list[str]:
        """
        :return: Why no action can be built from this form, one line per reason.
        """
        if self.parameter_form is None:
            return ["choose an action"]
        return self.parameter_form.problems()

    def request(self) -> ActionRequest:
        """
        :return: The request of the action entered in this form, which has no
            :meth:`problems`.
        """
        return self.parameter_form.request()


# %% screens


@dataclass(eq=False)
class ScreenLogHandler(logging.Handler):
    """
    Shows the log records of the process on a screen of the console.
    """

    screen: ActionScreen
    """
    The screen the records are shown on.
    """

    def __post_init__(self) -> None:
        super().__init__(level=logging.INFO)

    def emit(self, record: logging.LogRecord) -> None:
        self.screen.post_message(ActionScreen.LogLine(self.format(record)))


class SetupScreen(Screen[ConsoleDemonstration]):
    """
    Chooses the robot, the scene and how actions are executed, and dismisses with the
    demonstration they describe.
    """

    def __init__(self, robot_types: list[type[AbstractRobot]]) -> None:
        """
        :param robot_types: The robots to choose from.
        """
        super().__init__()
        self.robot_types = robot_types

    def compose(self) -> ComposeResult:
        yield Header()
        with VerticalScroll(id="setup"):
            yield Label("Robot")
            yield FuzzyChoice(
                [
                    LabelledValue(robot_type.__name__, robot_type)
                    for robot_type in self.robot_types
                ],
                id="robot",
            )
            yield Label("Scene")
            yield Select(
                [(scene._name_, scene) for scene in ConsoleScene],
                value=ConsoleScene.APARTMENT,
                allow_blank=False,
                id="scene",
            )
            yield Label("Run mode")
            yield Select(
                [(run_mode.value, run_mode) for run_mode in RunMode],
                value=RunMode.SIMULATED,
                allow_blank=False,
                id="run-mode",
            )
            yield Label("Start of a simulated robot in its odom frame")
            with Horizontal(classes="coordinates"):
                for coordinate in (Coordinate.X, Coordinate.Y, Coordinate.YAW):
                    yield Label(coordinate.value)
                    yield Input(
                        "0",
                        type="number",
                        validators=[Number()],
                        id=f"start-{coordinate.value}",
                    )
            yield Static(id="setup-problem")
            yield Button("Start", id="start", variant="primary")
        yield Footer()

    def start_coordinate(self, coordinate: Coordinate) -> str:
        """
        :param coordinate: A coordinate of the start of the robot.
        :return: The text entered for ``coordinate``.
        """
        return self.query_one(f"#start-{coordinate.value}", Input).value

    def problem(self) -> str | None:
        """
        :return: Why no demonstration can be started from the choices, or ``None`` if
            one can.
        """
        robot_type = self.query_one("#robot", FuzzyChoice).chosen_value
        if robot_type is None:
            return "Choose a robot."
        run_mode = self.query_one("#run-mode", Select).value
        if (
            run_mode is RunMode.GISKARD_STARTED_BY_CONSOLE
            and GiskardStandaloneModule.for_robot(robot_type) is None
        ):
            return f"There is no standalone Giskard for the {robot_type.__name__}."
        for coordinate in (Coordinate.X, Coordinate.Y, Coordinate.YAW):
            if not Number().validate(self.start_coordinate(coordinate)).is_valid:
                return f"The start's {coordinate.value} is not a number."
        return None

    @on(Button.Pressed, "#start")
    def start(self) -> None:
        """
        Dismiss with the chosen demonstration, unless the choices have a problem.
        """
        problem = self.problem()
        if problem is not None:
            self.query_one("#setup-problem", Static).update(problem)
            return
        robot_type = self.query_one("#robot", FuzzyChoice).chosen_value
        run_mode = self.query_one("#run-mode", Select).value
        self.dismiss(
            ConsoleDemonstration(
                used_robot=robot_type,
                scene=self.query_one("#scene", Select).value,
                executor_type=run_mode.executor_type,
                controller_launcher=run_mode.controller_launcher(robot_type),
                robot_start=HomogeneousTransformationMatrix.from_xyz_rpy(
                    x=float(self.start_coordinate(Coordinate.X)),
                    y=float(self.start_coordinate(Coordinate.Y)),
                    yaw=float(self.start_coordinate(Coordinate.YAW)),
                ),
            )
        )


class WorldAcquisitionScreen(Screen):
    """
    Shown while the world of the console is acquired.
    """

    def compose(self) -> ComposeResult:
        yield Header()
        yield Label("Acquiring the world ...")
        yield LoadingIndicator()
        yield Footer()


class ActionScreen(Screen):
    """
    Chooses actions, enters their parameters and runs them, one at a time.
    """

    BINDINGS = [Binding("ctrl+r", "run_action", "Run action")]

    @dataclass
    class LogLine(Message):
        """
        Posted to show a line in the log.
        """

        text: str
        """
        The line to show.
        """

    def __init__(
        self, session: ConsoleSession, demonstration: ConsoleDemonstration
    ) -> None:
        """
        :param session: The session the actions are chosen in.
        :param demonstration: The demonstration performing the actions against the
            world of ``session``.
        """
        super().__init__()
        self.session = session
        self.demonstration = demonstration
        self.log_handler = ScreenLogHandler(self)
        self.performance: Worker | None = None

    def compose(self) -> ComposeResult:
        yield Header()
        with Horizontal():
            yield ActionForm(
                self.session.catalog.builders, self.session, id="action-form"
            )
            yield RichLog(id="log", wrap=True, markup=False)
        yield Static("Choose an action, then press ctrl+r to run it.", id="status")
        yield Footer()

    def on_mount(self) -> None:
        logging.getLogger().addHandler(self.log_handler)

    def on_unmount(self) -> None:
        logging.getLogger().removeHandler(self.log_handler)

    def on_action_screen_log_line(self, message: LogLine) -> None:
        self.query_one("#log", RichLog).write(message.text)

    def report(self, text: str) -> None:
        """
        Show ``text`` as the status, and keep it in the log.

        :param text: What happened.
        """
        self.query_one("#status", Static).update(text)
        self.query_one("#log", RichLog).write(text)

    def action_run_action(self) -> None:
        """
        Run the action entered in the form, unless it has a problem or another action is
        still running.
        """
        if self.performance is not None and self.performance.is_running:
            self.report("An action is still running.")
            return
        form = self.query_one("#action-form", ActionForm)
        problems = form.problems()
        if problems:
            self.report("Cannot run the action: " + "; ".join(problems))
            return
        self.demonstration.action_request = form.request()
        self.report(f"Running {self.demonstration.action_request.builder.label} ...")
        self.performance = self.perform_action()

    @work(thread=True, exit_on_error=False)
    def perform_action(self) -> None:
        """
        Perform the requested action against the world of the session.
        """
        self.demonstration.perform(self.session.world)

    def on_worker_state_changed(self, event: Worker.StateChanged) -> None:
        if event.worker is not self.performance:
            return
        event.stop()
        label = self.demonstration.action_request.builder.label
        if event.state is WorkerState.SUCCESS:
            self.report(f"{label} succeeded.")
        elif event.state is WorkerState.ERROR:
            self.report(f"{label} failed: {event.worker.error!r}")


# %% application


class ActionConsole(App):
    """
    Runs single actions against a robot in a scene.
    """

    CSS_PATH = "action_console.tcss"
    TITLE = "coraplex action console"

    def __init__(self) -> None:
        super().__init__()
        self.demonstration: ConsoleDemonstration | None = None
        self.acquisition: Worker | None = None

    def on_mount(self) -> None:
        self.show_setup()

    def show_setup(self) -> None:
        """
        Let the robot, the scene and the run mode be chosen, and start with them.
        """
        self.push_screen(
            SetupScreen(ConsoleDemonstration.available_robot_types()),
            callback=self.start_demonstration,
        )

    def start_demonstration(self, demonstration: ConsoleDemonstration) -> None:
        """
        Acquire the world of ``demonstration``, and act in it once it is acquired.

        :param demonstration: The demonstration chosen in the setup.
        """
        self.demonstration = demonstration
        self.push_screen(WorldAcquisitionScreen())
        self.acquisition = self.acquire_world()

    @work(thread=True, exit_on_error=False)
    def acquire_world(self) -> World:
        """
        :return: The world the demonstration acts on.
        """
        return self.demonstration.acquire_world()

    def on_worker_state_changed(self, event: Worker.StateChanged) -> None:
        if event.worker is not self.acquisition:
            return
        if event.state is WorkerState.SUCCESS:
            session = ConsoleSession(
                world=event.worker.result,
                catalog=ActionCatalog.discover(),
                field_registry=ParameterFieldRegistry(),
            )
            self.switch_screen(ActionScreen(session, self.demonstration))
        elif event.state is WorkerState.ERROR:
            self.notify(
                f"Could not acquire the world: {event.worker.error!r}",
                severity="error",
            )
            self.demonstration.tear_down()
            self.pop_screen()
            self.show_setup()

    def on_unmount(self) -> None:
        if self.demonstration is not None:
            self.demonstration.tear_down()


def main() -> None:
    """
    Run the console, showing what the process logs from its information level on.
    """
    logging.getLogger().setLevel(logging.INFO)
    ActionConsole().run()


if __name__ == "__main__":
    main()
