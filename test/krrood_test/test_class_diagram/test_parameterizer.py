from __future__ import annotations
import pytest
from random_events.set import Set
from random_events.variable import Continuous, Integer, Symbolic
from krrood.class_diagrams.class_diagram import ClassDiagram
from krrood.class_diagrams.parameterizer import Parameterizer
from pycram.datastructures.enums import TorsoState
from pycram.robot_plans import MoveTorsoAction
from pycram.robot_plans.actions.core.navigation import NavigateAction
from pycram.datastructures.pose import (
    PoseStamped,
    PyCramPose,
    PyCramVector3,
    PyCramQuaternion,
    Header,
)
from ..dataset.example_classes import (
    Position,
    Orientation,
    Pose,
    Atom,
    Element,
)


@pytest.fixture
def parameterizer() -> Parameterizer:
    """
    Fixture for the Parameterizer instance.
    """
    return Parameterizer()


def test_parameterize_position(parameterizer: Parameterizer):
    """
    Test parameterization of the Position class.
    """
    class_diagram = ClassDiagram([Position])
    wrapped_position = class_diagram.get_wrapped_class(Position)
    variables = parameterizer(wrapped_position)
    expected_variables = [
        Continuous("Position.x"),
        Continuous("Position.y"),
        Continuous("Position.z"),
    ]
    assert variables == expected_variables


def test_parameterize_orientation(parameterizer: Parameterizer):
    """
    Test parameterization of the Orientation class.
    """
    class_diagram = ClassDiagram([Orientation])
    wrapped_orientation = class_diagram.get_wrapped_class(Orientation)
    variables = parameterizer(wrapped_orientation)
    expected_variables = [
        Continuous("Orientation.x"),
        Continuous("Orientation.y"),
        Continuous("Orientation.z"),
        Continuous("Orientation.w"),
    ]

    assert variables == expected_variables


def test_parameterize_pose(parameterizer: Parameterizer):
    """
    Test parameterization of the Pose class.
    """
    class_diagram = ClassDiagram([Pose, Position, Orientation])
    wrapped_pose = class_diagram.get_wrapped_class(Pose)
    variables = parameterizer(wrapped_pose)
    expected_variables = [
        Continuous("Pose.position.x"),
        Continuous("Pose.position.y"),
        Continuous("Pose.position.z"),
        Continuous("Pose.orientation.x"),
        Continuous("Pose.orientation.y"),
        Continuous("Pose.orientation.z"),
        Continuous("Pose.orientation.w"),
    ]

    assert variables == expected_variables


def test_parameterize_atom(parameterizer: Parameterizer):
    """
    Test parameterization of the Atom class.
    """
    class_diagram = ClassDiagram([Atom, Element])
    wrapped_atom = class_diagram.get_wrapped_class(Atom)
    variables = parameterizer(wrapped_atom)
    expected_variables = [
        Symbolic("Atom.element", Set.from_iterable([Element.C, Element.H])),
        Integer("Atom.type"),
        Continuous("Atom.charge"),
    ]

    assert [(type(v), v.name) for v in variables] == [
        (type(v), v.name) for v in expected_variables
    ]


def test_create_fully_factorized_distribution(parameterizer: Parameterizer):
    """
    Test for a fully factorized distribution.
    """
    variables = [
        Continuous("Variable.A"),
        Continuous("Variable.B"),
    ]
    probabilistic_circuit = parameterizer.create_fully_factorized_distribution(
        variables
    )
    assert len(probabilistic_circuit.variables) == 2
    assert set(probabilistic_circuit.variables) == set(variables)


def test_parameterize_movetorso_action(parameterizer: Parameterizer):
    """
    Test Parameterizer for MoveTorsoAction with multiple torso states.
    """
    class_diagram = ClassDiagram([MoveTorsoAction])
    wrapped_action = class_diagram.get_wrapped_class(MoveTorsoAction)
    variables = parameterizer(wrapped_action)

    expected_variable = Symbolic(
        "MoveTorsoAction.torso_state",
        Set.from_iterable(list(TorsoState))
    )

    assert len(variables) == 1
    variable = variables[0]
    assert isinstance(variable, Symbolic)
    assert variable.name == expected_variable.name
    assert set(variable.domain) == set(expected_variable.domain)

    # Assert that all states from TorsoState are represented in the domain
    domain_values = {str(value) for value in variable.domain}
    expected_values = {str(int(state)) for state in TorsoState}
    assert domain_values == expected_values


def test_parameterize_navigate_action(parameterizer: Parameterizer):
    """
    Test parameterization of the NavigateAction class.
    """
    class_diagram = ClassDiagram([
        NavigateAction,
        PoseStamped,
        PyCramPose,
        PyCramVector3,
        PyCramQuaternion,
        Header
    ])
    wrapped_navigate_action = class_diagram.get_wrapped_class(NavigateAction)
    variables = parameterizer(wrapped_navigate_action)

    expected_variable_names = {
        "NavigateAction.target_location.pose.position.x",
        "NavigateAction.target_location.pose.position.y",
        "NavigateAction.target_location.pose.position.z",
        "NavigateAction.target_location.pose.orientation.x",
        "NavigateAction.target_location.pose.orientation.y",
        "NavigateAction.target_location.pose.orientation.z",
        "NavigateAction.target_location.pose.orientation.w",
        "NavigateAction.target_location.header.sequence",
        "NavigateAction.keep_joint_states",
    }

    variable_names = {v.name for v in variables}
    assert variable_names == expected_variable_names


