from __future__ import annotations

import logging
from abc import ABC, abstractmethod
from dataclasses import dataclass, field

from typing_extensions import (
    Any,
    Dict,
    Optional,
)

from coraplex.plans.context_extensions import RobotAccess, StatementGrounding
from coraplex.plans.designator import DesignatorParameters
from cramph.context import StatechartContext
from cramph.data_types import SuccessDecider
from cramph.node import CompositeNode, NodeArtifacts, StatechartNode
from krrood.entity_query_language.core.base_expressions import SymbolicExpression
from krrood.entity_query_language.core.variable import Variable
from krrood.patterns.field_metadata import JSONMetadata
from krrood.symbolic_math.symbolic_math import Scalar
from semantic_digital_twin.robots.robot_parts import AbstractRobot
from semantic_digital_twin.world_description.world_entity import (
    KinematicStructureEntity,
)

logger = logging.getLogger(__name__)


@dataclass(eq=False, repr=False)
class Action(CompositeNode, DesignatorParameters, ABC):
    """
    Something a robot does, described by the parameters it is given and run as a node of
    a statechart.

    What it does is its :attr:`action_body`, the node :meth:`create_action_body`
    creates. The action reaches its goal once its body succeeded, and declares itself
    failed as soon as the body ended without succeeding, so an action can be a step of
    another one.

    .. note:: :class:`~coraplex.plans.designator.DesignatorParameters` is the last
        base, because ORMatic resolves a data access object's parent by walking the
        method resolution order and taking the first mapped class, and an action is
        stored as the statechart node it is.
    """

    success_decided_by = SuccessDecider.ITSELF
    fails_when_observing_false = True

    _action_body: Optional[StatechartNode] = field(
        init=False,
        default=None,
        repr=False,
        metadata=JSONMetadata(serialize=True).as_dict(),
    )
    """
    The node running what this action does, created when it is expanded.

    Serialized, because a statechart is sent after its nodes expanded and the receiver
    does not expand them again.
    """

    @abstractmethod
    def create_action_body(self) -> StatechartNode:
        """
        :return: A new node running what this action does, usually a
            :class:`~cramph.composites.Sequence` of its steps. Called once, when the
            action is expanded.
        """

    @property
    def action_body(self) -> Optional[StatechartNode]:
        """
        :return: The node running what this action does, None until it was expanded.
        """
        return self._action_body

    @property
    def robot(self) -> AbstractRobot:
        """
        :return: The robot performing this action.
        """
        return self.context.require_extension(RobotAccess).robot

    @property
    def sampling_seed(self) -> Optional[int]:
        """
        :return: The seed for the locations this action samples, so a run can be
            repeated; ``None`` samples afresh each run.
        """
        return self.context.require_extension(StatementGrounding).sampling_seed

    @property
    def controlled_root(self) -> KinematicStructureEntity:
        """
        :return: The topmost entity this action's motions may move the robot relative
            to, which its Cartesian goals are expressed in.
        """
        return self.context.require_extension(RobotAccess).controlled_root

    def expand(self, context: StatechartContext) -> None:
        """
        Puts the node this action creates below it.
        """
        self._action_body = self.create_action_body()
        self._add_child_to_statechart(self._action_body)

    def build_artifacts(self, context: StatechartContext) -> NodeArtifacts:
        """
        Report what the body reached.

        It is read through its last observation, which outlasts it, because a node that
        ended observes nothing any more.
        """
        return NodeArtifacts(observation=Scalar(self._action_body.last_observation))

    @staticmethod
    def pre_condition(
        variables: Dict[str, Variable],
        context: StatechartContext,
        kwargs: Dict[str, Any],
    ) -> SymbolicExpression:
        return True

    @staticmethod
    def post_condition(
        variables: Dict[str, Variable],
        context: StatechartContext,
        kwargs: Dict[str, Any],
    ) -> SymbolicExpression:
        return True
