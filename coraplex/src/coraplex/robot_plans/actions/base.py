from __future__ import annotations

import logging
from abc import ABC, abstractmethod
from dataclasses import Field, dataclass, field, fields

from typing_extensions import (
    Any,
    Dict,
    List,
    Optional,
)

from coraplex.datastructures.dataclasses import Context, PlanContextExtension
from coraplex.plans.designator import DesignatorParameters
from coraplex.plans.factories import ActionLike, as_statechart_node
from cramph.composites import Sequence
from cramph.context import StatechartContext
from cramph.data_types import SuccessDecider
from cramph.node import CompositeNode, NodeArtifacts, StatechartNode
from krrood.entity_query_language.core.base_expressions import SymbolicExpression
from krrood.entity_query_language.core.variable import Variable
from krrood.patterns.field_metadata import JSONMetadata
from krrood.symbolic_math.symbolic_math import Scalar
from semantic_digital_twin.robots.robot_parts import AbstractRobot
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.world_entity import (
    KinematicStructureEntity,
)

logger = logging.getLogger(__name__)


@dataclass(eq=False, repr=False)
class Action(CompositeNode, DesignatorParameters, ABC):
    """
    Something a robot does, described by the parameters it is given and run as a node of
    a statechart.

    The nodes named in :attr:`_sub_nodes` run one after another. The action reaches its
    goal once the last of them succeeded, and declares itself failed as soon as one of
    them ended without succeeding, so an action can be a step of another one.

    .. note:: :class:`~coraplex.plans.designator.DesignatorParameters` is the last
        base, because ORMatic resolves a data access object's parent by walking the
        method resolution order and taking the first mapped class, and an action is
        stored as the statechart node it is.
    """

    success_decided_by = SuccessDecider.ITSELF
    fails_when_observing_false = True

    _action_body: Optional[Sequence] = field(
        init=False,
        default=None,
        repr=False,
        metadata=JSONMetadata(serialize=True).as_dict(),
    )
    """
    The sequence running the nodes this action is made of, created when it is expanded.

    Serialized, because a statechart is sent after its nodes expanded and the receiver
    does not expand them again.
    """

    @property
    @abstractmethod
    def _sub_nodes(self) -> List[ActionLike]:
        """
        :return: The steps this action runs, in the order they run in.
        """

    @property
    def context(self) -> Context:
        """
        :return: The plan context this action is executed for.
        """
        return self.statechart.context.require_extension(PlanContextExtension).context

    @property
    def world(self) -> World:
        """
        :return: The world this action is executed in.
        """
        return self.statechart.context.world

    @property
    def robot(self) -> AbstractRobot:
        """
        :return: The robot performing this action.
        """
        return self.context.robot

    @property
    def controlled_root(self) -> KinematicStructureEntity:
        """
        :return: The topmost entity this action's motions may move the robot relative
            to, which its Cartesian goals are expressed in.
        """
        return self.context.controlled_root

    @classmethod
    def _machinery_fields(cls) -> List[Field]:
        return list(fields(Action))

    def expand(self, context: StatechartContext) -> None:
        """
        Puts the nodes this action is made of into one sequence below it.
        """
        self._action_body = Sequence(
            name=f"{self.name}/body",
            nodes=[as_statechart_node(step) for step in self._sub_nodes],
        )
        self._add_child_to_statechart(self._action_body)

    def build_artifacts(self, context: StatechartContext) -> NodeArtifacts:
        """
        Report what the sequence below reached.

        It is read through its last observation, which outlasts it, because a node that
        ended observes nothing any more.
        """
        return NodeArtifacts(observation=Scalar(self._action_body.last_observation))

    @staticmethod
    def pre_condition(
        variables: Dict[str, Variable], context: Context, kwargs: Dict[str, Any]
    ) -> SymbolicExpression:
        return True

    @staticmethod
    def post_condition(
        variables: Dict[str, Variable], context: Context, kwargs: Dict[str, Any]
    ) -> SymbolicExpression:
        return True
