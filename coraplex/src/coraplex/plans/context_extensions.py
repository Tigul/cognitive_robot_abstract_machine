from __future__ import annotations

from dataclasses import dataclass, field, replace

from typing_extensions import Optional, Self, TYPE_CHECKING

from coraplex.datastructures.enums import ExecutionType
from cramph.context import ContextExtension
from krrood.entity_query_language.backends import (
    EntityQueryLanguageGenerativeBackend,
    QueryBackend,
)
from semantic_digital_twin.robots.robot_part_mixins import HasMobileBase
from semantic_digital_twin.robots.robot_parts import AbstractRobot
from semantic_digital_twin.world_description.world_entity import (
    KinematicStructureEntity,
)

if TYPE_CHECKING:
    from semantic_digital_twin.world import World


# %% extensions a plan's context carries over to a copy of its world


@dataclass
class WorldCopyableExtension(ContextExtension):
    """
    A context extension a plan's statechart context carries, which a trial carries over
    to the context it runs a candidate in, over a copy of the world.
    """

    def for_world(self, world: World) -> Self:
        """
        :param world: A copy of the world this extension's context runs in.
        :return: This extension, referring to `world` wherever it refers to the world.
        """
        return self


@dataclass
class RobotAccess(WorldCopyableExtension):
    """
    Gives the nodes of a statechart the robot that performs the plan.
    """

    robot: AbstractRobot
    """
    The robot performing the plan.
    """

    @property
    def controlled_root(self) -> KinematicStructureEntity:
        """
        :return: The topmost entity the robot may move, which a Cartesian goal is
            expressed relative to. Driving the base while manipulating moves the robot
            relative to the world, so the world root is the only frame that holds still
            then; otherwise the robot's own root does.
        """
        if (
            isinstance(self.robot, HasMobileBase)
            and self.robot.mobile_base.full_body_controlled
        ):
            return self.robot._world.root
        return self.robot.root

    def for_world(self, world: World) -> Self:
        return replace(self, robot=world.get_semantic_annotation_by_id(self.robot.id))


@dataclass
class StatementGrounding(WorldCopyableExtension):
    """
    How the underspecified statements of a plan are grounded into actions.
    """

    query_backend: QueryBackend = field(
        default_factory=EntityQueryLanguageGenerativeBackend
    )
    """
    The backend answering the underspecified statements.

    Defaults to the deterministic generative backend, since underspecified actions are
    constructed (generated).
    """

    candidates_to_try: int = 50
    """
    How many candidates an underspecified step tries before giving up, unless the step
    has a limit of its own.
    """

    sampling_seed: Optional[int] = None
    """
    Seed for the locations the plan's actions sample, so a run can be repeated;
    ``None`` samples afresh each run.
    """


@dataclass
class MotionToleranceConfig(WorldCopyableExtension):
    """
    Default goal-achievement tolerances for motions that leave their own thresholds
    unset.
    """

    default_tcp_position_threshold: float = 0.005
    """
    Default position tolerance in meters for tool-center-point poses, tighter than
    Giskard's own task default so an approach doesn't stop short of a small object.
    """

    tool_orientation_threshold: float = 0.02
    """
    Default orientation tolerance in rad for tool-center-point poses.

    .. note:: A physically simulated arm's PD-tracked joints settle with a small
        residual orientation error, so reusing the (much tighter) position tolerance
        as the rotation tolerance can leave the task perpetually unfinished, stalling
        the rest of the plan behind it.
    """


# %% what the executor tells the plan


@dataclass
class ExecutionMode(ContextExtension):
    """
    How the executor running a statechart executes it, added by that executor.
    """

    execution_type: ExecutionType
    """
    Whether the plan drives the real robot or a simulated one.
    """

    collision_avoidance: bool = False
    """
    Whether the robot avoids colliding with its surroundings and with itself.
    """
