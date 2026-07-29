from __future__ import annotations

from dataclasses import dataclass
from typing_extensions import TYPE_CHECKING, Type, List

from giskardpy.motion_statechart.graph_node import MotionStatechartNode
from krrood.entity_query_language.factories import ConditionType, get_false_statements
from krrood.exceptions import DataclassException
from coraplex.datastructures.enums import Arms, ExecutionType
from coraplex.plans.failures import PlanFailure

if TYPE_CHECKING:
    from coraplex.failure_handling.failure_refiner import FailureDetector
    from coraplex.failure_handling.failure_handling_strategy import (
        FailureHandlingStrategy,
    )
    from coraplex.plans.designator import Designator
    from coraplex.robot_plans.actions.base import ActionDescription
    from semantic_digital_twin.robots.robot_parts import AbstractRobot
    from semantic_digital_twin.world_description.world_entity import (
        KinematicStructureEntity,
    )


@dataclass
class ContextIsUnavailable(DataclassException):
    """
    Raised when an instance that tries to access the context of a plan has no reference
    to the plan.

    Most likely raised when an action created a subplan without calling
    `ActionDescription.add_subplan`
    """

    instance: Designator
    """
    The instance where the plan node is None.
    """

    def error_message(self) -> str:
        return f"{self.instance} has no plan node."

    def suggest_correction(self) -> str:
        return (
            "did you forget to call `add_subplan` when creating plans inside actions?"
        )


@dataclass
class TipLinkDoesNotMatchAnyArm(DataclassException):
    """
    Raised when a reachability validator's tip link is not the tool frame of any arm of
    the robot, so no arm can be selected to reach the requested pose.
    """

    tip_link: KinematicStructureEntity
    """
    The tip link that did not match any arm.
    """

    robot: AbstractRobot
    """
    The robot whose arms were searched.
    """

    def error_message(self) -> str:
        return f"tip_link {self.tip_link} does not match any arm of {self.robot}"

    def suggest_correction(self) -> str:
        return "ensure the tip_link is the tool frame of one of the robot's arms."


@dataclass
class MissingWaypoints(DataclassException):
    """
    Raised when a waypoint motion or tool action produced no waypoints to follow.
    """

    instance: Designator
    """
    The designator that has no waypoints.
    """

    def error_message(self) -> str:
        return f"{self.instance} has no waypoints to follow."

    def suggest_correction(self) -> str:
        return "ensure the motion sequence samples at least one point."


@dataclass
class WipingTargetMissing(DataclassException):
    """
    Raised when a wiping action is created without a surface to wipe.
    """

    instance: Designator
    """
    The wiping action that has no target.
    """

    def error_message(self) -> str:
        return f"{self.instance} has neither a container nor a target pose."

    def suggest_correction(self) -> str:
        return "provide either a container body or a target pose to wipe."


@dataclass
class MissingToolFrame(DataclassException):
    """
    Raised when no tool frame is available for the requested arm.
    """

    arm: Arms
    """
    The arm whose tool frame was requested.
    """

    robot: AbstractRobot
    """
    The robot whose arm was searched.
    """

    def error_message(self) -> str:
        return f"no tool frame available for arm {self.arm} of {self.robot}"

    def suggest_correction(self) -> str:
        return "ensure the arm's end effector defines a tool frame."


@dataclass
class ConditionNotSatisfied(PlanFailure):

    pre_condition: bool
    action: Type[ActionDescription]
    condition: ConditionType

    def error_message(self) -> str:
        prefix = "Pre" if self.pre_condition else "Post"
        if isinstance(self.condition, bool):
            return f"{prefix}-Condition for Action '{self.action.__name__}' is not satisfied"
        false_statements = get_false_statements(self.condition)
        return f"{prefix}-Condition for Action '{self.action.__name__}' is not satisfied, following statements are false: {[s._name_ for s in false_statements]}"

    def suggest_correction(self) -> str:
        return ""


@dataclass
class MotionDidNotFinish(PlanFailure):

    failed_motions: List[MotionStatechartNode]

    def error_message(self) -> str:
        return f"Motion did not finish, following motions failed: {self.failed_motions}"

    def suggest_correction(self) -> str:
        return ""


@dataclass
class AmbiguousFailureDetector(DataclassException):
    """
    Raised when several failure detectors are equally specific for the same failure, so
    the refiner cannot decide which one refines it.
    """

    failure: PlanFailure
    """
    The failure that several detectors claim with the same specificity.
    """

    detectors: List[FailureDetector]
    """
    The detectors that are equally specific for the failure.
    """

    def error_message(self) -> str:
        return f"Detectors {self.detectors} are equally specific for {self.failure}"

    def suggest_correction(self) -> str:
        return (
            "narrow one detector down by declaring a more specific input failure type or "
            "more required parameter mixins."
        )


@dataclass
class FailureRefinementCycle(DataclassException):
    """
    Raised when a chain of failure detectors refines a failure back into a failure type
    it already produced, which would refine forever.
    """

    failure: PlanFailure
    """
    The failure whose refinement cycled.
    """

    repeated_failure_type: Type[PlanFailure]
    """
    The failure type the chain produced for the second time.
    """

    def error_message(self) -> str:
        return (
            f"Refining {self.failure} produced {self.repeated_failure_type.__name__} "
            f"again."
        )

    def suggest_correction(self) -> str:
        return "ensure the detectors refine failures towards more specific types only."


@dataclass
class AmbiguousFailureHandlingStrategy(DataclassException):
    """
    Raised when several failure handling strategies are equally specific for the same
    failure, so the handler cannot decide which one resolves it.
    """

    failure: PlanFailure
    """
    The failure that several strategies claim with the same specificity.
    """

    strategies: List[FailureHandlingStrategy]
    """
    The strategies that are equally specific for the failure.
    """

    def error_message(self) -> str:
        return f"Strategies {self.strategies} are equally specific for {self.failure}"

    def suggest_correction(self) -> str:
        return (
            "narrow one strategy down by declaring a more specific handled failure "
            "type."
        )


@dataclass
class UnknownExecutionType(DataclassException):
    """
    Raised when an executable is run with an execution type it does not handle.
    """

    execution_type: ExecutionType
    """
    The execution type that is not supported.
    """

    def error_message(self) -> str:
        return f"Unknown execution type: {self.execution_type}"

    def suggest_correction(self) -> str:
        return ""
