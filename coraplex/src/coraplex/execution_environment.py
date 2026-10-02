from __future__ import annotations

import logging
from dataclasses import dataclass, field

from typing_extensions import ClassVar, Optional

from coraplex.datastructures.enums import ExecutionType

logger = logging.getLogger(__name__)


@dataclass
class ExecutionEnvironment:
    """
    Base class for managing execution context of all actions within.

    Instances of this class is to be used with a "with" context block

    Example:

        >>> with ExecutionEnvironment(ExecutionType.SIMULATED):
        >>>     SequentialPlan(context, NavigateActionDescription, ...)
    """

    execution_type: ExecutionType
    """
    The type of the execution environment.
    """

    collision_avoidance: bool = False
    """
    Whether the robot avoids colliding with its surroundings and with itself in every
    motion state chart created within this environment.
    """

    current_execution_type: ClassVar[Optional[ExecutionType]] = None
    """
    The execution type of the innermost environment entered, None outside of every
    environment.
    """

    current_collision_avoidance: ClassVar[bool] = False
    """
    Whether the innermost environment entered avoids collisions.
    """

    previous_type: ExecutionType = field(init=False, default=None)
    """
    Type of the execution environment before setting it, used for nested environments.
    """

    previous_collision_avoidance: bool = field(init=False, default=False)
    """
    Collision avoidance setting before entering this environment, used for nested
    environments.
    """

    def __enter__(self):
        """
        Make this environment the current one, remembering the one it replaces.
        """
        self.previous_type = ExecutionEnvironment.current_execution_type
        self.previous_collision_avoidance = (
            ExecutionEnvironment.current_collision_avoidance
        )
        ExecutionEnvironment.current_execution_type = self.execution_type
        ExecutionEnvironment.current_collision_avoidance = self.collision_avoidance

    def __exit__(self, _type, value, traceback):
        """
        Make the environment this one replaced the current one again.
        """
        ExecutionEnvironment.current_execution_type = self.previous_type
        ExecutionEnvironment.current_collision_avoidance = (
            self.previous_collision_avoidance
        )

    def __call__(self, collision_avoidance: bool = False):
        """
        Configure the environment for use as a context manager, allowing ``with
        simulated_robot(collision_avoidance=True):``.
        """
        self.collision_avoidance = collision_avoidance
        return self


# These are imported, so they don't have to be initialized when executing with
simulated_robot = ExecutionEnvironment(ExecutionType.SIMULATED)
real_robot = ExecutionEnvironment(ExecutionType.REAL)
semi_real_robot = ExecutionEnvironment(ExecutionType.SEMI_REAL)
no_execution = ExecutionEnvironment(ExecutionType.NO_EXECUTION)
