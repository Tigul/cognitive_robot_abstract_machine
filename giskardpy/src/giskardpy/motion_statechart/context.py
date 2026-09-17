from __future__ import annotations

from dataclasses import dataclass, field

from typing_extensions import Self, List, Optional

from semantic_digital_twin.collision_checking.collision_manager import CollisionManager
from semantic_digital_twin.collision_checking.collision_variable_managers import (
    BaseCollisionVariableManager,
    SelfCollisionVariableManager,
    ExternalCollisionVariableManager,
)
from giskardpy.qp.qp_controller_config import QPControllerConfig

from semantic_digital_twin.world import World
from cramph.context import StatechartContext


@dataclass
class MotionStatechartContext(StatechartContext):
    """
    Context used during the build phase of a MotionStatechartNode.
    """

    qp_controller_config: QPControllerConfig = field(
        default_factory=QPControllerConfig.create_with_simulation_defaults
    )
    """
    Optional configuration for the QP Controller.

    Is only needed when constraints are present in the motion statechart.
    """

    tick_duration: Optional[float] = field(init=False, default=None)
    """
    The control time step of :attr:`qp_controller_config`, None without one.
    """

    _self_collision_manager: Optional[SelfCollisionVariableManager] = field(
        init=False, default=None, repr=False, compare=False
    )
    """
    Backs :attr:`self_collision_manager`, None until a node requests it.
    """

    _external_collision_manager: Optional[ExternalCollisionVariableManager] = field(
        init=False, default=None, repr=False, compare=False
    )
    """
    Backs :attr:`external_collision_manager`, None until a node requests it.
    """

    def __post_init__(self):
        if self.qp_controller_config is None:
            return
        self.tick_duration = self.qp_controller_config.control_dt

    @property
    def collision_manager(self) -> CollisionManager:
        return self.world.collision_manager

    @property
    def self_collision_manager(self) -> SelfCollisionVariableManager:
        """
        SelfCollisionVariableManager shared by all self collision avoidance nodes,
        created on first access.
        """
        if self._self_collision_manager is None:
            self._self_collision_manager = SelfCollisionVariableManager(
                self.float_variable_data
            )
            self.collision_manager.add_collision_consumer(self._self_collision_manager)
        return self._self_collision_manager

    @property
    def external_collision_manager(self) -> ExternalCollisionVariableManager:
        """
        ExternalCollisionVariableManager shared by all external collision avoidance
        nodes, created on first access.
        """
        if self._external_collision_manager is None:
            self._external_collision_manager = ExternalCollisionVariableManager(
                self.float_variable_data
            )
            self.collision_manager.add_collision_consumer(
                self._external_collision_manager
            )
        return self._external_collision_manager

    @property
    def _registered_collision_variable_managers(
        self,
    ) -> List[BaseCollisionVariableManager]:
        """
        :return: The collision variable managers that nodes have requested so far.
        """
        return [
            manager
            for manager in (
                self._self_collision_manager,
                self._external_collision_manager,
            )
            if manager is not None
        ]

    @property
    def requires_collision_checking(self) -> bool:
        """
        :return: True if a node requested a collision variable manager and therefore
            needs collisions to be computed in every control cycle.
        """
        return len(self._registered_collision_variable_managers) > 0

    def cleanup(self):
        """
        Removes the lazy-initialized collision managers from the collision manager.
        """
        super().cleanup()
        for manager in self._registered_collision_variable_managers:
            self.collision_manager.remove_collision_consumer(manager)
        self._self_collision_manager = None
        self._external_collision_manager = None

    @classmethod
    def empty(cls) -> Self:
        return cls(
            world=World(),
            float_variable_data=None,
            qp_controller_config=None,
        )
