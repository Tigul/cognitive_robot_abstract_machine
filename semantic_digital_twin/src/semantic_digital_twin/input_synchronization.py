from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field

from typing_extensions import List

from semantic_digital_twin.world import World

# %% base classes


@dataclass
class InputSynchronizer(ABC):
    """
    Writes an external source of truth, e.g. a robot's joint states, into the world
    state.
    """

    world: World
    """
    The world whose state is kept in sync with the external source.
    """

    @abstractmethod
    def apply(self) -> bool:
        """
        Write the most recent input into the world state.

        :return: Whether anything was written.
        """

    def close(self) -> None:
        """
        Release the resources used to receive inputs.
        """


@dataclass
class WorldStateInputs:
    """
    All inputs that one loop reads before it computes anything.
    """

    world: World
    """
    The world whose state is written and whose observers are notified.
    """

    synchronizers: List[InputSynchronizer] = field(default_factory=list)
    """
    The inputs, applied in the order they were added.
    """

    def apply_inputs(self) -> bool:
        """
        Write all inputs into the world state, in the order they were added.

        :return: Whether any of them wrote something.
        """
        wrote_something = False
        for synchronizer in self.synchronizers:
            wrote_something |= synchronizer.apply()
        return wrote_something

    def synchronize(self) -> None:
        """
        Write all inputs into the world state and announce the change.

        Nothing is announced when no input wrote, because announcing recomputes the
        forward kinematics and reaches every observer of the world.
        """
        if not self.apply_inputs():
            return
        self.announce_state()

    def synchronize_and_announce(self) -> None:
        """
        Write all inputs into the world state and announce the state even if no input
        wrote.

        Use this where nothing else announces, so that the observers of the world do not
        go stale.
        """
        self.apply_inputs()
        self.announce_state()

    def announce_state(self) -> None:
        """
        Hand the current world state to the observers of the world.

        Nothing is announced while the world model is being modified, because the
        observers would see an inconsistent model.
        """
        if self.world.world_is_being_modified:
            return
        self.world.notify_state_change()
