from __future__ import annotations

from enum import IntEnum, Enum
from typing import Union

import semantic_world.spatial_types.spatial_types as cas

goal_parameter = Union[str, float, bool, dict, list, IntEnum, None]


class LifeCycleState(IntEnum):
    not_started = 0
    running = 1
    paused = 2
    succeeded = 3
    failed = 4


class FloatEnum(float, Enum):
    """Enum where members are also (and must be) floats"""
    pass


class ObservationState(FloatEnum):
    false = cas.TrinaryFalse
    unknown = cas.TrinaryUnknown
    true = cas.TrinaryTrue
