from dataclasses import dataclass, field

import numpy as np
from typing_extensions import Callable, List

from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.spatial_types import FloatVariable, Point3


@dataclass(eq=False)
class AuxiliaryVariable(FloatVariable):
    name: PrefixedName = field(kw_only=True)
    provider: Callable[[], float] = field(kw_only=True)

    def resolve(self) -> float:
        return float(self.provider())


@dataclass
class AuxiliaryVariableManager:
    variables: List[AuxiliaryVariable] = field(default_factory=list)

    def create_float_variable(
        self, name: PrefixedName, provider: Callable[[], float] = None
    ) -> AuxiliaryVariable:
        v = AuxiliaryVariable(name=name, provider=provider)
        self.variables.append(v)
        return v

    def create_point3(
        self, name: PrefixedName, provider: Callable[[], List[float]] = None
    ) -> Point3:
        x = AuxiliaryVariable(
            name=PrefixedName("x", str(name)), provider=lambda: provider()[0]
        )
        y = AuxiliaryVariable(
            name=PrefixedName("y", str(name)), provider=lambda: provider()[1]
        )
        z = AuxiliaryVariable(
            name=PrefixedName("z", str(name)), provider=lambda: provider()[2]
        )
        self.variables.extend([x, y, z])
        return Point3(x, y, z)

    def resolve_auxiliary_variables(self) -> np.ndarray:
        return np.array([v.resolve() for v in self.variables])
