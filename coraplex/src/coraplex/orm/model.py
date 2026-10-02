from dataclasses import dataclass
from typing import Self

from krrood.ormatic.data_access_objects.alternative_mappings import (
    AlternativeMapping,
    T,
)
from typing_extensions import Optional

from coraplex.datastructures.enums import Arms
from coraplex.datastructures.grasp import GraspPose, GraspDescription
from semantic_digital_twin.orm.model import PoseMapping

# ----------------------------------------------------------------------------------------------------------------------
#            Map all Designators, that are not self-mapping, here.
#            By default all classes are self-mapping, so you only need to add the ones where not every attribute is
#            supposed to be mapped or where an attribute is from a type, which is not mapped itself.
#            Specify the columns(attributes) that are supposed to be tracked in the database.
#            One attribute equals one column. Please refer to the ORMatic documentation for more information.
# ----------------------------------------------------------------------------------------------------------------------


@dataclass(eq=False)
class GrasPoseMapping(PoseMapping, AlternativeMapping[GraspPose]):
    arm: Optional[Arms]

    grasp_description: Optional[GraspDescription]

    @classmethod
    def from_domain_object(cls, obj: GraspPose) -> Self:
        position = obj.to_position()
        orientation = obj.to_quaternion()
        result = cls(
            position=position,
            orientation=orientation,
            reference_frame=obj.reference_frame,
            grasp_description=obj.grasp_description,
            arm=obj.arm,
        )
        return result

    def to_domain_object(self) -> T:
        return GraspPose(
            position=self.position,
            orientation=self.orientation,
            reference_frame=self.reference_frame,
            grasp_description=self.grasp_description,
            arm=self.arm,
        )
