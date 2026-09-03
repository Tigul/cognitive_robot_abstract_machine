from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass

from typing_extensions import Type

from coraplex.datastructures.enums import DetectionTechnique
from coraplex.locations.factories import visibility_location
from coraplex.plans.factories import sequential, execute_single, try_in_order
from coraplex.robot_plans.actions.base import ActionDescription
from coraplex.robot_plans.actions.core.misc import DetectAction
from coraplex.robot_plans.actions.core.navigation import NavigateAction, LookAtAction
from krrood.entity_query_language.factories import a, variable
from semantic_digital_twin.spatial_types.spatial_types import Pose
from semantic_digital_twin.world_description.world_entity import SemanticAnnotation


@dataclass
class SearchAction(ActionDescription):
    """
    Searches for a target object around the given location.
    """

    target_location: Pose
    """
    Location around which to look for a target object.
    """

    object_semantic_annotation: Type[SemanticAnnotation]
    """
    Type of the object which is searched for.
    """

    @property
    def _action_plan(self):
        return sequential([a(NavigateAction)(
            target_location=variable(Pose,
                                     domain=visibility_location(
                                         target=self.target_location,
                                         context=self.context))),
            try_in_order([self._build_detection_plan(Pose.from_xyz_rpy(x, 1, 1, reference_frame=self.robot.root)) for x in [0, -0.5, 0.5]])
        ])

    def _build_detection_plan(self, target: Pose):
        return sequential([LookAtAction(target), DetectAction(DetectionTechnique.TYPES, object_sem_annotation=self.object_semantic_annotation)])

