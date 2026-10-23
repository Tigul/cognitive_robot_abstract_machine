"""
Draws the apartment's planar navigation map as a three-dimensional figure.

:class:`PlanarGraphOfBoundingBoxes` decomposes the apartment's free space on the floor
plane, while the obstacles constraining it stand in all three dimensions. The volume
figure draws three-dimensional geometry only, so the floor plan is raised into it here:
every convex set is extruded into a thin slab on the floor plane, and the path over them
is read at floor level.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import pytest
import rustworkx as rx
from typing_extensions import Dict, List, Set, Tuple

from coraplex.testing import setup_world
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    SemanticEnvironmentAnnotation,
)
from semantic_digital_twin.spatial_types import Point2
from semantic_digital_twin.spatial_types.spatial_types import (
    HomogeneousTransformationMatrix,
)
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.geometry import (
    PlanarBoundingBox,
    VolumetricBoundingBox,
)
from semantic_digital_twin.world_description.graph_of_convex_sets.boxes import (
    BoundingBoxAdjacency,
    PlanarGraphOfBoundingBoxes,
    VolumetricGraphOfBoundingBoxes,
)
from semantic_digital_twin.world_description.graph_of_convex_sets.plotting import (
    Footprint,
    NavigationPath,
    NavigationScene,
    Theme,
)
from semantic_digital_twin.world_description.graph_of_convex_sets.volume_figure import (
    GraphOfConvexSetsVolumeFigure,
)
from semantic_digital_twin.world_description.shape_collection import (
    BoundingBoxCollection,
)

from ..semantic_digital_twin_test.test_worlds.gcs_test_helpers import (
    hardest_path_query,
)

# %% how the map of the apartment is built

FLOOR_LEVEL = 0.0
"""
The height the apartment's floor lies at, and the bottom of the space the map is built
in.
"""

NAVIGABLE_HEIGHT = 1.8
"""
Height of the space above the floor a robot has to fit through, so anything reaching
into it blocks the footprint underneath.
"""

CLEARANCE = 0.3
"""
Amount every obstacle is bloated by while the map is built, so what the map shows as
free is what a base of that radius can pass through.
"""

SLAB_THICKNESS = 0.02
"""
Thickness a convex set of the floor plan is drawn with: thin enough to read as the plane
it decomposes, thick enough to be seen from the side.
"""

ENVIRONMENT_NAME = "apartment"
"""
The name the figure is titled with and its files are named after.
"""


@dataclass(frozen=True)
class FloorPlanInVolume:
    """
    A planar navigation map presented as the three-dimensional geometry the volume
    figure draws.

    Every convex set becomes a slab of :attr:`slab_thickness`, centered on the floor
    plane it was decomposed on, and every adjacency of the floor plan is carried over
    between the slabs rather than derived again from them.
    """

    navigation_map: PlanarGraphOfBoundingBoxes
    """
    The floor-plan decomposition of the apartment's free space.
    """

    search_space: BoundingBoxCollection[VolumetricBoundingBox, Point3]
    """
    The volume the map was built in, and the extent every panel of the figure is framed
    to.
    """

    slab_thickness: float = SLAB_THICKNESS
    """
    Thickness each convex set is drawn with.
    """

    def as_slabs(self) -> VolumetricGraphOfBoundingBoxes:
        """
        :return: The floor plan's convex sets and adjacencies, over slabs instead of
            floor-plan rectangles.
        """
        slabs = VolumetricGraphOfBoundingBoxes(
            self.navigation_map.world, self.search_space
        )
        slab_indices = self._add_slabs(slabs)
        for source, target, adjacency in self.navigation_map.graph.weighted_edge_list():
            slabs.graph.add_edge(
                slab_indices[source],
                slab_indices[target],
                BoundingBoxAdjacency(
                    intersection=self.slab_of(adjacency.intersection),
                    distance=adjacency.distance,
                ),
            )
        return slabs

    def _add_slabs(self, slabs: VolumetricGraphOfBoundingBoxes) -> Dict[int, int]:
        """
        :param slabs: The graph to add a slab per convex set to.
        :return: Which node of ``slabs`` each convex set of the floor plan became.
        """
        slab_indices = {}
        for index in self.navigation_map.graph.node_indices():
            slab = self.slab_of(self.navigation_map.graph[index])
            slabs.add_node(slab)
            slab_indices[index] = slabs.box_to_index_map[slab]
        return slab_indices

    def slab_of(self, box: PlanarBoundingBox) -> VolumetricBoundingBox:
        """
        :param box: The floor-plan rectangle to raise into the volume.
        :return: The slab drawn in its place.
        """
        return box.extrude(self.slab_thickness / 2)

    def scene(self, waypoints: List[Point2]) -> NavigationScene:
        """
        :param waypoints: The path over the floor plan, from its start to its goal.
        :return: The scene the figure draws: the slabs, the path over them at floor
            level, and the apartment's true collision geometry.
        """
        return NavigationScene(
            graph_of_convex_sets=self.as_slabs(),
            environment_name=ENVIRONMENT_NAME,
            path=NavigationPath(
                [waypoint.to_point3(FLOOR_LEVEL) for waypoint in waypoints]
            ),
            obstacles=self.obstacles(),
        )

    def obstacles(self) -> BoundingBoxCollection[VolumetricBoundingBox, Point3]:
        """
        :return: The collision geometry the map was built around, unbloated, so that the
            figure shows how far a convex set stays from what it has to avoid.
        """
        return SemanticEnvironmentAnnotation(
            root=self.navigation_map.world.root, _world=self.navigation_map.world
        ).build_bloated_obstacle_collection(
            self.search_space, obstacle_height_clearance=0.0
        )


def adjacency_footprints(
    graph: rx.PyGraph,
) -> Set[Tuple[Footprint, Footprint, Footprint]]:
    """
    :param graph: The graph of convex sets to read the edges of.
    :return: One triple per edge: the footprints of the two convex sets it connects and
        of the portal between them.
    """
    return {
        (
            Footprint.of(graph[source]),
            Footprint.of(graph[target]),
            Footprint.of(adjacency.intersection),
        )
        for source, target, adjacency in graph.weighted_edge_list()
    }


# %% the scene every test draws


@pytest.fixture(scope="module")
def apartment_world() -> World:
    """
    The apartment of :func:`setup_world`, holding a robot, its furniture and the objects
    standing on it.
    """
    return setup_world()


@pytest.fixture(scope="module")
def apartment_floor_plan(apartment_world: World) -> FloorPlanInVolume:
    """
    The apartment's navigation map, built over the whole area its collision geometry
    spans and up to the height a robot needs to pass through.
    """
    origin = HomogeneousTransformationMatrix(reference_frame=apartment_world.root)
    extent = (
        SemanticEnvironmentAnnotation(root=apartment_world.root, _world=apartment_world)
        .as_bounding_box_collection_at_origin(origin)
        .bounding_box()
    )
    search_space = BoundingBoxCollection(
        [
            VolumetricBoundingBox(
                extent.min_x,
                extent.min_y,
                FLOOR_LEVEL,
                extent.max_x,
                extent.max_y,
                FLOOR_LEVEL + NAVIGABLE_HEIGHT,
                origin,
            )
        ],
        apartment_world.root,
    )
    return FloorPlanInVolume(
        navigation_map=PlanarGraphOfBoundingBoxes.navigation_map_from_world(
            apartment_world, search_space=search_space, bloat_obstacles=CLEARANCE
        ),
        search_space=search_space,
    )


@pytest.fixture(scope="module")
def apartment_path(apartment_floor_plan: FloorPlanInVolume) -> List[Point2]:
    """
    The path across the apartment its two hardest-to-reach convex sets force.
    """
    query = hardest_path_query(apartment_floor_plan.navigation_map)
    return apartment_floor_plan.navigation_map.path_from_to(query.start, query.goal)


@pytest.fixture(scope="module")
def apartment_scene(
    apartment_floor_plan: FloorPlanInVolume, apartment_path: List[Point2]
) -> NavigationScene:
    """
    The apartment's navigation map raised into the volume its obstacles stand in.
    """
    return apartment_floor_plan.scene(apartment_path)


# %% what the figure is drawn from


def test_the_apartment_is_navigable_from_end_to_end(
    apartment_floor_plan: FloorPlanInVolume,
):
    """
    The map decomposes the apartment into a single connected component, so the figure
    shows one navigable floor plan rather than rooms cut off from each other.
    """
    assert len(rx.connected_components(apartment_floor_plan.navigation_map.graph)) == 1


def test_every_convex_set_is_drawn_as_a_slab_of_its_own_footprint(
    apartment_floor_plan: FloorPlanInVolume, apartment_scene: NavigationScene
):
    """
    Raising the floor plan into the volume gives every convex set a height to be seen
    at without moving it: the slabs keep the footprints and the count of the sets they
    stand for.
    """
    convex_sets = apartment_floor_plan.navigation_map.graph.nodes()
    slabs = apartment_scene.convex_sets

    assert len(slabs) == len(convex_sets)
    assert {Footprint.of(slab) for slab in slabs} == {
        Footprint.of(convex_set) for convex_set in convex_sets
    }
    assert all(
        (float(slab.min_z), float(slab.max_z))
        == (-SLAB_THICKNESS / 2, SLAB_THICKNESS / 2)
        for slab in slabs
    )


def test_the_drawn_graph_keeps_every_adjacency_of_the_floor_plan(
    apartment_floor_plan: FloorPlanInVolume, apartment_scene: NavigationScene
):
    """
    The edges are carried over from the floor plan rather than derived again from the
    slabs, so the figure draws the graph the path was planned over.
    """
    floor_plan_graph = apartment_floor_plan.navigation_map.graph

    assert adjacency_footprints(
        apartment_scene.graph_of_convex_sets.graph
    ) == adjacency_footprints(floor_plan_graph)
    assert len(apartment_scene.adjacencies) == floor_plan_graph.num_edges()


def test_the_path_is_drawn_at_floor_level(
    apartment_path: List[Point2], apartment_scene: NavigationScene
):
    """
    The path the floor plan returns has no height of its own, so it is drawn waypoint
    for waypoint on the floor the map decomposes.
    """
    assert apartment_scene.path.coordinates.tolist() == [
        [float(waypoint.x), float(waypoint.y)] for waypoint in apartment_path
    ]
    assert apartment_scene.path.spatial_coordinates[:, 2].tolist() == [
        FLOOR_LEVEL
    ] * len(apartment_path)


# %% the figure


def test_the_figure_spans_the_height_the_map_was_built_for(
    apartment_scene: NavigationScene,
):
    """
    What the figure draws of the decomposition is flat, so the height the panels frame
    has to come from the space the map was built in rather than from the slabs standing
    for its convex sets.
    """
    figure = GraphOfConvexSetsVolumeFigure(apartment_scene).render()

    assert figure.layout["scene1"]["zaxis"].range == pytest.approx(
        (FLOOR_LEVEL, FLOOR_LEVEL + NAVIGABLE_HEIGHT)
    )


def test_the_figure_is_written_as_an_interactive_page(
    apartment_scene: NavigationScene, tmp_path: Path
):
    """
    A single camera angle hides whatever it projects behind something else, and an
    apartment seen from outside is mostly its own walls, so the figure is written as a
    page that can be turned.
    """
    written = GraphOfConvexSetsVolumeFigure(
        apartment_scene, theme=Theme.LIGHT, image_formats=()
    ).save(tmp_path / "figures")

    assert [path.name for path in written] == [
        f"graph_of_convex_sets_volume_{ENVIRONMENT_NAME}_light.html"
    ]
    assert "plotly" in written[0].read_text()
