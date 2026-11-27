from __future__ import annotations

from copy import deepcopy

import pytest
from krrood.ormatic.data_access_objects.helper import to_dao

from krrood_spatial_math.spatial_types import (
    HomogeneousTransformationMatrix,
    Point2,
    Point3,
)
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.exceptions import MissingMovableJointError
from semantic_digital_twin.reasoning.predicates import InsideRegion
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    Bathroom,
    BathroomCabin,
    Elevator,
    FirstFloor,
    Floor,
    GroundFloor,
    Level,
    Room,
)
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import FixedConnection
from semantic_digital_twin.world_description.geometry import Box, Scale
from semantic_digital_twin.world_description.shape_collection import ShapeCollection
from semantic_digital_twin.world_description.world_entity import Body, Region

# %% a building with an elevator


PERSON_SCALE = Scale(0.4, 0.4, 1.7)
"""
The size of an adult standing upright.
"""


@pytest.fixture
def building(multi_story_building: World) -> World:
    """
    A copy of the two-storey building with an elevator, which a test may change.
    """
    return deepcopy(multi_story_building)


@pytest.fixture
def elevator(building: World) -> Elevator:
    [elevator] = building.get_semantic_annotations_by_type(Elevator)
    return elevator


@pytest.fixture
def ground_floor(building: World) -> GroundFloor:
    [ground_floor] = building.get_semantic_annotations_by_type(GroundFloor)
    return ground_floor


@pytest.fixture
def first_floor(building: World) -> FirstFloor:
    [first_floor] = building.get_semantic_annotations_by_type(FirstFloor)
    return first_floor


def person_standing_at(world: World, point: Point3) -> Body:
    """
    :return: A body the size of an adult, standing with its feet on ``point``.
    """
    world_P_feet = world.transform(point, world.root)
    person = Body(name=PrefixedName(f"person_{len(world.bodies)}"))
    person.collision = ShapeCollection(
        [
            Box(
                origin=HomogeneousTransformationMatrix(reference_frame=person),
                scale=PERSON_SCALE,
            )
        ]
    )
    with world.modify_world():
        world.add_connection(
            FixedConnection(
                parent=world.root,
                child=person,
                parent_T_connection_expression=HomogeneousTransformationMatrix.from_xyz_rpy(
                    x=float(world_P_feet.x),
                    y=float(world_P_feet.y),
                    z=float(world_P_feet.z) + PERSON_SCALE.z / 2,
                ),
            )
        )
    return person


def floor_height(level: Level) -> float:
    return float(level.floor_plane[0].z)


def cabin_centre_on(elevator: Elevator, level: Level) -> Point3:
    """
    :return: The middle of the cabin's footprint, at the height of ``level``'s floor.
    """
    world_P_cabin = elevator._world.transform(
        Point3(reference_frame=elevator.root), elevator._world.root
    )
    return Point3(
        world_P_cabin.x,
        world_P_cabin.y,
        floor_height(level),
        reference_frame=elevator._world.root,
    )


def in_front_of_doors_on(elevator: Elevator, level: Level, distance: float) -> Point3:
    """
    :return: The point ``distance`` in front of the middle of the doors, on ``level``'s
        floor.
    """
    world = elevator._world
    world_P_front = world.transform(
        elevator.pose_in_front_of_doors(distance).position, world.root
    )
    return Point3(
        world_P_front.x,
        world_P_front.y,
        floor_height(level),
        reference_frame=world.root,
    )


def bounds_in_world(region: Region) -> list[float]:
    """
    :return: The region's x, y and z extents in the frame of its world's root.
    """
    box = region.area.as_bounding_box_collection_in_frame(
        region._world.root
    ).bounding_box()
    return [box.min_x, box.max_x, box.min_y, box.max_y, box.min_z, box.max_z]


# %% the cabin


def test_a_person_inside_the_cabin_is_in_the_cabin_area(elevator, ground_floor):
    person = person_standing_at(
        elevator._world, cabin_centre_on(elevator, ground_floor)
    )

    assert InsideRegion(body=person, region=elevator.cabin_area)()


def test_a_person_in_front_of_the_doors_is_not_in_the_cabin_area(
    elevator, ground_floor
):
    person = person_standing_at(
        elevator._world, in_front_of_doors_on(elevator, ground_floor, 1.0)
    )

    assert not InsideRegion(body=person, region=elevator.cabin_area)()


def test_the_cabin_area_moves_with_the_cabin(elevator, ground_floor, first_floor):
    upstairs = person_standing_at(
        elevator._world, cabin_centre_on(elevator, first_floor)
    )
    downstairs = person_standing_at(
        elevator._world, cabin_centre_on(elevator, ground_floor)
    )

    elevator.drive_to_floor(first_floor)

    assert InsideRegion(body=upstairs, region=elevator.cabin_area)()
    assert not InsideRegion(body=downstairs, region=elevator.cabin_area)()


def test_the_cabin_area_is_the_inside_of_the_cabin(elevator):
    cabin = elevator.root.collision.as_bounding_box_collection_in_frame(
        elevator._world.root
    ).bounding_box()

    assert bounds_in_world(elevator.cabin_area) == pytest.approx(
        [cabin.min_x, cabin.max_x, cabin.min_y, cabin.max_y, cabin.min_z, cabin.max_z]
    )


def test_the_cabin_area_enters_the_world_once_on_first_access(building, elevator):
    regions_before = list(building.regions)

    cabin_area = elevator.cabin_area

    assert elevator.cabin_area is cabin_area
    assert list(building.regions) == regions_before + [cabin_area]


# %% the doorway


def test_the_doorway_lies_between_the_doors_on_the_cabin_floor(elevator, ground_floor):
    world = elevator._world
    world_P_doorway = world.transform(elevator.doorway_pose.position, world.root)
    world_P_doors = [
        world.transform(Point3(reference_frame=door.leaf), world.root)
        for door in elevator.doors
    ]

    assert float(world_P_doorway.x) == pytest.approx(
        sum(float(door.x) for door in world_P_doors) / len(world_P_doors)
    )
    assert float(world_P_doorway.y) == pytest.approx(
        sum(float(door.y) for door in world_P_doors) / len(world_P_doors)
    )
    assert float(world_P_doorway.z) == pytest.approx(floor_height(ground_floor))


def test_a_pose_in_front_of_the_doors_lies_that_far_out_of_the_doorway(elevator):
    distance = 1.25

    doorway_P_front = elevator._world.transform(
        elevator.pose_in_front_of_doors(distance).position,
        elevator.root,
    )
    cabin_P_doorway = elevator.doorway_pose.position

    assert float(cabin_P_doorway.x - doorway_P_front.x) == pytest.approx(distance)
    assert float(doorway_P_front.y) == pytest.approx(float(cabin_P_doorway.y))
    assert float(doorway_P_front.z) == pytest.approx(float(cabin_P_doorway.z))


# %% waiting areas


def test_a_person_in_front_of_the_doors_waits_on_its_level(
    elevator, ground_floor, first_floor
):
    person = person_standing_at(
        elevator._world,
        in_front_of_doors_on(elevator, ground_floor, elevator.waiting_area_scale.x / 2),
    )

    assert InsideRegion(body=person, region=elevator.waiting_areas[ground_floor])()
    assert not InsideRegion(body=person, region=elevator.waiting_areas[first_floor])()


def test_a_person_inside_the_cabin_is_not_waiting(elevator, ground_floor):
    person = person_standing_at(
        elevator._world, cabin_centre_on(elevator, ground_floor)
    )

    assert not InsideRegion(body=person, region=elevator.waiting_areas[ground_floor])()


def test_a_person_beyond_the_waiting_depth_is_not_waiting(elevator, ground_floor):
    person = person_standing_at(
        elevator._world,
        in_front_of_doors_on(elevator, ground_floor, elevator.waiting_area_scale.x + 1),
    )

    assert not InsideRegion(body=person, region=elevator.waiting_areas[ground_floor])()


def test_a_waiting_area_stands_on_its_levels_floor(elevator, first_floor):
    bounds = bounds_in_world(elevator.waiting_areas[first_floor])

    assert bounds[4] == pytest.approx(floor_height(first_floor))


def test_a_waiting_area_stays_on_its_level_while_the_cabin_rides(
    elevator, ground_floor, first_floor
):
    bounds_before = bounds_in_world(elevator.waiting_areas[ground_floor])

    elevator.drive_to_floor(first_floor)

    assert bounds_in_world(elevator.waiting_areas[ground_floor]) == pytest.approx(
        bounds_before
    )


def test_a_waiting_area_has_the_elevators_waiting_area_scale(elevator, ground_floor):
    assert elevator.waiting_areas[ground_floor].area[0].scale == (
        elevator.waiting_area_scale
    )


def test_a_waiting_area_is_fixed_to_the_shaft(elevator, ground_floor):
    waiting_area = elevator.waiting_areas[ground_floor]

    assert waiting_area.parent_kinematic_structure_entity is (
        elevator.movable_joint.parent
    )


def test_an_elevator_has_a_waiting_area_on_every_level(
    elevator, ground_floor, first_floor
):
    assert set(elevator.waiting_areas) == {ground_floor, first_floor}


def test_the_waiting_areas_enter_the_world_once_on_first_access(building, elevator):
    regions_before = list(building.regions)

    waiting_areas = elevator.waiting_areas

    assert elevator.waiting_areas is waiting_areas
    assert list(building.regions) == regions_before + list(waiting_areas.values())


def test_an_elevator_without_a_drive_has_no_waiting_areas():
    world = World.create_with_root_body("root")
    with world.modify_world():
        elevator = Elevator.create_with_new_body_in_world(
            name="elevator", world=world, scale=Scale(2, 2, 2)
        )

    with pytest.raises(MissingMovableJointError) as raised:
        elevator.waiting_areas

    assert raised.value.semantic_annotation is elevator


# %% areas of a rebuilt world


def test_a_rebuilt_world_finds_the_areas_it_already_holds(elevator, ground_floor):
    cabin_area_name = elevator.cabin_area.name
    waiting_area_name = elevator.waiting_areas[ground_floor].name
    rebuilt = deepcopy(elevator._world)
    regions_before = list(rebuilt.regions)
    [rebuilt_elevator] = rebuilt.get_semantic_annotations_by_type(Elevator)
    [rebuilt_ground_floor] = rebuilt.get_semantic_annotations_by_type(GroundFloor)

    assert rebuilt_elevator.cabin_area.name == cabin_area_name
    assert rebuilt_elevator.waiting_areas[rebuilt_ground_floor].name == (
        waiting_area_name
    )
    assert list(rebuilt.regions) == regions_before


# %% levels


def test_a_level_holds_a_person_standing_in_it(
    elevator, building, ground_floor, first_floor
):
    person = person_standing_at(
        building, in_front_of_doors_on(elevator, ground_floor, 3.0)
    )

    assert Level.holding(person) == [ground_floor]


def test_the_level_under_a_person_is_the_highest_whose_floor_lies_below_its_origin(
    elevator, building, first_floor
):
    person = person_standing_at(
        building, in_front_of_doors_on(elevator, first_floor, 3.0)
    )

    assert Level.under(person) is first_floor


def test_no_level_is_under_a_person_below_every_level(elevator, building):
    person = person_standing_at(
        building, Point3(0.0, 0.0, -5.0, reference_frame=building.root)
    )

    assert Level.under(person) is None


def test_a_level_is_named_as_people_speak_of_it():
    assert GroundFloor.spoken_name == "ground floor"


def test_the_free_space_of_a_level_spans_its_footprint_around_obstacles():
    world = World.create_with_root_body("root")
    with world.modify_world():
        level = GroundFloor.create_with_new_region_in_world(
            name="ground_floor",
            world=world,
            world_root_T_self=HomogeneousTransformationMatrix.from_xyz_rpy(z=1.5),
            scale=Scale(4, 4, 3),
        )
    person_standing_at(world, Point3(1.0, 1.0, 0.0, reference_frame=world.root))

    free_space = level.planar_free_space(max_height=2.0)

    assert free_space.is_free(point=Point2(-1.0, -1.0, reference_frame=world.root))
    assert not free_space.is_free(point=Point2(1.0, 1.0, reference_frame=world.root))
    assert not free_space.is_free(point=Point2(3.0, 3.0, reference_frame=world.root))


# %% floors and rooms


@pytest.fixture
def two_stacked_floors() -> tuple[Floor, Floor]:
    """
    A floor at the bottom of a world and another three meters above it.
    """
    world = World.create_with_root_body("root")
    with world.modify_world():
        lower = Floor.create_with_new_body_in_world(
            name="lower", world=world, scale=Scale(4, 4, 0.1)
        )
        upper = Floor.create_with_new_body_in_world(
            name="upper",
            world=world,
            world_root_T_self=HomogeneousTransformationMatrix.from_xyz_rpy(z=3.0),
            scale=Scale(4, 4, 0.1),
        )
    return lower, upper


def standing_on(floor: Floor, x: float = 0.0) -> Body:
    """
    :return: A person standing on top of ``floor``, ``x`` along the world's x-axis.
    """
    world = floor._world
    return person_standing_at(
        world, Point3(x, 0.0, floor.extent.max_z, reference_frame=world.root)
    )


def test_a_floors_extent_is_the_box_around_its_own_shapes(two_stacked_floors):
    lower, _ = two_stacked_floors
    shapes = lower.root.collision.as_bounding_box_collection_in_frame(
        lower._world.root
    ).bounding_box()

    assert [lower.extent.min_z, lower.extent.max_z] == pytest.approx(
        [shapes.min_z, shapes.max_z]
    )
    assert [lower.extent.min_x, lower.extent.max_x] == pytest.approx(
        [shapes.min_x, shapes.max_x]
    )


def test_the_floor_under_a_person_is_the_topmost_below_it(two_stacked_floors):
    lower, upper = two_stacked_floors

    assert Floor.under(standing_on(upper)) is upper
    assert Floor.under(standing_on(lower)) is lower


def test_no_floor_is_under_a_person_beside_every_floor(two_stacked_floors):
    lower, _ = two_stacked_floors

    assert Floor.under(standing_on(lower, x=10.0)) is None


def test_the_floor_centre_of_a_room_lies_on_top_of_the_middle_of_its_floor(
    two_stacked_floors,
):
    _, upper = two_stacked_floors
    world = upper._world
    room = Room(name=PrefixedName("office"), floor=upper)
    with world.modify_world():
        world.add_semantic_annotation(room)
    shapes = upper.root.collision.as_bounding_box_collection_in_frame(
        world.root
    ).bounding_box()

    world_P_centre = world.transform(room.floor_centre.position, world.root)

    assert [
        float(world_P_centre.x),
        float(world_P_centre.y),
        float(world_P_centre.z),
    ] == pytest.approx(
        [
            (shapes.min_x + shapes.max_x) / 2,
            (shapes.min_y + shapes.max_y) / 2,
            shapes.max_z,
        ]
    )


def test_a_bathroom_keeps_its_toilet_cabins_through_the_database(two_stacked_floors):
    lower, upper = two_stacked_floors
    world = lower._world
    cabin = BathroomCabin(name=PrefixedName("cabin"), floor=upper)
    bathroom = Bathroom(name=PrefixedName("bathroom"), floor=lower, cabins=[cabin])
    with world.modify_world():
        world.add_semantic_annotation(cabin)
        world.add_semantic_annotation(bathroom)

    rebuilt = to_dao(world).from_dao()

    [rebuilt_bathroom] = rebuilt.get_semantic_annotations_by_type(Bathroom)
    assert [rebuilt_cabin.name for rebuilt_cabin in rebuilt_bathroom.cabins] == [
        cabin.name
    ]
