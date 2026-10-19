import numpy as np
import pytest

from semantic_digital_twin.datastructures.camera_resolution import CameraResolution
from semantic_digital_twin.exceptions import InvalidCameraResolutionError
from semantic_digital_twin.spatial_computations.exceptions import (
    RayEndpointCountMismatchError,
)
from semantic_digital_twin.spatial_computations.raytracer import RayTracer
from semantic_digital_twin.spatial_types.spatial_types import (
    HomogeneousTransformationMatrix,
)
from semantic_digital_twin.testing import ray_test_world
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.world_entity import Body

# %% helpers


def _place_body(world: World, body: Body, x: float, z: float = 0.0) -> None:
    """
    Move a body of the ray test world to a position on the world's x axis.
    """
    world.get_connection(world.root, body).origin = (
        HomogeneousTransformationMatrix.from_xyz_rpy(
            x=x, z=z, reference_frame=world.root, child_frame=body
        )
    )


# %% camera images


def test_create_segmentation_mask(ray_test_world):
    world, body1, body2, body3, body4 = ray_test_world
    rt = RayTracer(world)
    rt.update_scene()

    camera_pose = np.array(
        [
            [1.0, 0.0, 0.0, -2.5],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.2],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )

    world.get_connection(world.root, body2).origin = HomogeneousTransformationMatrix(
        np.array([[1, 0, 0, -1], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]),
        reference_frame=world.root,
        child_frame=body2,
    )

    resolution = CameraResolution(width=256, height=256)
    seg = rt.create_segmentation_mask(
        HomogeneousTransformationMatrix(camera_pose, reference_frame=world.root),
        resolution=resolution,
    )
    assert seg.shape == (resolution.width, resolution.height)

    hit, index, body = rt.ray_test(np.array([1, 0, 1]), np.array([-1, 0, 1]))
    assert hit is not None
    assert index is not None
    assert body is not None
    assert body1.index in seg
    assert body2.index in seg


def test_create_depth_map(ray_test_world):
    world, body1, body2, body3, body4 = ray_test_world
    rt = RayTracer(world)
    rt.update_scene()

    camera_pose = np.array(
        [
            [1.0, 0.0, 0.0, -2.5],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )

    resolution = CameraResolution(width=512, height=512)
    depth_map = rt.create_depth_map(
        HomogeneousTransformationMatrix(camera_pose, reference_frame=world.root),
        resolution=resolution,
    )
    assert depth_map is not None
    assert depth_map[0, 0] == -1  # Assuming no objects are hit at the upper left corner
    assert depth_map.shape == (resolution.width, resolution.height)
    assert depth_map.max() <= 2.5
    assert depth_map[depth_map > 0].min() >= 2.375


def test_create_camera_rays_with_rectangular_resolution(ray_test_world):
    world, *_ = ray_test_world
    ray_tracer = RayTracer(world)

    camera_pose = np.eye(4, dtype=float)
    camera_pose[:3, 3] = [-2.5, 0.0, 0.0]

    resolution = CameraResolution(width=128, height=64)
    ray_origins, ray_directions, pixels = ray_tracer.create_camera_rays(
        HomogeneousTransformationMatrix(camera_pose, reference_frame=world.root),
        resolution=resolution,
    )

    assert ray_origins.shape == (resolution.width * resolution.height, 3)
    assert ray_directions.shape == (resolution.width * resolution.height, 3)
    assert pixels.shape == (resolution.width * resolution.height, 2)
    assert set(pixels[:, 0]) == set(range(resolution.width))
    assert set(pixels[:, 1]) == set(range(resolution.height))


def test_create_segmentation_mask_with_rectangular_resolution(ray_test_world):
    world, *_ = ray_test_world
    ray_tracer = RayTracer(world)

    camera_pose = np.array(
        [
            [1.0, 0.0, 0.0, -2.5],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.2],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )

    resolution = CameraResolution(width=128, height=64)
    segmentation_mask = ray_tracer.create_segmentation_mask(
        HomogeneousTransformationMatrix(camera_pose, reference_frame=world.root),
        resolution=resolution,
    )

    assert segmentation_mask.shape == (resolution.width, resolution.height)


def test_create_depth_map_with_rectangular_resolution(ray_test_world):
    world, *_ = ray_test_world
    ray_tracer = RayTracer(world)

    camera_pose = np.array(
        [
            [1.0, 0.0, 0.0, -2.5],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )

    resolution = CameraResolution(width=128, height=64)
    depth_map = ray_tracer.create_depth_map(
        HomogeneousTransformationMatrix(camera_pose, reference_frame=world.root),
        resolution=resolution,
    )

    assert depth_map.shape == (resolution.width, resolution.height)


def test_camera_resolution_defaults_to_square_resolution():
    resolution = CameraResolution()

    assert resolution == CameraResolution(width=512, height=512)


@pytest.mark.parametrize(
    ("width", "height"),
    [(0, 64), (-1, 64), (64, 0), (64, -1)],
)
def test_non_positive_camera_resolution_raises_domain_exception(width, height):
    with pytest.raises(InvalidCameraResolutionError):
        CameraResolution(width=width, height=height)


# %% ray tests


def test_ray_test(ray_test_world):
    world, body1, body2, body3, body4 = ray_test_world
    rt = RayTracer(world)
    rt.update_scene()

    hit, index, body = rt.ray_test(np.array([1, 0, 0.1]), np.array([-1, 0, 0.1]))
    assert hit is not None
    assert index is not None
    assert body is not None
    assert body1 in body

    # Test with a ray that does not hit any object
    hit, index, body = rt.ray_test(np.array([10, 10, 10]), np.array([20, 20, 20]))
    assert not np.any(hit)
    assert not np.any(index)
    assert not np.any(body)


def test_ray_test_batch(ray_test_world):
    world, body1, body2, body3, body4 = ray_test_world
    world.get_connection(world.root, body1).origin = HomogeneousTransformationMatrix(
        np.array([[1, 0, 0, 1], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]),
        reference_frame=world.root,
        child_frame=body1,
    )
    world.get_connection(world.root, body2).origin = HomogeneousTransformationMatrix(
        np.array([[1, 0, 0, -1], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]),
        reference_frame=world.root,
        child_frame=body2,
    )

    rt = RayTracer(world)
    rt.update_scene()

    rays = np.array([[1, 1, 0.1], [-1, 1, 0.1]])
    targets = np.array([[1, -1, 0.1], [-1, -1, 0.1]])

    hits, indices, bodies = rt.ray_test(rays, targets)
    assert hits is not None
    assert indices is not None
    assert bodies is not None
    assert len(hits) == len(rays)
    assert len(indices) == len(rays)
    assert len(bodies) == len(rays)
    # Test return
    assert bodies[0] == body1
    assert bodies[1] == body2


def test_min_distance(ray_test_world):
    world, body1, body2, body3, body4 = ray_test_world
    world.get_connection(world.root, body1).origin = HomogeneousTransformationMatrix(
        np.array([[1, 0, 0, 0.5], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]),
        reference_frame=world.root,
        child_frame=body1,
    )
    world.get_connection(world.root, body2).origin = HomogeneousTransformationMatrix(
        np.array([[1, 0, 0, -1], [0, 1, 0, 0], [0, 0, 1, 10], [0, 0, 0, 1]]),
        reference_frame=world.root,
        child_frame=body2,
    )

    rt = RayTracer(world)
    rt.update_scene()

    rays = np.array([[0, 0, 0.1], [-1, 0, 0.1]])
    targets = np.array([[1, 0, 0.1], [1, 0, 0.1]])

    hits, indices, bodies = rt.ray_test(rays, targets, min_distance=1)

    assert len(hits) == 1
    assert bodies[0] == body1


def test_max_distance(ray_test_world):
    world, body1, body2, body3, body4 = ray_test_world
    world.get_connection(world.root, body1).origin = HomogeneousTransformationMatrix(
        np.array([[1, 0, 0, 1.5], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]),
        reference_frame=world.root,
        child_frame=body1,
    )
    world.get_connection(world.root, body2).origin = HomogeneousTransformationMatrix(
        np.array([[1, 0, 0, -1], [0, 1, 0, 0], [0, 0, 1, 10], [0, 0, 0, 1]]),
        reference_frame=world.root,
        child_frame=body2,
    )

    rt = RayTracer(world)
    rt.update_scene()

    rays = np.array([[0, 0, 0.1], [-1, 0, 0.1]])
    targets = np.array([[2, 0, 0.1], [2, 0, 0.1]])

    hits, indices, bodies = rt.ray_test(rays, targets)

    assert len(hits) == 2

    hits, indices, bodies = rt.ray_test(rays, targets, max_distance=1)

    assert len(hits) == 0


def test_ray_test_ignores_excluded_bodies(ray_test_world):
    world, body1, body2, *_ = ray_test_world
    _place_body(world, body1, x=1)
    _place_body(world, body2, x=-1)
    ray_tracer = RayTracer(world, excluded_bodies=[body1])

    _, _, bodies = ray_tracer.ray_test(np.array([3, 0, 0.1]), np.array([-3, 0, 0.1]))

    assert bodies == [body2]


def test_ray_test_follows_a_body_moved_after_the_previous_test(ray_test_world):
    world, body1, body2, *_ = ray_test_world
    _place_body(world, body1, x=1)
    _place_body(world, body2, x=-1)
    ray_tracer = RayTracer(world)
    ray_tracer.ray_test(np.array([3, 0, 0.1]), np.array([-3, 0, 0.1]))

    _place_body(world, body1, x=1, z=10)
    _, _, bodies = ray_tracer.ray_test(np.array([3, 0, 0.1]), np.array([-3, 0, 0.1]))

    assert bodies == [body2]


def test_multiple_hits_are_ordered_by_distance_along_each_ray(ray_test_world):
    world, body1, body2, *_ = ray_test_world
    _place_body(world, body1, x=1)
    _place_body(world, body2, x=-1)
    origin = np.array([3, 0, 0.1])
    box_half_size = body1.collision[0].scale.x / 2

    hits, _, bodies = RayTracer(world).ray_test(
        origin, np.array([-3, 0, 0.1]), multiple_hits=True
    )

    assert bodies == [body1, body1, body2, body2]
    np.testing.assert_allclose(
        np.linalg.norm(hits - origin, axis=1),
        [
            origin[0] - 1 - box_half_size,
            origin[0] - 1 + box_half_size,
            origin[0] + 1 - box_half_size,
            origin[0] + 1 + box_half_size,
        ],
        atol=1e-5,
    )


def test_ray_test_rejects_unequal_numbers_of_origins_and_targets(ray_test_world):
    world, *_ = ray_test_world

    with pytest.raises(RayEndpointCountMismatchError):
        RayTracer(world).ray_test(np.zeros((2, 3)), np.ones((3, 3)))


# %% distances along rays


def test_distances_reach_the_first_surface_each_ray_meets(ray_test_world):
    world, body1, body2, *_ = ray_test_world
    _place_body(world, body1, x=1)
    _place_body(world, body2, x=-1)
    origin = np.array([3, 0, 0.1])
    box_half_size = body1.collision[0].scale.x / 2

    distances = RayTracer(world).measure_distances(
        origin_points=[origin, origin],
        directions=[[-1, 0, 0], [1, 0, 0]],
    )

    np.testing.assert_allclose(
        distances, [origin[0] - 1 - box_half_size, np.inf], atol=1e-5
    )


def test_distances_skip_surfaces_closer_than_the_minimum_distance(ray_test_world):
    world, body1, body2, *_ = ray_test_world
    _place_body(world, body1, x=1)
    _place_body(world, body2, x=-1)
    origin = np.array([3, 0, 0.1])
    box_half_size = body1.collision[0].scale.x / 2

    [distance] = RayTracer(world).measure_distances(
        origin_points=[origin],
        directions=[[-1, 0, 0]],
        minimum_distance=origin[0] - 1 + box_half_size + 0.1,
    )

    assert distance == pytest.approx(origin[0] + 1 - box_half_size, abs=1e-5)


def test_distances_beyond_the_maximum_distance_are_infinite(ray_test_world):
    world, body1, *_ = ray_test_world
    _place_body(world, body1, x=1)

    distances = RayTracer(world).measure_distances(
        origin_points=[[3, 0, 0.1]], directions=[[-1, 0, 0]], maximum_distance=1.0
    )

    assert distances.tolist() == [np.inf]


# %% closest surface points


def test_closest_surface_point_lies_on_the_face_in_front_of_the_point(
    ray_test_world,
):
    world, body1, *_ = ray_test_world
    _place_body(world, body1, x=1)
    box_half_size = body1.collision[0].scale.x / 2

    surface = RayTracer(world).find_closest_surface_points([[3, 0, 0.05]])

    np.testing.assert_allclose(
        surface.positions, [[1 + box_half_size, 0, 0.05]], atol=1e-5
    )
    np.testing.assert_allclose(np.abs(surface.normals), [[1, 0, 0]], atol=1e-5)
