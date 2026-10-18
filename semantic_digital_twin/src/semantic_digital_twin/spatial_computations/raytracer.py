from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
import numpy.typing as npt
import trimesh
from trimesh import Scene
from typing_extensions import TYPE_CHECKING

from semantic_digital_twin.datastructures.camera_resolution import CameraResolution
from semantic_digital_twin.datastructures.field_of_view import FieldOfView
from semantic_digital_twin.spatial_computations.exceptions import (
    RayEndpointCountMismatchError,
)
from semantic_digital_twin.spatial_types.spatial_types import GenericSpatialType
from semantic_digital_twin.world_description.geometry import Shape
from semantic_digital_twin.world_description.world_entity import Body

if TYPE_CHECKING:
    from open3d.t.geometry import RaycastingScene

    from semantic_digital_twin.world import World

# %% collision geometry placed in the world


@dataclass
class PlacedCollisionShape:
    """
    A collision shape of a body together with where the world currently holds it.
    """

    body: Body
    """
    The body the shape belongs to.
    """

    shape: Shape
    """
    The collision shape.
    """

    world_T_shape: npt.NDArray[np.float64]
    """
    The pose of the shape in the world root's frame.
    """


# %% ray tracer


@dataclass(eq=False)
class RayTracer:
    """
    Casts rays into the collision geometry of a world and renders camera views of it.
    """

    world: World
    """
    The world to use for ray tracing.
    """

    excluded_bodies: list[Body] = field(default_factory=list)
    """
    Bodies whose collision geometry rays pass through without a hit.

    Only ray tests honour this; :attr:`scene` still holds these bodies.
    """

    scene: Scene = field(init=False, default_factory=Scene)
    """
    The trimesh scene mirroring the world, used for camera rays and rendering.
    """

    _last_world_model: int = field(init=False, default=-1)
    """
    Last model version of the world to which :attr:`scene` was updated.
    """

    _last_world_state: int = field(init=False, default=-1)
    """
    Last state version of the world to which :attr:`scene` was updated.
    """

    _raycasting_scene: RaycastingScene | None = field(init=False, default=None)
    """
    The open3d scene rays are intersected with, built from :attr:`_placed_shapes`.
    """

    _placed_shapes: list[PlacedCollisionShape] = field(init=False, default_factory=list)
    """
    The shapes :attr:`_raycasting_scene` holds, indexed by their open3d geometry id.
    """

    _raycasting_scene_model_version: int = field(init=False, default=-1)
    """
    The world model version :attr:`_raycasting_scene` was built for.
    """

    _raycasting_scene_state_version: int = field(init=False, default=-1)
    """
    The world state version :attr:`_raycasting_scene` was last checked against.
    """

    def __post_init__(self):
        self.update_scene()

    def update_scene(self):
        """
        Updates :attr:`scene` with the current state of the world.

        Camera rays and rendering read :attr:`scene`, so this has to be called whenever
        the world changed before they are used.
        """
        if self._last_world_model != self.world.get_world_model_manager().version:
            self.add_missing_bodies()
            self._last_world_model = self.world.get_world_model_manager().version
        if self._last_world_state != self.world.state.version:
            self.update_transforms()
            self._last_world_state = self.world.state.version

    def add_missing_bodies(self):
        """
        Adds all bodies from the world to :attr:`scene` that are not already present.
        """
        # Bodies are added to the scene with their name as the node name plus a suffix for collision geometries.
        # We check if a body is not in the complete list of all node names in the scene graph.
        # If the body is not present, we add it to the scene.
        bodies_to_add = [
            body
            for body in self.world.bodies
            if body.name.name not in "\t".join(self.scene.graph.nodes)
        ]
        for body in bodies_to_add:
            for i, collision in enumerate(body.collision):
                self.scene.add_geometry(
                    collision.mesh,
                    node_name=body.name.name + f"_collision_{i}",
                    parent_node_name="world",
                    transform=self.world.compute_forward_kinematics_np(
                        self.world.root, body
                    )
                    @ collision.origin.to_np(),
                )

    def update_transforms(self):
        """
        Updates the transforms of all bodies in :attr:`scene`.
        """
        for body in self.world.bodies:
            for i, collision in enumerate(body.collision):
                transform = (
                    self.world.compute_forward_kinematics_np(self.world.root, body)
                    @ collision.origin.to_np()
                )
                self.scene.graph[body.name.name + f"_collision_{i}"] = transform

    def _update_raycasting_scene(self) -> None:
        """
        Rebuilds :attr:`_raycasting_scene` if the world model changed or a shape it
        holds moved since it was built.
        """
        model_version = self.world.get_world_model_manager().version
        state_version = self.world.state.version
        is_model_unchanged = model_version == self._raycasting_scene_model_version
        if is_model_unchanged and state_version == self._raycasting_scene_state_version:
            return
        self._raycasting_scene_state_version = state_version

        placed_shapes = self._place_collision_shapes()
        if is_model_unchanged and self._shapes_stayed_in_place(placed_shapes):
            return
        self._build_raycasting_scene(placed_shapes)
        self._raycasting_scene_model_version = model_version

    def _place_collision_shapes(self) -> list[PlacedCollisionShape]:
        """
        :return: Every collision shape of the bodies rays can hit, with its current pose.
        """
        excluded_bodies = set(self.excluded_bodies)
        return [
            PlacedCollisionShape(
                body=body,
                shape=shape,
                world_T_shape=self.world.compute_forward_kinematics_np(
                    self.world.root, body
                )
                @ shape.origin.to_np(),
            )
            for body in self.world.bodies
            if body not in excluded_bodies
            for shape in body.collision
        ]

    def _shapes_stayed_in_place(
        self, placed_shapes: list[PlacedCollisionShape]
    ) -> bool:
        """
        :param placed_shapes: The shapes rays can hit, with their current poses.
        :return: Whether every shape is where :attr:`_raycasting_scene` holds it.
        """
        return len(placed_shapes) == len(self._placed_shapes) and all(
            np.array_equal(current.world_T_shape, built.world_T_shape)
            for current, built in zip(placed_shapes, self._placed_shapes)
        )

    def _build_raycasting_scene(
        self, placed_shapes: list[PlacedCollisionShape]
    ) -> None:
        """
        Replaces :attr:`_raycasting_scene` by one holding the given shapes.

        :param placed_shapes: The shapes rays can hit, with their current poses.
        """
        # open3d adds about a second to every import of the world, so only a process
        # that casts rays loads it.
        from open3d.t.geometry import RaycastingScene

        self._raycasting_scene = RaycastingScene()
        for placed_shape in placed_shapes:
            mesh = placed_shape.shape.mesh_in_frame(self.world.root)
            self._raycasting_scene.add_triangles(
                np.asarray(mesh.vertices, dtype=np.float32),
                np.asarray(mesh.faces, dtype=np.uint32),
            )
        self._placed_shapes = placed_shapes

    def create_segmentation_mask(
        self,
        camera_pose: GenericSpatialType,
        resolution: CameraResolution | None = None,
        min_distance: float = 0,
        max_distance: float = np.inf,
        field_of_view: FieldOfView | None = None,
    ) -> np.ndarray:
        """
        Creates a segmentation mask for the ray tracer scene from the camera position to
        the target position.

        Each pixel in the mask corresponds to the index of a body in the scene or -1 if
        no body is hit at that pixel.

        :param camera_pose: The position of the camera.
        :param resolution: The camera resolution, defaulting to
            :class:`CameraResolution`.
        :param min_distance: The minimum distance of a body to be considered a hit.
        :param max_distance: The maximum distance of a body to be considered a hit.
        :param field_of_view: The field of view of the camera, defaulting to
            FieldOfView()
        :return: A segmentation mask as a numpy array.
        """
        self.update_scene()
        ray_origins, ray_directions, pixels = self.create_camera_rays(
            camera_pose, resolution=resolution, field_of_view=field_of_view
        )

        target_points = ray_origins + ray_directions * 10

        points, index_ray, bodies = self.ray_test(
            ray_origins,
            target_points,
            multiple_hits=True,
            min_distance=min_distance,
            max_distance=max_distance,
        )
        unique_index = np.unique(index_ray, return_index=True)[1]

        index_ray = index_ray[unique_index]

        bodies = np.array([body.index for body in bodies])[unique_index]

        pixel_ray = pixels[index_ray]

        # create a numpy array we can turn into an image
        # doing it with uint8 creates an `L` mode greyscale image
        a = np.zeros(self.scene.camera.resolution, dtype=np.int32) - 1

        # assign bodies to correct pixel locations
        a[pixel_ray[:, 0], pixel_ray[:, 1]] = bodies

        return a

    def create_depth_map(
        self,
        camera_pose: GenericSpatialType,
        resolution: CameraResolution | None = None,
        min_distance: float = 0,
        max_distance: float = np.inf,
        field_of_view: FieldOfView | None = None,
    ) -> np.ndarray:
        """
        Creates a depth map for the ray tracer scene from the camera position to the
        target position.

        Each pixel in the depth map corresponds to the distance from the camera to the
        closest point on the surface of the scene or -1 if no point is hit.

        :param camera_pose: The position of the camera.
        :param resolution: The camera resolution, defaulting to
            :class:`CameraResolution`.
        :param min_distance: The minimum distance of a body to be considered a hit.
        :param max_distance: The maximum distance of a body to be considered a hit.
        :param field_of_view: The field of view of the camera, defaulting to
            FieldOfView()
        :return: A depth map as a numpy array.
        """
        self.update_scene()
        ray_origins, ray_directions, pixels = self.create_camera_rays(
            camera_pose, resolution=resolution, field_of_view=field_of_view
        )

        target_points = ray_origins + ray_directions * 10

        points, index_ray, bodies = self.ray_test(
            ray_origins,
            target_points,
            multiple_hits=True,
            min_distance=min_distance,
            max_distance=max_distance,
        )

        # Filters rays to the first hit only to create a depth map. This is done by finding the unique indices of the
        # rays that hit the scene and keeping only the first hit for each ray.
        unique_index = np.unique(index_ray, return_index=True)[1]
        if len(unique_index) != 0:
            index_ray = index_ray[unique_index]
            points = points[unique_index]
            ray_origins = ray_origins[unique_index]

        depth = trimesh.util.diagonal_dot(
            points - ray_origins[0], ray_directions[index_ray]
        )
        pixel_ray = pixels[index_ray]

        # create a numpy array we can turn into an image
        # doing it with uint8 creates an `L` mode greyscale image
        a = np.zeros(self.scene.camera.resolution, dtype=np.float32) - 1

        # assign depth to correct pixel locations
        a[pixel_ray[:, 0], pixel_ray[:, 1]] = depth

        return a

    def create_camera_rays(
        self,
        camera_pose: GenericSpatialType,
        resolution: CameraResolution | None = None,
        field_of_view: FieldOfView | None = None,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Creates camera rays for the ray tracer scene from the camera position to the
        target position.

        Places the camera at the given position and orientation view of the camera is
        along the x-axis.

        :param camera_pose: The position of the camera as a 4x4 transformation matrix.
        :param field_of_view: The field of view of the camera, defaulting to
            FieldOfView()
        :param resolution: The camera resolution, defaulting to
            :class:`CameraResolution`.
        :return: The origin points of the rays, the direction vectors of the rays, and
            the pixel coordinates.
        """
        field_of_view = field_of_view or FieldOfView()
        resolution = resolution or CameraResolution()
        camera_pose = camera_pose.to_np()
        self.update_scene()
        # By default, the camera is looking along the -z axis, so we need to rotate it to look along the x-axis.
        rotate = trimesh.transformations.rotation_matrix(
            angle=np.radians(-90.0), direction=[0, 1, 0]
        )
        rotate_x = trimesh.transformations.rotation_matrix(
            angle=np.radians(180.0), direction=[1, 0, 0]
        )

        self.scene.camera.fov = (
            np.degrees(field_of_view.horizontal_angle),
            np.degrees(field_of_view.vertical_angle),
        )
        self.scene.camera.resolution = (resolution.width, resolution.height)
        self.scene.graph[self.scene.camera.name] = camera_pose @ rotate_x @ rotate

        return self.scene.camera_rays()

    def ray_test(
        self,
        origin_points: npt.ArrayLike,
        target_points: npt.ArrayLike,
        multiple_hits: bool = False,
        min_distance: float = 0,
        max_distance: float = np.inf,
    ) -> tuple[npt.NDArray[np.float64], npt.NDArray[np.int_], list[Body]]:
        """
        Casts rays from the origin points through the target points.

        A ray does not end at its target, which only sets its direction.

        :param origin_points: The starting point of every ray.
        :param target_points: A point on every ray, one per origin.
        :param multiple_hits: Whether to report every surface a ray passes, or only the
            closest one.
        :param min_distance: The minimum distance of a reported hit from its origin.
        :param max_distance: The maximum distance of a reported hit from its origin.
        :return: The points where the rays hit a surface, the ray each point belongs to,
            and the body hit there, ordered by ray and then by distance from its origin.
        :raises RayEndpointCountMismatchError: If the number of origins and targets
            differ.
        """
        origin_points = np.asarray(origin_points, dtype=float).reshape((-1, 3))
        target_points = np.asarray(target_points, dtype=float).reshape((-1, 3))
        if origin_points.shape != target_points.shape:
            raise RayEndpointCountMismatchError(
                origin_count=len(origin_points), target_count=len(target_points)
            )
        self._update_raycasting_scene()

        ray_vectors = target_points - origin_points
        ray_directions = ray_vectors / np.linalg.norm(
            ray_vectors, axis=1, keepdims=True
        )
        rays = np.hstack((origin_points, ray_directions)).astype(np.float32)
        index_ray, distances, geometry_ids = self._intersect_rays(rays, multiple_hits)

        in_range = (distances >= min_distance) & (distances <= max_distance)
        index_ray = index_ray[in_range]
        distances = distances[in_range]
        geometry_ids = geometry_ids[in_range]
        order = np.lexsort((distances, index_ray))
        index_ray = index_ray[order]
        distances = distances[order]

        points = (
            origin_points[index_ray] + ray_directions[index_ray] * distances[:, None]
        )
        bodies = [
            self._placed_shapes[geometry_id].body for geometry_id in geometry_ids[order]
        ]
        return points, index_ray, bodies

    def _intersect_rays(
        self, rays: npt.NDArray[np.float32], multiple_hits: bool
    ) -> tuple[npt.NDArray[np.int_], npt.NDArray[np.float64], npt.NDArray[np.int_]]:
        """
        :param rays: One ray per row, its origin followed by its unit direction.
        :param multiple_hits: Whether to report every surface a ray passes, or only the
            closest one.
        :return: The ray, the distance from its origin and the open3d geometry of every
            hit.
        """
        if multiple_hits:
            intersections = self._raycasting_scene.list_intersections(rays)
            return (
                intersections["ray_ids"].numpy().astype(int),
                intersections["t_hit"].numpy().astype(float),
                intersections["geometry_ids"].numpy().astype(int),
            )
        closest_hits = self._raycasting_scene.cast_rays(rays)
        distances = closest_hits["t_hit"].numpy().astype(float)
        is_hit = np.isfinite(distances)
        return (
            np.flatnonzero(is_hit),
            distances[is_hit],
            closest_hits["geometry_ids"].numpy()[is_hit].astype(int),
        )
