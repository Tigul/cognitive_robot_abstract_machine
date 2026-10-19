from __future__ import annotations

from dataclasses import dataclass

import itertools

import numpy as np
import numpy.typing as npt
from scipy.sparse import coo_array
from scipy.sparse.csgraph import connected_components
from typing_extensions import Self

from semantic_digital_twin.spatial_types.spatial_types import Pose2D
from semantic_digital_twin.world_description.geometry import PlanarBoundingBox

# %% pose histogram


@dataclass
class PoseHistogram:
    """
    A grid of cells dividing the poses on a floor by position and heading.
    """

    cell_length: float = 0.5
    """
    The extent of a cell along x and y, in meters.
    """

    cell_angle: float = np.radians(10)
    """
    The extent of a cell along the yaw, in radians.
    """

    @property
    def heading_cell_count(self) -> int:
        """
        :return: How many cells one turn of the yaw spans.
        """
        return int(np.ceil(2 * np.pi / self.cell_angle))

    def find_cells(self, poses: npt.NDArray[np.float64]) -> npt.NDArray[np.int_]:
        """
        :param poses: One pose per row as x, y and yaw.
        :return: The integer x, y and yaw index of every pose's cell; the yaw index
            counts from -pi and wraps around after a full turn.
        """
        return np.column_stack(
            (
                np.floor(poses[:, :2] / self.cell_length),
                np.floor((poses[:, 2] + np.pi) / self.cell_angle)
                % self.heading_cell_count,
            )
        ).astype(int)

    def find_connected_groups(
        self, poses: npt.NDArray[np.float64]
    ) -> npt.NDArray[np.int_]:
        """
        :param poses: One pose per row as x, y and yaw.
        :return: For every pose, the number of the group of occupied cells it falls in;
            cells touching at a face, an edge or a corner belong to one group.
        """
        occupied_cells, cell_of_pose = np.unique(
            self.find_cells(poses), axis=0, return_inverse=True
        )
        # A margin of one cell around the occupied ones keeps the numbers of the
        # neighbours of the outermost cells apart.
        lowest_cell = occupied_cells.min(axis=0) - 1
        cell_extent = occupied_cells.max(axis=0) - lowest_cell + 2
        cell_extent[2] = self.heading_cell_count
        cell_numbers = self._number_cells(occupied_cells, lowest_cell, cell_extent)
        order = np.argsort(cell_numbers)

        cells, neighbours = [], []
        for offset in itertools.product((-1, 0, 1), repeat=3):
            neighbour_numbers = self._number_cells(
                occupied_cells + offset, lowest_cell, cell_extent
            )
            positions = np.minimum(
                np.searchsorted(cell_numbers, neighbour_numbers, sorter=order),
                len(order) - 1,
            )
            is_occupied = cell_numbers[order[positions]] == neighbour_numbers
            cells.append(np.flatnonzero(is_occupied))
            neighbours.append(order[positions[is_occupied]])
        cells = np.concatenate(cells)
        adjacency = coo_array(
            (np.ones(len(cells)), (cells, np.concatenate(neighbours))),
            shape=(len(occupied_cells),) * 2,
        )
        _, group_of_cell = connected_components(adjacency, directed=False)
        return group_of_cell[cell_of_pose.ravel()]

    def _number_cells(
        self,
        cells: npt.NDArray[np.int_],
        lowest_cell: npt.NDArray[np.int_],
        cell_extent: npt.NDArray[np.int_],
    ) -> npt.NDArray[np.int_]:
        """
        :param cells: The x, y and yaw index of one cell per row.
        :param lowest_cell: The cell numbered zero.
        :param cell_extent: How many cells the numbering spans along x, y and yaw.
        :return: A distinct number for every cell within the extent, wrapping the yaw
            index around a full turn.
        """
        shifted = cells - lowest_cell
        shifted[:, 2] = cells[:, 2] % self.heading_cell_count
        return np.ravel_multi_index(shifted.T, cell_extent)


# %% particle set


@dataclass
class ParticleSet:
    """
    Weighted hypotheses of where a robot base stands on the floor of a world.
    """

    poses: npt.NDArray[np.float64]
    """
    One pose per row as x, y and yaw, in the world root's frame; the yaw lies between
    -pi and pi.
    """

    weights: npt.NDArray[np.float64]
    """
    The weight of every pose, summing to one.
    """

    @classmethod
    def uniform(
        cls,
        region: PlanarBoundingBox,
        count: int,
        random_generator: np.random.Generator,
    ) -> Self:
        """
        :param region: The part of the world root's x-y plane the particles cover.
        :param count: How many particles to draw.
        :param random_generator: The source of the particles' randomness.
        :return: Equally weighted particles at uniformly drawn positions and headings in
            the region.
        """
        poses = random_generator.uniform(
            low=[region.x_interval.lower, region.y_interval.lower, -np.pi],
            high=[region.x_interval.upper, region.y_interval.upper, np.pi],
            size=(count, 3),
        )
        return cls(poses=poses, weights=np.full(count, 1 / count))

    @classmethod
    def around(
        cls,
        pose: Pose2D,
        covariance: npt.NDArray[np.float64],
        count: int,
        random_generator: np.random.Generator,
    ) -> Self:
        """
        :param pose: The pose the particles are centered on.
        :param covariance: The covariance of x, y and yaw the particles are drawn with.
        :param count: How many particles to draw.
        :param random_generator: The source of the particles' randomness.
        :return: Equally weighted particles normally distributed around the pose.
        """
        particles = cls(
            poses=random_generator.multivariate_normal(pose.to_np(), covariance, count),
            weights=np.full(count, 1 / count),
        )
        particles._wrap_headings()
        return particles

    def __len__(self) -> int:
        return len(self.weights)

    @property
    def effective_sample_size(self) -> float:
        """
        :return: How many equally weighted particles would carry as much information as
            these.
        """
        return float(1 / np.sum(self.weights**2))

    def mean(self) -> Pose2D:
        """
        :return: The weighted mean pose of the particles, averaging headings on the
            circle.
        """
        x, y = self.weights @ self.poses[:, :2]
        yaw = np.arctan2(
            self.weights @ np.sin(self.poses[:, 2]),
            self.weights @ np.cos(self.poses[:, 2]),
        )
        return Pose2D(x=x, y=y, yaw=yaw)

    def covariance(self) -> npt.NDArray[np.float64]:
        """
        :return: The weighted covariance of x, y and yaw around :meth:`mean`, measuring
            headings by the shorter way around the circle.
        """
        deviations = self.poses - self.mean().to_np()
        deviations[:, 2] = np.arctan2(
            np.sin(deviations[:, 2]), np.cos(deviations[:, 2])
        )
        return (self.weights[:, None] * deviations).T @ deviations

    def heaviest_cluster(self, histogram: PoseHistogram) -> ParticleSet:
        """
        :param histogram: The cells particles are grouped by.
        :return: The particles of the connected group of occupied cells holding the most
            weight, with their weights rescaled to sum to one.
        """
        groups = histogram.find_connected_groups(self.poses)
        in_cluster = groups == np.bincount(groups, weights=self.weights).argmax()
        return ParticleSet(
            poses=self.poses[in_cluster],
            weights=self.weights[in_cluster] / self.weights[in_cluster].sum(),
        )

    def move(self, displacements: npt.NDArray[np.float64]) -> None:
        """
        Moves every particle by a displacement given in its own frame.

        :param displacements: One displacement per particle as x, y and yaw, in the
            frame of that particle.
        """
        yaws = self.poses[:, 2]
        cosines, sines = np.cos(yaws), np.sin(yaws)
        self.poses[:, 0] += cosines * displacements[:, 0] - sines * displacements[:, 1]
        self.poses[:, 1] += sines * displacements[:, 0] + cosines * displacements[:, 1]
        self.poses[:, 2] += displacements[:, 2]
        self._wrap_headings()

    def _wrap_headings(self) -> None:
        """
        Brings every particle's yaw between -pi and pi without turning it.
        """
        yaws = self.poses[:, 2]
        self.poses[:, 2] = np.arctan2(np.sin(yaws), np.cos(yaws))

    def to_transformation_matrices(self) -> npt.NDArray[np.float64]:
        """
        :return: The homogeneous transformation of every particle in the world root's
            frame, stacked along the first axis.
        """
        cosines, sines = np.cos(self.poses[:, 2]), np.sin(self.poses[:, 2])
        matrices = np.tile(np.eye(4), (len(self), 1, 1))
        matrices[:, 0, 0] = cosines
        matrices[:, 0, 1] = -sines
        matrices[:, 1, 0] = sines
        matrices[:, 1, 1] = cosines
        matrices[:, :2, 3] = self.poses[:, :2]
        return matrices
