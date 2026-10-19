from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass

import numpy as np
import numpy.typing as npt
from typing_extensions import Generic, TypeVar

from krrood.patterns.subclass_safe_generic import SubClassSafeGeneric
from krrood.utils import recursive_subclasses
from semantic_digital_twin.localization.exceptions import NoMotionModelForDrive
from semantic_digital_twin.spatial_types.spatial_types import Pose2D
from semantic_digital_twin.world_description.connections import (
    DifferentialDrive,
    OmniDrive,
    WheeledDrive,
)

# %% motion models

DriveType = TypeVar("DriveType", bound=WheeledDrive)


@dataclass
class MotionModel(Generic[DriveType], SubClassSafeGeneric, ABC):
    """
    How uncertain the motion of a robot base is that its odometry measured.

    Each model binds the kind of drive whose motion it describes, as in
    ``MotionModel[DifferentialDrive]``.

    The noise parameters weigh the squared measured translation, in square meters, and
    the squared measured rotation, in square radians, into the variance of the sampled
    motion.
    """

    rotation_noise_from_rotation: float = 0.2
    """
    How much the measured rotation adds to the rotation's variance (AMCL's alpha1).
    """

    rotation_noise_from_translation: float = 0.2
    """
    How much the measured translation adds to the rotation's variance (AMCL's alpha2).
    """

    translation_noise_from_translation: float = 0.2
    """
    How much the measured translation adds to the translation's variance (AMCL's
    alpha3).
    """

    translation_noise_from_rotation: float = 0.2
    """
    How much the measured rotation adds to the translation's variance (AMCL's alpha4).
    """

    @classmethod
    def for_drive_type(cls, drive_type: type[WheeledDrive]) -> MotionModel:
        """
        :param drive_type: The kind of drive whose motion the odometry measures.
        :return: The motion model of that kind of drive, with default noise.
        :raises NoMotionModelForDrive: If no motion model describes that kind of drive.
        """
        for model_type in recursive_subclasses(MotionModel):
            if issubclass(
                drive_type, model_type.get_type_of_generic_parameter(DriveType)
            ):
                return model_type()
        raise NoMotionModelForDrive(drive_type=drive_type)

    def _rotation_deviation(self, rotation: float, translation: float) -> float:
        """
        :param rotation: A measured rotation, in radians.
        :param translation: The measured translation along with it, in meters.
        :return: The standard deviation of the true rotation, in radians.
        """
        return np.sqrt(
            self.rotation_noise_from_rotation * rotation**2
            + self.rotation_noise_from_translation * translation**2
        )

    @abstractmethod
    def sample_displacements(
        self,
        motion: Pose2D,
        count: int,
        random_generator: np.random.Generator,
    ) -> npt.NDArray[np.float64]:
        """
        :param motion: The pose of the base at the current odometry reading, in the
            base's frame at the previous one.
        :param count: How many displacements to draw.
        :param random_generator: The source of the noise.
        :return: One displacement the base may really have moved by per row, as x, y
            and yaw in the base's frame at the previous reading.
        """


@dataclass
class DifferentialDriveMotionModel(MotionModel[DifferentialDrive]):
    """
    The motion of a base that turns, drives straight along its heading and turns again.
    """

    def sample_displacements(
        self,
        motion: Pose2D,
        count: int,
        random_generator: np.random.Generator,
    ) -> npt.NDArray[np.float64]:
        x, y, yaw = motion.to_np()
        translation = np.hypot(x, y)
        first_rotation = np.arctan2(y, x) if translation > 0 else 0.0
        if abs(first_rotation) > np.pi / 2:
            # A base driving backwards turns by the angle to its rear, not its front.
            translation = -translation
            first_rotation = np.arctan2(
                np.sin(first_rotation - np.pi), np.cos(first_rotation - np.pi)
            )
        second_rotation = yaw - first_rotation

        noisy_first_rotation = first_rotation + random_generator.normal(
            0, self._rotation_deviation(first_rotation, translation), count
        )
        noisy_translation = translation + random_generator.normal(
            0,
            np.sqrt(
                self.translation_noise_from_translation * translation**2
                + self.translation_noise_from_rotation
                * (first_rotation**2 + second_rotation**2)
            ),
            count,
        )
        noisy_second_rotation = second_rotation + random_generator.normal(
            0, self._rotation_deviation(second_rotation, translation), count
        )
        return np.column_stack(
            (
                noisy_translation * np.cos(noisy_first_rotation),
                noisy_translation * np.sin(noisy_first_rotation),
                noisy_first_rotation + noisy_second_rotation,
            )
        )


@dataclass
class OmniDriveMotionModel(MotionModel[OmniDrive]):
    """
    The motion of a base that drives in any direction while turning.
    """

    strafe_noise_from_translation: float = 0.2
    """
    How much the measured translation adds to the variance of the translation sideways
    to the direction of travel (AMCL's alpha5).
    """

    def sample_displacements(
        self,
        motion: Pose2D,
        count: int,
        random_generator: np.random.Generator,
    ) -> npt.NDArray[np.float64]:
        x, y, yaw = motion.to_np()
        translation = np.hypot(x, y)
        bearing = np.arctan2(y, x)

        noisy_translation = translation + random_generator.normal(
            0,
            np.sqrt(
                self.translation_noise_from_translation * translation**2
                + self.translation_noise_from_rotation * yaw**2
            ),
            count,
        )
        strafe = random_generator.normal(
            0,
            np.sqrt(
                self.strafe_noise_from_translation * translation**2
                + self.translation_noise_from_rotation * yaw**2
            ),
            count,
        )
        noisy_rotation = yaw + random_generator.normal(
            0, self._rotation_deviation(yaw, translation), count
        )
        return np.column_stack(
            (
                noisy_translation * np.cos(bearing) - strafe * np.sin(bearing),
                noisy_translation * np.sin(bearing) + strafe * np.cos(bearing),
                noisy_rotation,
            )
        )
