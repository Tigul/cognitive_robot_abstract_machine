from abc import ABC
from dataclasses import dataclass, fields

import numpy as np
import pytest

from semantic_digital_twin.localization.exceptions import NoMotionModelForDrive
from semantic_digital_twin.localization.motion_models import (
    DifferentialDriveMotionModel,
    MotionModel,
    OmniDriveMotionModel,
)
from semantic_digital_twin.spatial_types.spatial_types import Pose2D
from semantic_digital_twin.world_description.connections import (
    DifferentialDrive,
    OmniDrive,
    WheeledDrive,
)

# %% helpers


def noise_free(model_type: type[MotionModel]) -> MotionModel:
    """
    :return: A motion model of the given type that adds no noise.
    """
    return model_type(**{noise.name: 0.0 for noise in fields(model_type)})


@dataclass(eq=False)
class UnmodelledDrive(WheeledDrive, ABC):
    """
    A kind of drive no motion model describes.
    """


# %% choosing the model of a drive


@pytest.mark.parametrize(
    "drive_type, model_type",
    [
        (DifferentialDrive, DifferentialDriveMotionModel),
        (OmniDrive, OmniDriveMotionModel),
    ],
)
def test_a_drive_is_given_the_motion_model_of_its_type(drive_type, model_type):
    model = MotionModel.for_drive_type(drive_type)

    assert type(model) is model_type


def test_a_drive_no_motion_model_describes_is_rejected():
    with pytest.raises(NoMotionModelForDrive):
        MotionModel.for_drive_type(UnmodelledDrive)


# %% without noise


@pytest.mark.parametrize(
    "model_type", [DifferentialDriveMotionModel, OmniDriveMotionModel]
)
def test_without_noise_every_particle_moves_by_the_measured_motion(model_type):
    motion = Pose2D(x=0.3, y=0.1, yaw=0.2)

    displacements = noise_free(model_type).sample_displacements(
        motion, 5, np.random.default_rng(0)
    )

    np.testing.assert_allclose(displacements, np.tile(motion.to_np(), (5, 1)))


def test_a_differential_drive_driving_backwards_moves_backwards():
    motion = Pose2D(x=-0.3, y=0.0, yaw=0.0)

    displacements = noise_free(DifferentialDriveMotionModel).sample_displacements(
        motion, 1, np.random.default_rng(0)
    )

    np.testing.assert_allclose(displacements, [motion.to_np()], atol=1e-12)


# %% with noise


def test_differential_drive_translation_noise_grows_with_the_distance_driven():
    model = DifferentialDriveMotionModel(
        rotation_noise_from_rotation=0.0,
        rotation_noise_from_translation=0.0,
        translation_noise_from_translation=0.04,
        translation_noise_from_rotation=0.0,
    )
    distance = 2.0

    displacements = model.sample_displacements(
        Pose2D(x=distance), 20000, np.random.default_rng(0)
    )

    assert displacements[:, 0].std() == pytest.approx(
        np.sqrt(model.translation_noise_from_translation) * distance, rel=0.05
    )


def test_omni_drive_strafe_noise_moves_particles_sideways():
    model = OmniDriveMotionModel(
        rotation_noise_from_rotation=0.0,
        rotation_noise_from_translation=0.0,
        translation_noise_from_translation=0.0,
        translation_noise_from_rotation=0.0,
        strafe_noise_from_translation=0.04,
    )
    distance = 2.0

    displacements = model.sample_displacements(
        Pose2D(x=distance), 20000, np.random.default_rng(0)
    )

    assert displacements[:, 1].std() == pytest.approx(
        np.sqrt(model.strafe_noise_from_translation) * distance, rel=0.05
    )
