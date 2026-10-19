import numpy as np

from semantic_digital_twin.robots.garmi import Garmi

# %% the merged scan of both scanners


def test_garmi_sweeps_its_merged_scan_from_where_its_base_link_is(_garmi_world_setup):
    garmi = _garmi_world_setup.get_semantic_annotations_by_type(Garmi)[0]

    np.testing.assert_allclose(
        _garmi_world_setup.compute_forward_kinematics_np(
            garmi.root, garmi.mobile_base.lidar.root
        ),
        np.eye(4),
        atol=1e-12,
    )
