"""
Fixtures for running demonstrations against a controller in its own process.
"""

from __future__ import annotations

from importlib.resources import files
from pathlib import Path

import pytest
from rclpy.action import get_action_names_and_types

from coraplex.perception import ROBOKUDO_QUERY_ACTION_NAME
from experiments.real_stretch_apartment_demo.demo import CEREAL_NAME
from semantic_digital_twin.semantic_annotations.semantic_annotations import CheezeIt

from coraplex.standalone_process import (
    StandaloneControllerProcess,
    StandaloneProcess,
)

# %% standalone controller process


@pytest.fixture()
def stretch_controller_process():
    """
    A standalone Giskard controller for the Stretch, running in its own process.
    """
    launcher = files("giskardpy").joinpath(
        "middleware/ros2/scripts/iai_robots/stretch/stretch_standalone.py"
    )
    with StandaloneControllerProcess(launcher_path=Path(str(launcher))) as controller:
        yield controller


# %% standalone perception pipeline


@pytest.fixture()
def cereal_perception_process(stretch_controller_process):
    """
    A perception pipeline reporting the demonstration's cereal, in its own process.

    The demonstration detects before it grasps, so a real run needs a pipeline answering
    queries. The cereal is reported at the origin of its own frame, so the pipeline sees
    it wherever it currently stands: a detection taken after the demonstration has
    carried it somewhere else confirms that pose instead of pulling it back to the one it
    was spawned at.
    """
    pytest.importorskip("robokudo_msgs")
    probe_node = stretch_controller_process.session.node

    def is_serving_queries() -> bool:
        return any(
            name.lstrip("/") == ROBOKUDO_QUERY_ACTION_NAME
            for name, _ in get_action_names_and_types(probe_node)
        )

    cereal_origin = (0.0, 0.0, 0.0)

    with StandaloneProcess(
        launcher_path=Path(__file__).parent.parent.parent
        / "dataset"
        / "perception_pipeline_stand_in.py",
        is_ready=is_serving_queries,
        arguments=[
            "--class-label",
            CheezeIt.__name__,
            "--frame-id",
            CEREAL_NAME,
            "--position",
            *(str(coordinate) for coordinate in cereal_origin),
        ],
    ) as process:
        yield process
