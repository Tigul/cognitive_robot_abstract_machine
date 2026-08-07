import pytest

import giskardpy.middleware.ros2.python_interface as giskard_python_interface_module
from coraplex.datastructures.dataclasses import Context
from krrood.utils import clear_memoization_cache

# %% test doubles


class FakeGiskardWrapper:
    """Stand-in for ``GiskardWrapper`` that avoids constructing a real ROS action client."""

    construction_count: int = 0

    def __init__(self, ros_node, world):
        FakeGiskardWrapper.construction_count += 1
        self.ros_node = ros_node
        self.world = world


@pytest.fixture(autouse=True)
def fake_giskard_wrapper(monkeypatch):
    FakeGiskardWrapper.construction_count = 0
    monkeypatch.setattr(
        giskard_python_interface_module, "GiskardWrapper", FakeGiskardWrapper
    )


@pytest.fixture
def context(tracy_world) -> Context:
    return Context.from_world(tracy_world)


# %% Context.giskard_wrapper


def test_giskard_wrapper_is_cached_across_repeated_access(context):
    first = context.giskard_wrapper
    second = context.giskard_wrapper

    assert first is second
    assert FakeGiskardWrapper.construction_count == 1


def test_giskard_wrapper_is_rebuilt_after_memoization_cache_is_cleared(context):
    first = context.giskard_wrapper

    clear_memoization_cache(context)

    second = context.giskard_wrapper

    assert second is not first
    assert FakeGiskardWrapper.construction_count == 2
