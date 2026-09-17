import pytest

from giskardpy.motion_statechart.context import MotionStatechartContext
from semantic_digital_twin.world import World

# %% tick duration


def test_motion_context_ticks_at_the_control_rate():
    context = MotionStatechartContext(world=World())

    assert context.tick_duration == context.qp_controller_config.control_dt


def test_motion_context_takes_its_tick_duration_only_from_the_controller():
    with pytest.raises(TypeError):
        MotionStatechartContext(world=World(), tick_duration=1.0)


def test_empty_motion_context_does_not_know_its_tick_duration():
    context = MotionStatechartContext.empty()

    assert context.tick_duration is None
