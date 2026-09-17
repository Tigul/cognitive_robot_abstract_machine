from __future__ import annotations

from dataclasses import dataclass

import pytest

from cramph.context import ContextExtension, StatechartContext
from cramph.exceptions import ConflictingTickDurationError

# %% context extensions


@dataclass
class ExtensionRecordingItsCleanup(ContextExtension):
    """
    A context extension that remembers whether it was cleaned up.
    """

    cleaned_up: bool = False
    """
    Whether :meth:`cleanup` was called.
    """

    def cleanup(self):
        self.cleaned_up = True


def test_cleaning_up_a_context_cleans_up_its_extensions(
    statechart_context: StatechartContext,
):
    extension = ExtensionRecordingItsCleanup()
    statechart_context.add_extension(extension)

    statechart_context.cleanup()

    assert extension.cleaned_up


# %% tick duration


def test_a_context_without_a_tick_duration_takes_the_one_set(
    statechart_context_without_tick_duration: StatechartContext,
):
    statechart_context_without_tick_duration.set_tick_duration(0.02)

    assert statechart_context_without_tick_duration.require_tick_duration() == 0.02


def test_setting_the_tick_duration_a_context_already_has_keeps_it(
    statechart_context: StatechartContext,
):
    tick_duration = statechart_context.require_tick_duration()

    statechart_context.set_tick_duration(tick_duration)

    assert statechart_context.require_tick_duration() == tick_duration


def test_setting_a_different_tick_duration_is_rejected(
    statechart_context: StatechartContext,
):
    tick_duration = statechart_context.require_tick_duration()

    with pytest.raises(ConflictingTickDurationError):
        statechart_context.set_tick_duration(tick_duration * 2)

    assert statechart_context.require_tick_duration() == tick_duration
