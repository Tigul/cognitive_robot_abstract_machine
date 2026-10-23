"""
Coverage for launching a component in a process of its own and stopping it again.
"""

from __future__ import annotations

from pathlib import Path

import pytest

from coraplex.exceptions import ProcessDiedDuringStartup, ProcessNeverBecameReady
from coraplex.standalone_process import StandaloneProcess

# %% launchers


@pytest.fixture
def dataset_directory() -> Path:
    """
    The directory the launchers of these tests are read from.
    """
    return Path(__file__).parent.parent / "dataset"


# %% start-up


def test_a_process_dying_during_startup_is_reported(dataset_directory):
    launcher = dataset_directory / "launcher_exiting_at_once.py"
    process = StandaloneProcess(launcher_path=launcher, is_ready=lambda: False)

    with pytest.raises(ProcessDiedDuringStartup) as error:
        process.start()

    assert error.value.launcher_path == launcher


def test_a_process_that_never_becomes_ready_is_reported(dataset_directory):
    launcher = dataset_directory / "launcher_running_until_interrupted.py"
    process = StandaloneProcess(
        launcher_path=launcher, is_ready=lambda: False, ready_timeout=1.0
    )

    try:
        with pytest.raises(ProcessNeverBecameReady) as error:
            process.start()
    finally:
        process.stop()

    assert error.value.launcher_path == launcher


# %% shutdown


def test_a_ready_process_runs_until_it_is_stopped(dataset_directory):
    process = StandaloneProcess(
        launcher_path=dataset_directory / "launcher_running_until_interrupted.py",
        is_ready=lambda: True,
    )

    with process:
        assert process.process.poll() is None

    assert process.process.poll() is not None
