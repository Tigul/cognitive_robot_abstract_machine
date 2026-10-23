"""
Components launched in a process of their own, such as a controller or a perception
pipeline, which share nothing with the launching interpreter but the middleware.
"""

from __future__ import annotations

import os
import signal
import subprocess
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

from typing_extensions import Callable

from coraplex.demonstrations import RobotDemonstrationRosSession
from coraplex.exceptions import ProcessDiedDuringStartup, ProcessNeverBecameReady
from krrood.ormatic.utils import classproperty

# %% launched processes


@dataclass
class StandaloneProcess:
    """
    A component launched as its own process and stopped again once it is not needed.
    """

    launcher_path: Path
    """
    Script to run.
    """

    is_ready: Callable[[], bool]
    """
    Whether the launched component is up yet.

    Supplied by the caller because only it knows what the component announces and which
    node to look from.
    """

    arguments: list[str] = field(default_factory=list)
    """
    Command-line arguments passed to the script.
    """

    ready_timeout: float = 180.0
    """
    How long to wait for :attr:`is_ready` to hold, in seconds.
    """

    readiness_poll_interval: float = 0.5
    """
    How long to wait between two checks of :attr:`is_ready`, in seconds.
    """

    shutdown_timeout: float = 60.0
    """
    How long to let the process shut down after SIGINT before killing it, in seconds.
    """

    process: subprocess.Popen | None = field(init=False, default=None)
    """
    The launched process, once started.
    """

    def start(self) -> None:
        """
        Launch the process and block until it is ready.
        """
        # Output is discarded rather than piped: an undrained pipe blocks the child once
        # it fills. start_new_session gives the process its own group, so it can be
        # signalled without hitting the launching process.
        self.process = subprocess.Popen(
            [sys.executable, str(self.launcher_path), *self.arguments],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
            env={**os.environ, "PYTHONPATH": os.pathsep.join(sys.path)},
        )
        self.wait_until_ready()

    def wait_until_ready(self) -> None:
        """
        Block until the launched component announces itself.

        :raises ProcessDiedDuringStartup: If the process ends before it is ready.
        :raises ProcessNeverBecameReady: If it is not ready within
            :attr:`ready_timeout`.
        """
        deadline = time.monotonic() + self.ready_timeout
        while time.monotonic() < deadline:
            if self.process.poll() is not None:
                raise ProcessDiedDuringStartup(
                    launcher_path=self.launcher_path,
                    return_code=self.process.returncode,
                )
            if self.is_ready():
                return
            time.sleep(self.readiness_poll_interval)
        raise ProcessNeverBecameReady(
            launcher_path=self.launcher_path, ready_timeout=self.ready_timeout
        )

    def stop(self) -> None:
        """
        Interrupt the process, killing it if it outstays its shutdown budget.
        """
        if self.process.poll() is not None:
            return
        os.killpg(os.getpgid(self.process.pid), signal.SIGINT)
        try:
            self.process.wait(timeout=self.shutdown_timeout)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait()

    def __enter__(self) -> StandaloneProcess:
        self.start()
        return self

    def __exit__(self, exception_type, exception, traceback) -> None:
        self.stop()


# %% controllers


@dataclass
class StandaloneControllerProcess:
    """
    A standalone Giskard controller running in its own process, as on the robot.

    Nothing is shared with the launching interpreter, so every exchange with the
    controller has to survive the middleware.
    """

    launcher_path: Path
    """
    Standalone controller script to run.
    """

    session: RobotDemonstrationRosSession | None = field(init=False, default=None)
    """
    Session watching the controller's services.

    It owns the ROS context while the controller runs, so a demonstration running
    against this controller finds a context it did not create and leaves it alone.
    """

    process: StandaloneProcess | None = field(init=False, default=None)
    """
    The controller's process, once started.
    """

    @classproperty
    def world_fetch_service_suffix(cls) -> str:
        """
        :return: Suffix of the service a controller announces once it is serving its
            world.
        """
        return "fetch_world"

    def start(self) -> None:
        """
        Launch the controller and block until it serves its world.
        """
        self.session = RobotDemonstrationRosSession.start("controller_process_probe")
        self.process = StandaloneProcess(
            launcher_path=self.launcher_path, is_ready=self.is_serving_world
        )
        self.process.start()

    def is_serving_world(self) -> bool:
        """
        Whether the controller advertises its world-fetch service yet.
        """
        return any(
            name.endswith(self.world_fetch_service_suffix)
            for name, _ in self.session.node.get_service_names_and_types()
        )

    def stop(self) -> None:
        """
        Stop the controller and release the ROS context.
        """
        self.process.stop()
        self.session.stop()

    def __enter__(self) -> StandaloneControllerProcess:
        self.start()
        return self

    def __exit__(self, exception_type, exception, traceback) -> None:
        self.stop()
