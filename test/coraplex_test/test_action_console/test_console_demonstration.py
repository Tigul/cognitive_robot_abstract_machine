"""
Coverage for the demonstration performing what the action console asks for.
"""

from __future__ import annotations

from pathlib import Path

from coraplex.demonstrations import RobotDemonstrationRosSession
from coraplex.plans.executors import (
    KinematicallySimulatedPlanExecutor,
    RealRobotPlanExecutor,
)
from coraplex.robot_plans.actions.core.container import (
    MovesContainerByItsHandle,
    OpenAction,
)
from coraplex.standalone_process import StandaloneControllerProcess
from cramph.composites import Sequence, StatementNode
from semantic_digital_twin.robots.minimal_robot import MinimalRobot
from semantic_digital_twin.robots.pr2 import PR2
from semantic_digital_twin.semantic_annotations.semantic_annotations import Handle

# %% run modes


def test_only_a_simulated_run_executes_in_this_process(action_console):
    executor_types = {
        run_mode: run_mode.executor_type for run_mode in action_console.RunMode
    }

    assert executor_types == {
        action_console.RunMode.SIMULATED: KinematicallySimulatedPlanExecutor,
        action_console.RunMode.GISKARD_STARTED_BY_CONSOLE: RealRobotPlanExecutor,
        action_console.RunMode.GISKARD_RUNNING: RealRobotPlanExecutor,
    }


def test_only_the_console_mode_starting_giskard_has_a_controller(action_console):
    launchers = {
        run_mode: run_mode.controller_launcher(PR2)
        for run_mode in action_console.RunMode
    }

    assert launchers == {
        action_console.RunMode.SIMULATED: None,
        action_console.RunMode.GISKARD_STARTED_BY_CONSOLE: (
            action_console.GiskardStandaloneModule.PR2.launcher_path
        ),
        action_console.RunMode.GISKARD_RUNNING: None,
    }


def test_the_standalone_controller_of_a_robot_is_installed(action_console):
    assert action_console.GiskardStandaloneModule.for_robot(PR2).launcher_path.is_file()


# %% plans


def test_a_statement_of_an_action_is_grounded_when_the_plan_runs(
    action_console,
    pr2_console_demonstration,
    pr2_apartment_context,
    pr2_statechart_context,
):
    world, pr2, _ = pr2_apartment_context
    [factory] = [
        builder
        for builder in action_console.ActionCatalog.discover().builders_of(OpenAction)
        if isinstance(builder, action_console.ActionFactory)
        and builder.method.__func__ is MovesContainerByItsHandle.from_handle.__func__
    ]
    pr2_console_demonstration.action_request = action_console.ActionRequest(
        factory,
        {
            "handle": world.get_semantic_annotations_by_type(Handle)[0],
            "arm": pr2.all_arms[0],
        },
    )

    statechart = pr2_console_demonstration.build_statechart(pr2_statechart_context)

    [sequence] = [node for node in statechart.nodes if isinstance(node, Sequence)]
    [step] = sequence.nodes
    assert type(step) is StatementNode


# %% controllers


def test_a_started_controller_serves_the_world_and_is_stopped_at_tear_down(
    action_console, cylinder_bot_world, monkeypatch
):
    events = []
    monkeypatch.setattr(
        StandaloneControllerProcess,
        "start",
        lambda controller: events.append(("started", controller.launcher_path)),
    )
    monkeypatch.setattr(
        StandaloneControllerProcess,
        "stop",
        lambda controller: events.append(("stopped", controller.launcher_path)),
    )

    def serve_world(session, timeout_seconds=None):
        events.append(("served", None))
        return cylinder_bot_world

    monkeypatch.setattr(RobotDemonstrationRosSession, "fetch_world", serve_world)
    launcher = Path("controller.py")
    demonstration = action_console.ConsoleDemonstration(
        used_robot=MinimalRobot,
        executor_type=RealRobotPlanExecutor,
        controller_launcher=launcher,
    )

    try:
        demonstration.acquire_world()
    finally:
        demonstration.tear_down()

    assert events == [("started", launcher), ("served", None), ("stopped", launcher)]
