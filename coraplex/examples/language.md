---
jupyter:
  jupytext:
    text_representation:
      extension: .md
      format_name: markdown
      format_version: '1.3'
      jupytext_version: 1.16.3
  kernelspec:
    display_name: Python 3 (ipykernel)
    language: python
    name: python3
---

# Plan Language

The CoraPlex plan language structures what a plan does. It is the set of cramph composites: statechart nodes that run
their children in a given order and decide, from how the children ended, whether they succeeded themselves. A plan is a
tree of these composites with actions at its leaves, built on its own and then compiled and executed by a
{class}`~coraplex.plans.plan_execution.PlanExecutor`.

| Name                | Description                                                                                                         |
|---------------------|---------------------------------------------------------------------------------------------------------------------|
| **Sequence**        | Runs its children one after another and fails as soon as one of them fails.                                         |
| **TryInOrder**      | Runs its children one after another until one succeeds, and fails only if all of them failed.                      |
| **Parallel**        | Holds all children at once and succeeds once enough of them, all by default, are at their goals together.          |
| **TryAll**          | Runs all children at once and succeeds once one of them succeeded.                                                  |
| **RepeatUntil**     | Attempts a child again whenever an attempt fails, until it succeeds or a monitor calls the repeating off.           |
| **Monitored nodes** | `CancelledWhenTrue`, `PausedWhileTrue` and `PausedUntilTrue` cancel or pause a child depending on a monitor.        |

# Setup the World

If you are performing a plan with a simulated robot, you need a world and a context to execute the plan in.

```python
from coraplex.execution_environment import simulated_robot
from coraplex.plans.plan_execution import PlanExecutor
from coraplex.testing import setup_world
from coraplex.datastructures.dataclasses import Context
from semantic_digital_twin.robots.pr2 import PR2

world = setup_world()
pr2 = PR2.from_world(world)

context = Context(world, pr2)
executor = PlanExecutor(context)
```

## Sequence

A sequence runs its children one after another. If one of them fails, the sequence fails and the children after it
never start.

We will start with a simple example that moves the robot and parks its arms.

```python
from coraplex.datastructures.enums import Arms
from coraplex.robot_plans.actions.core.navigation import NavigateAction
from coraplex.robot_plans.actions.core.robot_body import ParkArmsAction
from cramph.composites import Sequence
from semantic_digital_twin.spatial_types import Pose

navigate = NavigateAction(Pose.from_xyz_rpy(1, 1, 0, reference_frame=world.root))
park = ParkArmsAction(Arms.BOTH)

plan = Sequence([navigate, park])
```

The plan is executed by compiling and executing it inside a ```with simulated_robot``` environment.

```python
with simulated_robot:
    executor.compile(plan)
    executor.execute()
```

Afterwards the statechart the plan ran in can be inspected in an interactive visualization.

```python
plan.statechart.visualize()
```

## Try In Order

Try in order also runs its children one after another, but a failing child does not end it: the next child is tried
instead. It fails only if all of its children failed.

```python
from coraplex.datastructures.enums import Arms
from coraplex.robot_plans.actions.core.navigation import NavigateAction
from coraplex.robot_plans.actions.core.robot_body import ParkArmsAction
from cramph.composites import TryInOrder
from semantic_digital_twin.spatial_types import Pose

navigate = NavigateAction(Pose.from_xyz_rpy(1, 1, 0, reference_frame=world.root))
park = ParkArmsAction(Arms.BOTH)

plan = TryInOrder([navigate, park])

with simulated_robot:
    executor.compile(plan)
    executor.execute()
```

## Parallel

Parallel holds all of its children at once, in the same statechart, and succeeds once enough of them, all of them
by default, are at their goals on the same tick.

```python
from coraplex.datastructures.enums import Arms
from coraplex.robot_plans.actions.core.navigation import NavigateAction
from coraplex.robot_plans.actions.core.robot_body import ParkArmsAction
from cramph.composites import Parallel
from semantic_digital_twin.spatial_types import Pose

navigate = NavigateAction(Pose.from_xyz_rpy(1, 1, 0, reference_frame=world.root))
park = ParkArmsAction(Arms.BOTH)

plan = Parallel([navigate, park])

with simulated_robot:
    executor.compile(plan)
    executor.execute()
```

## Try All

TryAll is to Parallel what TryInOrder is to Sequence: it runs all of its children at once and succeeds once one of
them succeeded.

```python
from coraplex.datastructures.enums import Arms
from coraplex.robot_plans.actions.core.navigation import NavigateAction
from coraplex.robot_plans.actions.core.robot_body import ParkArmsAction
from cramph.composites import TryAll
from semantic_digital_twin.spatial_types import Pose

navigate = NavigateAction(Pose.from_xyz_rpy(1, 1, 0, reference_frame=world.root))
park = ParkArmsAction(Arms.BOTH)

plan = TryAll([navigate, park])

with simulated_robot:
    executor.compile(plan)
    executor.execute()
```

## Combination of Expressions

Composites are statechart nodes themselves, so they nest. For example, a Sequence can run as one child of a Parallel.

```python
from coraplex.datastructures.enums import Arms
from coraplex.robot_plans.actions.core.navigation import NavigateAction
from coraplex.robot_plans.actions.core.robot_body import MoveTorsoAction, ParkArmsAction
from cramph.composites import Parallel, Sequence
from semantic_digital_twin.datastructures.definitions import TorsoState
from semantic_digital_twin.spatial_types import Pose

navigate = NavigateAction(Pose.from_xyz_rpy(1, 1, 0, reference_frame=world.root))
park = ParkArmsAction(Arms.BOTH)
move_torso = MoveTorsoAction(TorsoState.HIGH)

plan = Parallel([navigate, Sequence([park, move_torso])])

with simulated_robot:
    executor.compile(plan)
    executor.execute()
```

In this case 'park' and 'move_torso' form a Sequence, and that Sequence runs in parallel with 'navigate'.

## Code Objects

A plan can also call Python code. A {class}`~coraplex.plans.function_call.FunctionCall` calls its function in a thread
of its own and succeeds once the function returned.

The function can either be a lambda expression or, for more complex code, a function.

```python
from coraplex.datastructures.enums import Arms
from coraplex.plans.function_call import FunctionCall
from coraplex.robot_plans.actions.core.robot_body import ParkArmsAction
from cramph.composites import Parallel


def code_test():
    print("-" * 20)
    print("Code function")


park = ParkArmsAction(Arms.BOTH)
code_lambda = FunctionCall(function=lambda: print("This is from the code object"))
code_func = FunctionCall(function=code_test)

plan = Parallel([park, code_lambda, code_func])

with simulated_robot:
    executor.compile(plan)
    executor.execute()
```

## Exception Handling

A {class}`~coraplex.plans.failures.PlanFailure` raised by a step makes that step fail, and its composite decides what
follows: a Sequence fails as well, while TryInOrder and TryAll go on with their other children and only fail if all of
them failed. Any other exception, such as a KeyError, is raised out of the execution.

We will see how a failure is handled at a simple example using TryAll, so that a failing step does not fail the whole
plan.

```python
from coraplex.plans.failures import PlanFailure
from coraplex.plans.function_call import FunctionCall
from coraplex.robot_plans.actions.core.navigation import NavigateAction
from cramph.composites import TryAll
from semantic_digital_twin.spatial_types import Pose


def code_test():
    raise PlanFailure


navigate = NavigateAction(Pose.from_xyz_rpy(1, 1, 0, reference_frame=world.root))
code_func = FunctionCall(function=code_test)

plan = TryAll([navigate, code_func])

with simulated_robot:
    executor.compile(plan)
    executor.execute()

print(plan.life_cycle_state)
print(code_func.life_cycle_state)
```

## Repeat

{class}`~giskardpy.motion_statechart.goals.templates.RepeatOnStall` attempts a task again whenever an attempt stops
making progress, until the task succeeds or its stop monitor fires. Counting the attempts with
{class}`~cramph.monitors.CountNodeResets` limits how often it tries, and the exception it is given is raised once the
attempts run out.

```python
from coraplex.plans.failures import RepetitionsExhausted
from coraplex.robot_plans.actions.core.robot_body import MoveTorsoAction
from cramph.composites import Sequence
from cramph.monitors import CountNodeResets
from giskardpy.motion_statechart.goals.templates import RepeatOnStall
from semantic_digital_twin.datastructures.definitions import TorsoState

move_torso = Sequence([MoveTorsoAction(TorsoState.HIGH), MoveTorsoAction(TorsoState.LOW)])

plan = RepeatOnStall(
    task=move_torso,
    stop_retry_monitor=CountNodeResets(node=move_torso, target=3),
    exception=RepetitionsExhausted(repeated_node=move_torso, maximum_repetitions=3),
)

with simulated_robot:
    executor.compile(plan)
    executor.execute()
```

## Monitors

A monitor lets you attach a condition to a part of the plan that is evaluated alongside it, so it can act on that part
while it is running. The condition is a statechart node, for instance a monitor that turns True after a fixed amount of
simulation time.

There are three monitored nodes for this:

* {class}`~cramph.composites.CancelledWhenTrue` stops its node once the monitor observes True and gives up on the plan
  with the exception it is given, such as {class}`~coraplex.plans.failures.PlanCancelled`, instead of leaving the rest
  of the plan waiting for a subtree that will not finish.
* {class}`~cramph.composites.PausedWhileTrue` holds its node for as long as the monitor observes True.
* {class}`~cramph.composites.PausedUntilTrue` holds its node until the monitor observes True, then lets it run.

For the example we will move the torso up and down, and stop it after 2 seconds of simulation time. Since cancelling
gives up on the plan, executing it raises {class}`~coraplex.plans.failures.PlanCancelled`.

```python
from coraplex.plans.failures import PlanCancelled
from coraplex.robot_plans.actions.core.robot_body import MoveTorsoAction
from cramph.composites import CancelledWhenTrue, Sequence
from cramph.monitors import CountSimulationTimeSeconds
from semantic_digital_twin.datastructures.definitions import TorsoState

two_seconds = CountSimulationTimeSeconds(seconds=2)

plan = CancelledWhenTrue(
    monitor=two_seconds,
    monitored_node=Sequence(
        [MoveTorsoAction(TorsoState.HIGH), MoveTorsoAction(TorsoState.LOW)]
    ),
    exception=PlanCancelled(monitor=two_seconds),
)

try:
    with simulated_robot:
        executor.compile(plan)
        executor.execute()
except PlanCancelled as cancelled:
    print(cancelled)
```

{class}`~cramph.composites.PausedUntilTrue` can be used the same way to launch a subtree in a paused state that is only
released once the monitor's condition is fulfilled.

```python
from coraplex.robot_plans.actions.core.robot_body import MoveTorsoAction
from cramph.composites import PausedUntilTrue, Sequence
from cramph.monitors import CountSimulationTimeSeconds
from semantic_digital_twin.datastructures.definitions import TorsoState

plan = PausedUntilTrue(
    monitor=CountSimulationTimeSeconds(seconds=2),
    monitored_node=Sequence(
        [MoveTorsoAction(TorsoState.HIGH), MoveTorsoAction(TorsoState.LOW)]
    ),
)

with simulated_robot:
    executor.compile(plan)
    executor.execute()
```
This will hold the wrapped plan for the first 2 seconds of simulation time before letting it run.
