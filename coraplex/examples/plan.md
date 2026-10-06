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
# Introduction to Plans
A plan in CoraPlex is a statechart node that describes what the robot does, usually a cramph composite such as
`Sequence`, `Parallel` or `TryInOrder` holding actions. Plans are built from the constructs introduced in the
[Language](language.md) section, independently of the world they run in. A `PlanExecutor` then compiles the plan into
one statechart and executes it, in a simulated environment or on a real robot.

We will now go through a simple example of how to create and execute a plan.

# Setup a World

```python
from coraplex.execution_environment import simulated_robot
from coraplex.testing import setup_world
from coraplex.datastructures.dataclasses import Context
from semantic_digital_twin.robots.pr2 import PR2

world = setup_world()

pr2 = PR2.from_world(world)

context = Context(world, pr2)
```


## Example Plan

```python
from coraplex.robot_plans import *
from coraplex.datastructures.enums import Arms
from coraplex.robot_plans.actions.core.robot_body import ParkArmsAction
from coraplex.robot_plans.actions.core.navigation import NavigateAction
from cramph.composites import Sequence

navigate = NavigateAction(Pose.from_xyz_quaternion(1, 1, 0, reference_frame=world.root))
park = ParkArmsAction(Arms.BOTH)

plan = Sequence([navigate, park])
```

This creates a plan with a `Sequence` as its root and the two actions as its children.

## Plan Execution
A `PlanExecutor` executes plans in a context. `compile` builds the statechart running the plan, in the way the
execution environment in force asks for, and `execute` runs it until the plan succeeded. Underspecified actions are
grounded while the plan runs.

```python
from coraplex.plans.plan_execution import PlanExecutor

executor = PlanExecutor(context)
with simulated_robot:
    executor.compile(plan)
    executor.execute()
```

This will execute the plan in a simulated environment.

### Collision Avoidance
The execution environments accept a `collision_avoidance` flag. When set to `True`, collision avoidance goals are added
to the statechart a plan is compiled into, keeping the robot from colliding with the rest of the world while the
motions run.

```python
with simulated_robot(collision_avoidance=True):
    executor.compile(plan)
    executor.execute()
```

The flag also works when constructing an environment directly, and is restored correctly for
nested environments:

```python
from coraplex.datastructures.enums import ExecutionType
from coraplex.execution_environment import ExecutionEnvironment

with ExecutionEnvironment(ExecutionType.SIMULATED, collision_avoidance=True):
    executor.compile(plan)
    executor.execute()
```

## Inspecting a Plan

Every node of an executed plan reports how its run went:

* life_cycle_state: Whether the node has not started, is running or paused, or succeeded, failed or was interrupted
* start_time/end_time: When the node started and ended

```python
print(plan.life_cycle_state)
print(plan.children[0].life_cycle_state)
print(plan.start_time)
print(plan.end_time)
```

You can open an interactive visualization of the statechart the plan ran in using its `visualize` method.

```python
plan.statechart.visualize()
```
