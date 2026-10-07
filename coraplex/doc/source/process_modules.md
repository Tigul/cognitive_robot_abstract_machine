# Motion Execution in CoraPlex

Motion execution is CoraPlex's bridge from symbolic intentions to concrete robot motions. It translates the motion
designators produced by a plan into giskard motion state charts and runs them, either in simulation or on a real robot.
By keeping the "how" of actuation behind a common abstraction, plans remain robot-agnostic and execution-aware without
being robot-specific.

```{note}
Earlier versions of CoraPlex used a `ProcessModule`/`ProcessModuleManager` mechanism. That layer has been
replaced by the motion / executable / execution-environment model described here.
```

## Motions

A motion is a giskard node, built by the action that wants it as one of the statechart nodes the action expands
into. Actions build them straight from the tasks and goals in
{mod}`giskardpy.motion_statechart` (for example {class}`~giskardpy.motion_statechart.tasks.joint_tasks.JointPositionList`
to move a joint, or {class}`~giskardpy.motion_statechart.goals.gripper.MoveGripper` to open a gripper).

An action resolves what is coraplex's own before it builds a goal: an arm's tool frame comes from its
{class}`~semantic_digital_twin.robots.robot_parts.EndEffector`, goal-achievement thresholds fall back to
the {class}`~coraplex.plans.context_extensions.MotionToleranceConfig` of the context, and the link a Cartesian
goal is expressed relative to comes from
{attr}`~coraplex.plans.context_extensions.RobotAccess.controlled_root`. The mixins in
{mod}`coraplex.robot_plans.mixins` do this for the goals several actions share.

## The Plan Executors

A {class}`~coraplex.plans.executors.PlanExecutor` runs a plan, the top-level nodes of a
{class}`~cramph.statechart.Statechart` built in its context. It builds that context over its world out of the context
extensions it is given, such as the {class}`~coraplex.plans.context_extensions.RobotAccess` of the robot performing
the plan. {meth}`~coraplex.plans.executors.PlanExecutor.compile` adds the collision avoidance goals when collision
avoidance is enabled and an end motion once every plan node succeeded, and
{meth}`~coraplex.plans.executors.PlanExecutor.execute` runs the plan. An executor runs one statechart, so every plan
gets an executor of its own.

## Choosing Between Simulated and Real Execution

Which executor runs a plan decides how it is executed:

```python
from coraplex.plans.context_extensions import RobotAccess
from coraplex.plans.executors import RobotPlanExecutor, SimulatedPlanExecutor
from cramph.statechart import Statechart

executor = SimulatedPlanExecutor(world, context_extensions=[RobotAccess(robot)])
statechart = Statechart(context=executor.context)
statechart.add_node(plan)
executor.compile(statechart)
executor.execute()
```

A plan for the real robot is built and run the same way in a
`RobotPlanExecutor(world, context_extensions=[RobotAccess(robot)], ros_node=node)`.
{meth}`~coraplex.plans.executors.PlanExecutor.type_for` returns the executor of an
{class}`~coraplex.datastructures.enums.ExecutionType`, for code that is told how to execute at run time. The nodes of
a plan read how they are executed from the {class}`~coraplex.plans.context_extensions.ExecutionMode` the executor puts
into the context.

Collision avoidance is a setting of the executor:

```python
executor = SimulatedPlanExecutor(
    world, context_extensions=[RobotAccess(robot)], collision_avoidance=True
)
```

## What happens for each executor

- {class}`~coraplex.plans.executors.SimulatedPlanExecutor`: the chart is compiled and ticked against the world until
  every plan node succeeded. A plan that stops approaching its goal gives up and raises
  {class}`~coraplex.plans.failures.MotionMadeNoProgress`, naming the tasks that stalled; one that keeps converging is
  never cut off for taking many ticks, only once it exceeds the simulated time limit.
- {class}`~coraplex.plans.executors.RobotPlanExecutor`: the chart is sent to giskard via the `GiskardWrapper`, which
  grounds underspecified actions while giskard runs it.
- {meth}`~coraplex.plans.executors.PlanExecutor.prepare` builds the chart without compiling or running it, for
  inspecting a plan as it would run.

## Robot-Specific Motions

Some robots need a different implementation of a motion.

```{warning}
The {class}`~coraplex.alternative_motion_mapping.AlternativeMotion` mechanism keyed off the motion designators that
this layer replaced, so the mappings in {mod}`coraplex.alternative_motion_mappings` no longer import. Robot-specific
overrides are being rebuilt on top of the giskard goals.
```

## Key takeaways

- Actions build giskard goals directly; plans never execute them one at a time.
- A {class}`~coraplex.plans.executors.PlanExecutor` compiles the statechart holding a plan and executes it.
- The executor's type chooses simulated or real execution; collision avoidance is one of its settings.
