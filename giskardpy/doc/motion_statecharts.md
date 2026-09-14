# Motion Statecharts

Motion Statecharts are a core concept in Giskard for composing complex robot motions. They provide a structured way to manage the transition between different motion goals and monitors, making it easier to build robust and reactive robot behaviors.

## The Problem

Traditional robot motion planning often involves a sequence of fixed waypoints or a single, monolithic trajectory. This approach faces several challenges:

- **Complex Sequencing**: Coordinating multiple movements (e.g., "move arm to pre-grasp," then "close gripper," then "lift arm") can become hard to manage as the number of steps increases.
- **Error Handling**: What happens if a collision is detected mid-motion? Or if the gripper fails to close? Handling these contingencies in a flat script often leads to "spaghetti code."
- **Reactivity**: Modern robots need to respond to their environment. A simple trajectory doesn't easily allow for behavior like "move until a certain force is felt" or "stop if a human enters the workspace."

## How Motion Statecharts Solve It

Motion Statecharts address these issues by using a state machine-based approach to motion composition.
A Motion Statechart is a graph of **nodes**. Every node runs its own small state machine, and
the edges of the graph are the **conditions** under which one node's state machine reacts to
the state of other nodes. All nodes are updated together once per control cycle.

### Node types

- **Task**: A specific, single-purpose segment of the overall motion. Tasks add constraints to the motion problem and observe whether those constraints are currently satisfied. For example, a Cartesian position task observes whether the distance to its target is below a threshold.
- **Monitor**: A node that observes a condition without controlling the motion. For example, a monitor watching the distance between the gripper and a goal point, or a counter waiting for a number of control cycles.
- **CompositeStatechartNode**: A node that contains other nodes and wires their conditions. Composite statechart nodes encapsulate reusable, parameterized patterns, such as [the templates](#templates) that run steps in order or retry a failed motion.
- **Terminal node**: A node that ends the whole motion. **EndMotion** ends it successfully once it runs and observes True, **CancelMotion** ends it by raising its exception as soon as it runs.

Every node carries two pieces of state:

- an **observation**, which says whether what the node observes is currently True, False or Unknown, and
- a **life cycle state**, which says where the node is in its own execution.

## Observation state

The observation is a trinary value: **True**, **False** or **Unknown**. For a task it states
whether its constraints are currently satisfied, for a monitor whether its condition holds.
Unknown is needed because a node often cannot give an answer, for example before it has
ever run.

Only a running node observes. The observation is recomputed every control cycle and may
change in both directions:

| Life cycle state                  | Observation                                              |
|-----------------------------------|----------------------------------------------------------|
| NOT_STARTED                       | Unknown                                                  |
| RUNNING                           | recomputed every control cycle                           |
| PAUSED                            | frozen at its last value, because the node resumes later |
| SUCCEEDED, FAILED, INTERRUPTED    | Unknown, because the node never observes again           |

## Life cycle

A node's life cycle has six states. Only RUNNING is active: its constraints influence the
motion, its observation is recomputed and its `on_tick` callback is called. SUCCEEDED,
FAILED and INTERRUPTED are **final states**, and together they are the node's **outcome**.
A node only leaves a final state when it is reset.

```{mermaid}
flowchart LR
    entry(( )) --> NS([NOT_STARTED])
    NS -- start --> R
    subgraph active ["active"]
        direction TB
        R([RUNNING]) -- "pause is True" --> P([PAUSED])
        P -- "pause is not True" --> R
    end
    subgraph ended ["final, left only by reset"]
        direction TB
        S([SUCCEEDED])
        F([FAILED])
        I([INTERRUPTED])
    end
    active -- success --> S
    active -- fail --> F
    active -- interrupt --> I
    ended -- reset --> NS
    active -- reset --> NS

    classDef notStarted fill:#9CA3AF,stroke:#6B7280,color:#111111
    classDef running fill:#3B82F6,stroke:#1D4ED8,color:#FFFFFF
    classDef paused fill:#EAB308,stroke:#A16207,color:#111111
    classDef succeeded fill:#28A745,stroke:#15803D,color:#FFFFFF
    classDef failed fill:#EF4444,stroke:#B91C1C,color:#FFFFFF
    classDef interrupted fill:#F97316,stroke:#C2410C,color:#111111
    class NS notStarted
    class R running
    class P paused
    class S succeeded
    class F failed
    class I interrupted
```

Each transition is driven by one condition of the node:

| Transition | Condition attribute   | Default | From                          | To          |
|------------|-----------------------|---------|-------------------------------|-------------|
| start      | `start_condition`     | True    | NOT_STARTED                   | RUNNING     |
| pause      | `pause_condition`     | False   | RUNNING (True), PAUSED (False) | PAUSED, RUNNING |
| success    | `success_condition`   | False   | RUNNING, PAUSED               | SUCCEEDED   |
| fail       | `fail_condition`      | False   | RUNNING, PAUSED               | FAILED      |
| interrupt  | `interrupt_condition` | False   | RUNNING, PAUSED               | INTERRUPTED |
| reset      | `reset_condition`     | False   | any state                     | NOT_STARTED |

With the defaults a node starts right away and runs until the motion ends.

Unlike observations, conditions are two-valued: a transition happens when its condition is
**True**, and a paused node resumes as soon as its pause condition is False again.

**The condition that ends a node decides its outcome.** What the node observes at that moment
has no say in it:

- **SUCCEEDED**: the node's success condition held.
- **FAILED**: the node declared through its fail condition that it cannot continue.
- **INTERRUPTED**: the node's interrupt condition held, or one of its ancestors ended. Neither
  is a judgement of the node itself.

### When several conditions hold at once

A node takes exactly one transition per control cycle. If several conditions are True at the
same time, the first matching one in this order wins:

1. reset condition of the node or any ancestor
2. the node's own success condition
3. the node's own fail condition
4. the node's own interrupt condition, or any success, fail or interrupt condition of an ancestor
5. pause condition of the node or any ancestor
6. start condition

The ladder a RUNNING node goes through every control cycle:

```{mermaid}
flowchart TD
    A{"reset of the node or<br/>an ancestor is True?"}
    A -- yes --> NS([NOT_STARTED])
    A -- no --> B{"own success<br/>is True?"}
    B -- yes --> S([SUCCEEDED])
    B -- no --> C{"own fail<br/>is True?"}
    C -- yes --> F([FAILED])
    C -- no --> D{"own interrupt is True, or an<br/>ancestor's success, fail or<br/>interrupt is True?"}
    D -- yes --> I([INTERRUPTED])
    D -- no --> E{"pause of the node or<br/>an ancestor is True?"}
    E -- yes --> P([PAUSED])
    E -- no --> R([RUNNING])

    classDef notStarted fill:#9CA3AF,stroke:#6B7280,color:#111111
    classDef running fill:#3B82F6,stroke:#1D4ED8,color:#FFFFFF
    classDef paused fill:#EAB308,stroke:#A16207,color:#111111
    classDef succeeded fill:#28A745,stroke:#15803D,color:#FFFFFF
    classDef failed fill:#EF4444,stroke:#B91C1C,color:#FFFFFF
    classDef interrupted fill:#F97316,stroke:#C2410C,color:#111111
    class NS notStarted
    class R running
    class P paused
    class S succeeded
    class F failed
    class I interrupted
```

## Reading other nodes in conditions

Conditions are symbolic expressions over the state of other nodes, combined with
`logic_and`, `logic_or` and `logic_not`. Every variable a condition may read is a **predicate**
that is always True or False, so `not` is plain negation. A node's observation itself may be
Unknown, which is why a condition cannot read `observation_variable` or `last_observation`
directly (`UnsupportedConditionVariableError`); it asks about them through a predicate instead.

- **Observation predicates** ask what a node observes:

  | Predicate            | True while                                                  |
  |----------------------|-------------------------------------------------------------|
  | `observes_true`      | the node observes True                                      |
  | `observes_false`     | the node observes False                                     |
  | `last_observed_true` | the observation the node took most recently is True         |

  Unknown is neither True nor False, so a node that has not observed anything yet makes all
  three False. `observes_true` and `observes_false` turn False as soon as the node ends.
  `last_observed_true` keeps the answer the transition that ended the node saw, however the
  node ended, until a reset clears it. Read it to ask what a node saw, for example whether a
  monitor that ended itself had fired. It says nothing about how the node ended: a node
  interrupted while observing True still answers True, so ask a life cycle predicate when the
  verdict matters.

  Waiting for a monitor to turn True reads `logic_not(monitor.observes_true)`, which also waits
  while the monitor has not observed anything yet. `monitor.observes_false` would stop waiting
  only once the monitor observes False.
- **Life cycle predicates** such as `node.is_succeeded` answer questions about the life cycle
  state the node reaches this control cycle:

  | Predicate            | NOT_STARTED | RUNNING | PAUSED | SUCCEEDED | FAILED | INTERRUPTED |
  |----------------------|:-----------:|:-------:|:------:|:---------:|:------:|:-----------:|
  | `is_not_started`     | True        | False   | False  | False     | False  | False       |
  | `is_running`         | False       | True    | False  | False     | False  | False       |
  | `is_paused`          | False       | False   | True   | False     | False  | False       |
  | `is_terminated`      | False       | False   | False  | True      | True   | True        |
  | `is_succeeded`       | False       | False   | False  | True      | False  | False       |
  | `is_failed`          | False       | False   | False  | False     | True   | False       |
  | `is_interrupted`     | False       | False   | False  | False     | False  | True        |
  | `is_failed_or_interrupted` | False | False   | False  | False     | True   | True        |

  A negated verdict predicate is True before the node ends as well: `not node.is_succeeded`
  holds while the node runs. Waiting for a node to end any way but by succeeding reads
  `node.is_failed_or_interrupted`.
- **Settled life cycle predicates**, `node.has_succeeded` and
  `node.has_ended_without_succeeding`, answer the same questions about the state the node
  *entered* the control cycle with, which is what lets a condition read one about a direct
  child:

  | Predicate                     | NOT_STARTED | RUNNING | PAUSED | SUCCEEDED | FAILED | INTERRUPTED |
  |-------------------------------|:-----------:|:-------:|:------:|:---------:|:------:|:-----------:|
  | `has_succeeded`               | False       | False   | False  | True      | False  | False       |
  | `has_ended_without_succeeding`| False       | False   | False  | False     | True   | True        |

An observation may change in both directions, while an outcome stays fixed until a reset. A
condition that has to keep its answer after the node it reads has ended must therefore read
`last_observed_true`, or the outcome through a life cycle predicate, rather than
`observes_true`:

```{mermaid}
flowchart LR
    c1["RUNNING<br/>observes False<br/><b>last_observed_true: False</b>"]
    c2["RUNNING<br/>observes True<br/><b>last_observed_true: True</b>"]
    c3["RUNNING<br/>observes False<br/><b>last_observed_true: False</b>"]
    c4["RUNNING<br/>observes True<br/><b>last_observed_true: True</b>"]
    c5["SUCCEEDED<br/>observes Unknown<br/><b>last_observed_true: True</b>"]
    c6["SUCCEEDED<br/>observes Unknown<br/><b>last_observed_true: True</b>"]
    c1 --> c2 --> c3 --> c4 -- "success condition is True" --> c5 --> c6

    classDef running fill:#3B82F6,stroke:#1D4ED8,color:#FFFFFF
    classDef succeeded fill:#28A745,stroke:#15803D,color:#FFFFFF
    class c1,c2,c3,c4 running
    class c5,c6 succeeded
```

Conditions are checked when they are set and when the statechart is compiled:

- A condition may read its own node, a sibling (a node with the same parent) or a direct
  child; anything further away raises `ConditionScopeError`. A child may only be read through
  state it entered the control cycle with, its `last_observed_true` or one of its `has_*`
  predicates. A life cycle predicate of a child raises `ChildPredicateInConditionError`,
  because it answers about the state that child reaches this control cycle, which the node
  reading it is deciding at the same moment.
- A start condition may not read its own node.
- No condition may read an EndMotion or CancelMotion node, because nothing happens after one
  of them.
- Only predicates of nodes may appear in a condition, combined with `logic_and`, `logic_or`
  and `logic_not`; anything else, such as the trinary operators or the constant Unknown, is
  rejected because the condition could not be written down and read back.

A node's observation expression is trinary. It may read `observation_variable`,
`last_observation`, the observation predicates and the settled predicates of other nodes, but
not a life cycle predicate, and combines them with `trinary_logic_and`, `trinary_logic_or` and
`trinary_logic_not`. An observation that chooses between cases uses `trinary_if_cases`, which
selects a case only while its guard is True; `if_cases` would select it while the guard is
Unknown as well. It reads the observations the previous control
cycle left behind, so a node that ended on that cycle still shows the observation it ended
on; read the life cycle to tell the two apart.

### One control cycle

Every control cycle updates all observations first and all life cycle states second:

```{mermaid}
sequenceDiagram
    participant C as Control loop
    participant O as Observations
    participant L as Life cycle states
    C->>O: update
    Note over O: RUNNING nodes recompute their observation,<br/>PAUSED nodes keep it, all others are Unknown.<br/>Life cycle states are still the ones<br/>the control cycle started with.
    O->>L: update
    Note over L: every node takes one transition,<br/>using the new observations.<br/>on_start, on_pause, on_unpause, on_end<br/>and on_reset are called for changed nodes.
    L->>C: done
    Note over C: an EndMotion observing True ends the motion,<br/>a CancelMotion observing True raises its exception.
```

Within the life cycle update, a life cycle predicate reads the state its node reaches in the
**same** control cycle. A node waiting for another node's outcome, for example with
`start_condition = previous.is_succeeded`, therefore starts on the control cycle in which that
outcome is reached. A predicate a node reads about itself is the exception: it reads the state
the node started the control cycle with. Two nodes that read each other's predicates are
rejected with a `CyclicPredicateDependencyError`, since neither could be updated first.
Observation predicates read the observations updated earlier in the same control cycle, which were computed from the life cycle states the control cycle started
with.

## Who ends a node

> A node decides when it cannot continue. Its owner decides when it is done.

Failing has no physical consequence, so a node may declare it itself through its fail
condition. Succeeding does have one: a task that is ended stops being enforced, and the robot
can then be pulled out of the pose that task had just reached, for example by another task that
is still running. That is why a task never ends itself on reaching its goal; whoever runs it,
its **owner**, writes its success and interrupt conditions.

This splits nodes into two kinds:

- **`MaintenanceNode`**: a node whose observation says whether it has reached its goal, but
  which is only ever ended by its owner. Every `Task` is one, as are monitors watching a
  threshold (`PoseReached`, `JointPositionReached`, …), counters (`CountSeconds`,
  `CountControlCycles`), `Parallel` and the monitored composite statechart nodes.
- **`SelfDecidingNode`**: a node that can be ended without undoing what it did, and therefore
  ends itself. When the statechart is compiled, every such node gets its observation added to
  its success condition, so it succeeds once it observes True. Examples are `Attempt`, the
  ordering templates and nodes like `SetOdometry`.

Independent of how a node ends on success, a node can also be a **`SelfFailingNode`**: observing
False means it can no longer reach its goal. When the statechart is compiled, every such node gets
`not observation` added to its fail condition, so it fails once it observes False. `Attempt` and
the ordering templates are self-deciding and self-failing; `StoppedWhenTrue` and
`CancelledWhenTrue` are maintenance nodes that are self-failing.

`Attempt` is the bridge between the two: it runs a maintenance node and turns it into a node
that ends itself.

```{mermaid}
flowchart LR
    subgraph maintenance ["MaintenanceNode: ended by its owner"]
        direction TB
        task(["Task"])
        monitor(["threshold monitors,<br/>counters"])
        parallel(["Parallel"])
        monitored(["PausedWhileTrue, PausedUntilTrue,<br/>StoppedWhenTrue, CancelledWhenTrue"])
    end
    subgraph self_deciding ["SelfDecidingNode: ends itself"]
        direction TB
        attempt(["Attempt"])
        ordering(["Sequence, TryInOrder,<br/>TryAll, RepeatUntil"])
        other(["SetOdometry,<br/>SetSeedConfiguration"])
    end
    maintenance -- "wrapped in an Attempt" --> attempt
    self_deciding -- "usable as a step of" --> ordering
```

The ordering templates (`Sequence`, `TryInOrder`, `TryAll`, `RepeatUntil`) decide when their
children start and end, so they check every child they are given:

- A `MaintenanceNode` child is wrapped in an `Attempt` without failure monitors automatically.
- A child that is neither kind is rejected with a `NodeCannotDecideItselfError`, because
  nothing would ever move the template past it.
- A child whose start, pause, success, interrupt or reset condition was already set is
  rejected with a `ChildTransitionAlreadyWiredError`, because those are the template's to
  decide. The fail condition is exempt, since a node declares its own failure.

## Templates

The templates in `giskardpy.motion_statechart.goals.templates` and
`giskardpy.motion_statechart.monitors.templates` are composite statechart nodes that wire
their children for common patterns. In the diagrams below, an arrow from node A to node B
labelled `transition: expression` means that B's condition for that transition reads A.

### Attempt

Runs a `task` together with a list of `failure_monitors`, and ends as soon as either the task
reaches its goal or a monitor gives up on it.

- It observes **True** once the task's `last_observed_true` is True, and then succeeds.
- It observes **False** once any failure monitor's `last_observed_true` is True, and then fails.
  Reaching the goal wins if both happen on the same control cycle.
- It observes **False** as well once the task ended without succeeding, which is the task
  having concluded on its own; an attempt still waiting for it would never end.
- Otherwise it observes Unknown and keeps going.

The task is never ended by the attempt directly. It keeps being enforced until the attempt
itself ends and interrupts it. Each failure monitor succeeds on the control cycle it fires, so
`attempt.failure_reasons` can list the monitors that caused a failure after the fact. An empty
`failure_monitors` list states that the attempt cannot fail.

```{mermaid}
flowchart LR
    subgraph attempt ["Attempt"]
        direction TB
        task(["task"])
        m1(["failure monitor 1"])
        m2(["failure monitor 2"])
    end
    obs{"Attempt observes"}
    task -- "last_observed_true" --> obs
    m1 -- "last_observed_true" --> obs
    m2 -- "last_observed_true" --> obs
    obs -- "True: success" --> S([SUCCEEDED])
    obs -- "False: fail" --> F([FAILED])

    classDef succeeded fill:#28A745,stroke:#15803D,color:#FFFFFF
    classDef failed fill:#EF4444,stroke:#B91C1C,color:#FFFFFF
    class S succeeded
    class F failed
```

`RepeatOnStall` builds such an attempt for you, with a `Stalled` monitor as its failure monitor.

### Sequence

Runs its `nodes` one after another. Each step starts once the previous one has succeeded.

- It observes **True** once the last step succeeded.
- It observes **False** as soon as any step ended without succeeding.

```{mermaid}
flowchart LR
    subgraph sequence ["Sequence"]
        direction LR
        s1(["step 1"]) -- "start: is_succeeded" --> s2(["step 2"])
        s2 -- "start: is_succeeded" --> s3(["step 3"])
    end
    s3 -. "succeeded" .-> S([Sequence SUCCEEDED])
    sequence -. "any step ended without succeeding" .-> F([Sequence FAILED])

    classDef succeeded fill:#28A745,stroke:#15803D,color:#FFFFFF
    classDef failed fill:#EF4444,stroke:#B91C1C,color:#FFFFFF
    class S succeeded
    class F failed
```

### Parallel

Runs all of its `nodes` at the same time and observes **True** while at least
`minimum_success` of them (all of them by default) have `last_observed_true` True on the same
control cycle. A node that ended without succeeding stops counting, because the reading it
kept says where it was cut off rather than where it is, and `Parallel` fails once too few
nodes are left to reach `minimum_success` at all.

`Parallel` never ends any of its nodes: ending a task that reached its goal would let a node
that is still running pull the robot out of that goal again. For the same reason it is a
`MaintenanceNode` itself and never ends on its own. Put it into an `Attempt`, or hand it to an
ordering template, which does that for you, to get a step that finishes once all nodes are at
their goals together.

```{mermaid}
flowchart LR
    subgraph parallel ["Parallel"]
        direction TB
        n1(["node 1"])
        n2(["node 2"])
        n3(["node 3"])
    end
    count{"number of nodes with<br/>last_observed_true ≥<br/>minimum_success?"}
    n1 --> count
    n2 --> count
    n3 --> count
    count -- yes --> T["Parallel observes True"]
    count -- no --> Fa["Parallel observes False"]
```

### TryInOrder

Tries its `nodes` one after another and stops at the first one that succeeds. Each
alternative starts once the previous one ended without succeeding.

- It observes **True** as soon as an alternative succeeded.
- It observes **False** once every alternative ended without succeeding.

Each alternative decides for itself when to give up, typically as an `Attempt` with failure
monitors. An alternative without any way to fail keeps the later alternatives from ever
starting.

```{mermaid}
flowchart LR
    subgraph try_in_order ["TryInOrder"]
        direction LR
        a1(["alternative 1"]) -- "start: is_failed_or_interrupted" --> a2(["alternative 2"])
        a2 -- "start: is_failed_or_interrupted" --> a3(["alternative 3"])
    end
    try_in_order -. "any alternative succeeded" .-> S([TryInOrder SUCCEEDED])
    try_in_order -. "every alternative ended<br/>without succeeding" .-> F([TryInOrder FAILED])

    classDef succeeded fill:#28A745,stroke:#15803D,color:#FFFFFF
    classDef failed fill:#EF4444,stroke:#B91C1C,color:#FFFFFF
    class S succeeded
    class F failed
```

### TryAll

Runs all of its `nodes` at the same time and takes the first one that works.

- It observes **True** as soon as any alternative succeeded. `TryAll` then succeeds and
  interrupts the alternatives that are still running.
- It observes **False** once every alternative ended without succeeding.

### RepeatUntil and RepeatOnStall

`RepeatUntil` runs a `task` and resets it whenever it fails, until either the task succeeds or
`stop_retry_monitor` calls the retrying off.

- The task is wrapped in an attempt if it needs one. Its failure monitors decide what counts
  as a failed try.
- A failed try is reset on the next control cycle, as long as the stop monitor has not
  observed True. Resetting a goal resets everything below it, so a composite task starts over as a
  whole.
- Once the stop monitor's `last_observed_true` is True, the attempt is interrupted and not
  started again. A stop monitor that has not observed anything yet does not hold the attempt
  back.
- It observes **True** once the attempt succeeded, and **False** once the stop monitor
  observed True.
- If an `exception` is given, a `CancelMotion` raises it as soon as the stop monitor's
  `last_observed_true` is True.

`RepeatOnStall` is a `RepeatUntil` whose attempts fail once the task has not been approaching
its goal for `timeout`, measured by a `Stalled` monitor.

```{mermaid}
flowchart LR
    subgraph repeat ["RepeatUntil"]
        direction LR
        stop(["stop_retry_monitor"])
        attempt(["attempt"])
        cancel(["CancelMotion<br/>(only with an exception)"])
    end
    stop -- "start: not last_observed_true<br/>interrupt: last_observed_true" --> attempt
    attempt -- "reset: is_failed and not<br/>last_observed_true of the stop monitor" --> attempt
    stop -- "start: last_observed_true" --> cancel
```

### Monitored composite statechart nodes

These run a `monitored_node` next to a `monitor`, and let the monitor control the
monitored node's life cycle. They are maintenance nodes: their observation is the monitored
node's `last_observation`.

| Template            | Effect on the monitored node                                            |
|---------------------|-------------------------------------------------------------------------|
| `PausedWhileTrue`   | paused while the monitor observes True                                  |
| `PausedUntilTrue`   | paused while the monitor does not observe True, Unknown included        |
| `StoppedWhenTrue`   | interrupted once the monitor's `last_observed_true` is True             |
| `CancelledWhenTrue` | like `StoppedWhenTrue`, and a `CancelMotion` ends the whole motion      |

```{mermaid}
flowchart LR
    monitor(["monitor"])
    node(["monitored node"])
    monitor -- "PausedWhileTrue → pause: observes_true<br/>PausedUntilTrue → pause: not observes_true<br/>StoppedWhenTrue → interrupt: last_observed_true" --> node
```

`StoppedWhenTrue` observes True while the monitored node observes True or once it succeeded,
False once the monitor stopped it, and Unknown otherwise. It is a `SelfFailingNode`, so observing
False fails it: the monitored node is down by then, so nothing is being held any more, and
whoever runs it would otherwise wait for a subtree that can no longer arrive.

## Ending the motion

The motion ends once an `EndMotion` node is running and observes True. `EndMotion` observes
True once the robot has come to rest. A `CancelMotion` node ends the motion by raising its
exception as soon as it runs.

Both are usually created with factory methods that set their start condition:

| Factory                       | Starts once                                                              |
|-------------------------------|--------------------------------------------------------------------------|
| `when_true(node)`             | `node` observes True, or `node.is_succeeded` is True                     |
| `when_failed(node)`           | `node.is_failed` is True                                                 |
| `when_all_true(nodes)`        | every node observes True or has succeeded                                |
| `when_any_true(nodes)`        | any node observes True or has succeeded                                  |
| `EndMotion.when_false(node)`  | `node` currently observes False; this does not look at its outcome       |

## Example

A plan that first tries a slow approach and falls back to a fast one once the slow one takes
too long. The counters stand in for real tasks, so the example runs without a robot:

```python
from giskardpy.motion_statechart.goals.templates import Attempt, Sequence, TryInOrder
from giskardpy.motion_statechart.graph_node import EndMotion
from giskardpy.motion_statechart.monitors.payload_monitors import CountControlCycles
from giskardpy.motion_statechart.motion_statechart import MotionStatechart

motion_statechart = MotionStatechart()

slow_approach = Attempt(
    name="slow approach",
    task=CountControlCycles(name="slow", control_cycles=100),
    failure_monitors=[CountControlCycles(name="timeout", control_cycles=10)],
)
fast_approach = CountControlCycles(name="fast approach", control_cycles=5)

plan = Sequence(
    nodes=[
        TryInOrder(nodes=[slow_approach, fast_approach]),
        CountControlCycles(name="retreat", control_cycles=5),
    ]
)
motion_statechart.add_node(plan)
motion_statechart.add_node(EndMotion.when_true(plan))
```

After running it:

- `slow_approach` is FAILED, and `slow_approach.failure_reasons` names the `timeout` monitor.
- `fast_approach` and `retreat` were wrapped in attempts, which both SUCCEEDED.
- `plan` is SUCCEEDED, and the counters themselves are INTERRUPTED, because their attempts
  ended them.

## Benefits

- **Modularity**: Individual motions and checks are self-contained nodes that can be reused across different tasks.
- **Clarity**: The statechart structure provides a clear visual and logical representation of the robot's behavior.
- **Robustness**: Error handling and environment reactivity are built directly into the motion's structure through monitors and transitions, and every node that ended carries an outcome saying how.
- **Constraint-Based**: The constraints of all currently active tasks influence the motion, ensuring the robot satisfies all requirements simultaneously (e.g., "reach for the cup while keeping the arm away from the table").

For practical examples of how to use Motion Statecharts, see the [Basic Motion](examples/basic_motion.md) and [Cartesian Goals](examples/cartesian_goals.md) tutorials.
