# Unifying coraplex's Plan with cramph's Statechart

## Why this plan exists

coraplex carries **two parallel models for describing behaviour**, and they
duplicate each other:

1. **`Plan` / `PlanNode`** (`coraplex/plans/plan.py`, `plan_node.py`) — a
   rustworkx tree built and run eagerly. `ActionDescription.expand()` /
   `execute()` build a `PlanNode` subtree, and `PlanNode.perform()` walks it
   synchronously (`notify()` → `parse()` → `execute()`), using `PlanFailure`
   exceptions for control flow.
2. **`Statechart` / `CompositeNode`** (`cramph/node.py`, `statechart.py`) — a
   reactive, tick-based state machine where nodes wire each other's life-cycle
   transitions with symbolic conditions, built once and compiled (frozen)
   before it ever ticks.

They are already bridged, but one layer below `ActionDescription` and only
transiently: `BuildsMotionStateChart` / `GiskardExecutable` build a **fresh,
throwaway** `cramph.Statechart` per batch of plan children with no execution
boundary between them, compile it, tick it to completion, and discard it.

Worse, `coraplex/language.py` re-implements a whole second vocabulary of
`Sequence` / `Parallel` / `TryInOrder` / `RepeatUntil` / monitored-pause /
cancel nodes as imperative `PlanNode`s — even though each one already declares
its exact cramph equivalent via `motion_state_chart_template`, and
`add_to_motion_state_chart` already builds the real composite. Only the
imperative `notify()` path is redundant with what the chart-building path
already does.

**The end state:** retire the Plan tree entirely; `Statechart` becomes the
single persistent structure for both authoring and execution, gaining the
`Plan` conveniences that still make sense for a reactive, compile-once
structure. The lever is `ActionDescription` — it stops being a `Designator`
that builds `PlanNode` subtrees and becomes a `CompositeNode` that expands
into statechart nodes directly.

## Verified constraints (checked against source, not assumed)

These three findings each changed the design, and each cost real
investigation — do not re-litigate them from memory.

### `ActionDescription` must subclass `CompositeNode`, never `Sequence`

`CramLanguageNode` (`cramph/composites.py`) redefines
`nodes: List[StatechartNode] = field(default_factory=list, init=True)` —
**not** kw_only. That makes it the first defaulted, `init=True` parameter in
the whole `Sequence` hierarchy. Concrete actions declare their own fields as
plain non-default, non-kw_only (`MoveTorsoAction.torso_state: TorsoState`,
`ParkArmsAction.arm: Arms`, `SetGripperAction.gripper: Arms` / `motion:
GripperState`), which would land *after* `nodes` and raise
`TypeError: non-default argument follows default argument` at class-definition
time.

Making `CramLanguageNode.nodes` kw_only instead is **not** an escape hatch:
`Sequence(...)` / `Parallel(...)` are called positionally throughout
`giskardpy`, `coraplex`, `experiments` and both test suites.

`CompositeNode` and `StatechartNode` contribute **zero** positional `__init__`
parameters (`StatechartNode.name` is `field(default=None, kw_only=True)`;
`CompositeNode.nodes` is `init=False`) — the same property that makes today's
`Designator` base work. So: subclass `CompositeNode`, and own one internal
`Sequence` as the single child, mirroring `MonitoredCompositeNode`'s idiom of
a composite that forwards its inner node's `last_observation`.

### `NavigateAction` cannot be in the first conversion slice

It is resolved generatively via `a(NavigateAction)(...)` in
`coraplex/robot_plans/actions/composite/transporting.py`, `experiments/`, and
`test/coraplex_test/test_designator/test_underspecified_designator.py`.
`UnderspecifiedNode` / `ActionTrial` construct `ActionNode(designator=...)` and
call `.perform()` / `.notify()` on it — machinery that is itself deferred. So
converting `NavigateAction` early breaks `a(NavigateAction)(...)` resolution.

`MoveTorsoAction`, `ParkArmsAction` and `SetGripperAction` have **zero**
`a(...)` / `an(...)` usage anywhere in the repo (verified by grep) and are the
safe first slice. They also happen to minimize condition complexity: only
`MoveTorsoAction` has a condition at all (a post-condition), so the
condition-wiring can be proven on one simple case.

### Where this work lands, and what it stacks on

Work lands in two stages:

1. **Every item here is a pull request into `plan-cramp-second-iter`** on the
   Tigul fork, so each review sees only that item's own diff.
2. **`plan-cramp-second-iter` then goes to `ichumuh:cramph`**, the shared
   integration branch for the cramph package, carrying the collected work.

Nothing here targets `cram2/main`. `ichumuh#7` (`Tigul:cramph-new-motions` →
`cramph`, "Rip MotionDesignator") is the precedent for that second stage.

Verified at plan creation:

- `simon/cramph` (that integration branch) holds the cramph package and is
  **42 commits ahead of `cram2/main`**.
- `plan-cramp-second-iter` is **7 commits ahead of it and 0 behind** — graph
  navigation, start/end times, life-cycle callbacks, the candidate-generator
  move and typing fixes. Some of that is already Statechart-toward-Plan
  parity work, i.e. this plan's own direction arriving ahead of the plan.
- Those 7 commits touch `cramph/{statechart,node,context,data_types}.py` and
  `coraplex/plans/{plan_node,executables,underspecified}.py`, which is why
  later items branch off `plan-cramp-second-iter` rather than off the base.

**A correction worth recording:** an earlier pass of this analysis claimed
the cramph package existed on exactly one branch anywhere with no PR. That
was wrong — it came from `git branch --contains` over refs that had never
had `simon` fetched. cramph is perfectly well established on the integration
branch; only the 7 commits on top of it are unlanded.

**The base is currently broken**, inherited rather than introduced here: the
"Rip MotionDesignator" commit (`3cb9824eb`, an ancestor of `simon/cramph`,
landed out-of-band while its pull request shows closed) removed `MoveMotion`,
`ClosingMotion`, `MoveGripperMotion` and `LookingMotion`, but
`alternative_motion_mappings/{tiago,stretch,hsrb}_motion_mapping.py` still
import them. Confirmed with a real `ImportError`, not inferred.

## The first conversion PR (`action-description-composite-node`)

```python
@dataclass(eq=False, repr=False)
class ActionDescription(DesignatorParameters, CompositeNode, ABC):
    success_decided_by = SuccessDecider.ITSELF
    fails_when_observing_false = True

    _body: Sequence = field(init=False, repr=False)

    @property
    @abstractmethod
    def _sub_nodes(self) -> List[StatechartNode]:
        """Replaces `_action_plan`: the child nodes this action runs, in order."""

    def expand(self, context: StatechartContext) -> None:
        self._body = Sequence(name=f"{self.name}/body", nodes=list(self._sub_nodes))
        self._add_child_to_statechart(self._body)
        self._wire_conditions(context)

    def build_artifacts(self, context: StatechartContext) -> NodeArtifacts:
        return NodeArtifacts(observation=Scalar(self._body.last_observation))
```

This reuses `Sequence`'s own `_adopt` / `_wire_children` / `build_artifacts`
for free, including auto-wrapping any `OWNER`-deciding child (e.g. `Parallel`)
in a bare `Attempt` — exactly what `execute_single` / `sequential` do today.

`_sub_nodes` replaces `_action_plan` mechanically for this slice:

| Action | was | becomes |
| --- | --- | --- |
| `MoveTorsoAction` | `execute_single(JointPositionList(...))` | `[JointPositionList(...)]` |
| `SetGripperAction` | `sequential([gripper_goal(...) for arm in arms])` | `[gripper_goal(...) for arm in arms]` |
| `ParkArmsAction` | `execute_single(Parallel([joint_goal, JointVelocityLimit(...)]))` | `[Parallel([joint_goal, JointVelocityLimit(...)])]` |

### Why three roadmap steps became one item

The approved plan-mode plan listed the `Designator` split and the
`CoraplexContextExtension` as separate roadmap entries, but its own
"Files touched (first PR)" section bundled all three together.
`.claude/skills/add-plan-item/scope-decision.md` settles it: prefer the
change, because work is genuinely new only if it stands on its own once the
work before it lands. Neither prerequisite does — landing the `Designator`
split alone ships a mixin nothing consumes, and landing the context extension
alone ships a `ContextExtension` nothing reads. Both fold into the conversion.

### Pieces that ship inside that same item

- **`Designator` split.** `Designator`'s only field, `plan_node`, is
  `kw_only=True, init=False`; `fields` / `designator_parameter` /
  `bound_variables` / `_create_variables` are pure dataclass introspection
  needing no `plan_node`. Extract those into a `plan_node`-free mixin; keep
  `Designator` on top of it for every unconverted action.
- **coraplex `ContextExtension`.** A `CompositeNode`-based action only has
  `StatechartContext`, which has no `robot` / `evaluate_conditions` /
  `ticks_per_motion` / `query_backend`. Those cannot move onto
  `cramph.context.StatechartContext` — `cramph` may never import `coraplex`
  (enforced by `test/cramph_test/test_package_dependencies.py`) — so they ride
  in a coraplex-defined extension registered the way `RosNodeAccess` /
  `MotionControl` already are in `GiskardExecutable.create_executor`.
- **Pre/post conditions.** `GiskardExecutable._add_condition_monitors` is dead
  code today ("being reworked") but is exactly the pattern to inline into
  `expand()`: a `ThreadedPredicateMonitor` wrapping the EQL condition, gating
  the body's `start_condition`, with a `CancelStatechart` on the false
  observation. `condition_nodes.condition_monitor` already builds this monitor.
- **The `make_node` bridge.** `factories.make_node` dispatches
  `isinstance(action_like, ActionDescription) → ActionNode(designator=...)`.
  Once `ActionDescription` is no longer a `Designator`, that branch silently
  stops matching — and `transporting.py` already embeds `ParkArmsAction` /
  `MoveTorsoAction` in `sequential([...])`. So a new
  `ActionCompositeNode(PlanNode, BuildsMotionStateChart)` wrapper ships in the
  same PR, incrementing `motion_count` by the action's actual
  `MotionStatechartNode` descendant count rather than `MotionNode`'s flat `+1`
  (one converted action can expand into several motions).

## What deliberately does not port from `Plan`

Not an oversight — these assume a mutable, restartable, single-shot tree that
a compiled statechart structurally is not:

- `Plan.simplify()` / `PlanNode.merge` / `DesignatorNode.simplify()` — depend
  on removing edges and nodes after they exist; `Statechart.compile()` freezes
  the node set (`StatechartAlreadyCompiledError`). A *pre-compile* dedup pass
  is a different, narrower feature, and nothing in this migration needs it.
- `Plan.re_perform()` / `Plan.replay()` — assume eager, restartable leaf
  execution. The tick-based equivalent is reset-and-re-tick, which already
  exists via `reset_condition` / `on_reset`; it is a different mechanism, not
  a port.

And for `layers` / `visualize()`, note that `Statechart.rx_graph`'s edges are
**transition-condition dependencies**, not parent/child structure (see
`_add_transitions` / `_create_edge_for_condition`). `Plan.layers` can call
`rx.layers` on its graph directly; `Statechart` cannot. `layers` must BFS over
`StatechartNode.children`, and `visualize()` needs its own presentation graph,
the way `StatechartGraphviz` already builds one for `draw()`.

## Standing conventions for this plan

- **`cramph` never imports from `coraplex` or `giskardpy`** — enforced by
  `test/cramph_test/test_package_dependencies.py`. Every item here keeps its
  changes inside `coraplex`, except `statechart-plan-parity`, which is purely
  inside `cramph`.
- **Test-driven, per `AGENTS.md`** — prove a behaviour with a failing test
  first, and never modify a test to make it pass.
- **Converted actions get tick-based tests**, mirroring
  `test/cramph_test/test_statechart/test_cram_language_node.py`'s compile-and-
  tick helper rather than `Plan.perform()`:

  ```python
  statechart.add_node(action)
  statechart.add_node(EndStatechart.when_true(action))
  executor.compile(statechart)
  executor.tick_until_end(timeout=...)
  assert action.life_cycle_state == LifeCycleValues.SUCCEEDED
  ```

- **Converting an action changes how it appears in an unconverted plan tree**,
  so some existing tests do change with each conversion batch — an
  expectation this plan originally got wrong. Where the converted action was
  only a fixture, point the test at one still on the old path; where the test
  is about how an action parses, assert what it expands into now. What must
  never change is behaviour: every `perform()`-based test kept passing
  untouched.
- **Count a contributed node as one motion.** `motion_count` cannot be derived
  by counting what an action expands into, because a template assembling its
  children hands them to a goal that does not belong to a chart yet, so
  nothing has expanded. Getting this wrong makes `execute()` bail on
  `motion_count == 0` and the robot silently does nothing.
- Run tests with the project interpreter (`/home/jonas/envs/cram/bin/python`)
  and `pytest -n 2`.
- **Every pull request targets `ichumuh:cramph`**, based on
  `plan-cramp-second-iter` (or on whichever earlier item it depends on),
  never on `cram2/main`.

## History

- **Created 2026-09-23** from a Claude Code plan-mode plan approved earlier in
  the same session, after exploration of both node models, the existing
  bridge, and `coraplex/language.py`'s duplicate vocabulary.
- Scope decisions taken at creation, all confirmed with the user rather than
  inferred: full unification as the end goal; declarative actions converted
  first with imperative ones explicitly deferred; the three prerequisites
  folded into one conversion item; `plan-cramp-second-iter` tracked as the
  foundation item; no tracking issue for now (Issues are disabled on the
  Tigul fork, and opening one on the shared `cram2` upstream was declined).
- Per-branch working detail continues to live in
  `.claude/personal/pr-progress/<branch>.md`, independently of this plan.
