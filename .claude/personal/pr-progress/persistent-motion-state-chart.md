# persistent-motion-state-chart

Draft PR: Tigul#10 (https://github.com/Tigul/cognitive_robot_abstract_machine/pull/10).

## Plan (approved 2026-10-01, replaces the segment plan)
The statechart IS the plan: no parsing, no execution boundaries, plan layer
deleted. One PR, one commit per phase, each test-first and green on its own.
Full plan: ~/.claude/plans/i-want-to-further-polished-meadow.md
1. cramph: CompositeNodeChoosingItsChild + ChildChooser context extension
   (ChosenChild / NoChildLeft / ChoicePending), applied after tick callbacks
   inside modify(); suffix (de)serialization helpers.
2. coraplex: Plan = context + root chart node, perform() runs one chart with
   one EndMotion; factories build cramph composites; a(...) -> choosing node
   with UnderspecifiedChildChooser; remaining 15 ActionDescriptions -> Action
   (FullBody: root_link = world root, no flag toggling; Wiping: success OR
   final-waypoint monitor; Place: previous pick-up via chart navigation;
   ElevatorNavigation: MoveBranch).
3. coraplex: delete PlanNode family, language nodes, executables, Designator,
   ActionDescription; regenerate ORM; persistence/replay tests on Statechart.
4. giskardpy: ~/extend_goal JsonAction to extend a running goal,
   ChildSentByClient chooser on server, waiting indices in feedback; client
   loop in GiskardWrapper.execute; Plan.perform REAL serves choices.
Bookkeeping at the end: fold convert-imperative-actions and
retire-plan-layer into this item in plan.yaml; roadmap entry.

## Done
- Branch created off plan-cramp-second-iter (d8cd3dcc4), bootstrap commit pushed.
- Plan approved; branch up to date with origin/plan-cramp-second-iter.
- Phase 1 committed (dc331c495): CompositeNodeChoosingItsChild, partial JSON,
  deserialized nodes keep their id.
- Phase 2+3 committed and pushed (859e9abf9): Plan(root, context) +
  PlanExecution; factories build cramph composites; UnderspecifiedNode chart
  node + chooser + ActionTrial; 15 actions -> Action; plan layer deleted; ORM
  regenerated (Action's DesignatorParameters base moved last for ORMatic);
  training env keeps tried_actions. cramph 606 / coraplex 441 green.
- Fixes found on the way: DesignatorParameters eq=False; extensions hold still
  before a node chooses; tick budget counts sequence steps.

- daa4ea57c: motion server tests build statecharts with a context; server
  parses goals with the executor context; client reads results with statechart=.
- a87ef641a: split_list_by_type removed (user OK).
- b544b929f Phase 4: child_choices (ChildChoiceMessage, ChildSentByClient,
  ChildChoiceClient), feedback lists waiting nodes, GiskardWrapper.execute(..,
  child_chooser), PlanExecution REAL hands UnderspecifiedChildChooser;
  UnderspecifiedNode sent as CompositeNodeChoosingItsChild; Action._body serialized.
- plan.yaml/roadmap updated: convert-imperative-actions and retire-plan-layer
  folded into this item.

## Review round 1 (2026-10-05), plan ~/.claude/plans/i-added-some-comments-effervescent-barto.md
One commit each, then push, reply on threads, back to draft, update description.
1. Action._body -> _action_body
2. find_earlier_action -> Statechart.get_preceding_node_by_type
3. ChildChooser returns Optional[node] + has_choice_for; delete ChildChoice family
4. _joins_a_compiled_node uses node.parent_node (register first inside modify)
5. note in child_choices.py: interim mechanism
6. "WorldUpdates?": reply only (why the chooser waits on world updates)

## Next
- Known, not fixed (separate root causes, told user): giskardpy MoveGripper does
  not survive a JSON round trip (blocks real-stretch cross-process test);
  ~70 Statechart() calls without context in giskardpy ROS 2 integration tests
  and test_motion_goal.py.
