# persistent-motion-state-chart

No pull request yet: creating one failed on the gh token (cram2 org lifetime
policy / 403); the user said to go ahead without it.

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

## Next
- Phase 1: failing cramph tests for CompositeNodeChoosingItsChild.
