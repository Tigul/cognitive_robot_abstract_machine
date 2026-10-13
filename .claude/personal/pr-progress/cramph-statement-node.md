Branch `cramph-statement-node` (off `simon/cramph` @ d8184a3d4): plan language nodes
accept a krrood `Match` directly (future_problems.md, design 1, chosen 2026-10-13).
Plan: ~/.claude/plans/the-future-problems-md-sketches-a-parsed-iverson.md

Done (uncommitted, all in working tree):
- cramph: `StatementNode` (statement + candidate iteration, `grounded_nodes`),
  `ChosenChildObserver` context-extension interface notified by
  `CompositeNodeChoosingItsChild.adopt_chosen_child`; `CramLanguageNode` wraps a
  `Match` in `__post_init__`/`add_node`/`_insert_at`/`replace`.
- coraplex: `UnderspecifiedNode` + `ActionOfSteps` removed; chooser grounds
  `StatementNode`; `PlanRewriting` is a `ChosenChildObserver`; exception renamed
  `NotAStatementNode`; call sites/tests/experiments/docs migrated.
- Tests green: cramph_test (740), coraplex test_plan (192), test_designator +
  test_transporting (173), bowl/cup/training/orm/failure-handling (13), giskardpy
  motion_statechart (382; 1 pre-existing error from untracked sensor_monitors.py).

Deviations from plan: grounding state on the cramph node (context cleanup runs before
the candidate-limit check); rewriting via context observer, not a chooser hook
(test_the_opening_joins_the_sequence_a_grounded_pick_up_runs_in needs it chooser-independent);
plan transformations build `StatementNode` explicitly (AFTER-insertion anchors).

Next: user review; commit when asked. Open: keep or drop the future_problems entry
on step-field typing (user said Match is typed as what it matches).
Second change (same working tree, 2026-10-13): handle grasps sampled from geometry.
- SDT `Handle.grasp_candidates`: 4 grasps at origin, approach ⟂ bar (longest bbox
  extent), fingers across the bar; NoGraspGeometry without collision.
- coraplex `MovesHandle(Action, HasGraspChoice)` base: Open/CloseAction take
  `grasp`, `from_handle(handle, arm) -> Match`; `MoveAndOpenAction.from_standing_position`
  takes a grasp, new `from_handle_by_closest_grasps`; drawer transformation uses it.
- bowl_and_cup drops `add_handle_frame`; future_problems entry removed.
- Green: SDT annotations (183), coraplex test_plan, test_designator (146),
  transporting + bowl_and_cup (29).
Committed: statement node ac360cb82 (by user), handle grasps dd21667c8. Not pushed.
Next: push / PR when the user asks.
