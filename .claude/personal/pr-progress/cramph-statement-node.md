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
Merged simon/cramph (138964c02, future_problems conflict resolved), pushed to origin.
Draft PR: https://github.com/ichumuh/cognitive_robot_abstract_machine/pull/19 (base cramph).
Third change (2026-10-14): simulation time limit per started action (2b3983b6c):
`SimulatedPlanExecutor.simulation_time_per_action` (60 s), `simulation_time_limit` is a
property = budget x started Action nodes. Merged ichumuh/cramph (a00e08062, 8ae057c60);
merge fixes 1c779eecd: `RobotDemonstration.simulation_time_per_action` replaces Simon's
fixed limit, demos drop 10 min overrides, `_attempt_of` uses `AttemptRewritingItsTask`
again (Stretch demo stall), Simon's Stretch tests ported to `StatementNode`.
Green: both demos, cramph_test 748, cramera_test 1061, coraplex test_plan+demos 238.
Pushed to origin; PR #19 back to draft, description updated.
Simon's review (2026-10-14) on #19: blockers 1-4 + smaller points. Handle-grasp point done
in 70bfd601c: grasps at bar bbox centre; `MovesHandle.from_handle` ranks all 4 grasps by
`GraspCloseness` to the robot at grounding (`GraspsClosestToTheRobot`, coraplex/querying/
grasp_closeness.py); replied on PR. Open from review: split time-limit commit into own PR
(he objects: per-tick scan, nested actions counted, weak test), rewrite happens twice
(AttemptRewritingItsTask + observer: pick one), commented-out UnderspecifiedNode (gone),
panel.js still lists UnderspecifiedNode, type hints `StatechartNode | Match`,
candidate_limit as field, AGENTS style in new tests, per-pose grasp ordering in
from_handle_by_closest_grasps.
Double rewrite (16c7a02a9, pushed, PR description updated, 2026-10-14): user chose AttemptRewritingItsTask as the only
place; `ChosenChildObserver` removed from cramph (+ its cramph test), `PlanRewriting` is a
plain ContextExtension, test_the_opening_joins... builds AttemptRewritingItsTask. ORM
regenerated. Green: cramph_test 747, coraplex test_plan 198, designator/transporting/
bowl/failure 181, both demos.
Time-limit refinement (c7dd71b02, pushed, PR description updated, 2026-10-14): user keeps per-action budget over
Simon's fixed per-demo limit; default 40 s; `simulation_time_limit` is a field counted
from Action nodes in the statechart at compile + after_recompile (no per-tick scan).
New test test_the_simulation_time_limit_grows_when_an_action_joins; renamed
..._more_actions_it_holds. Green: test_plan+demonstrations 213, designator+transporting
176, both demos.
Action conditions removed (c91fcc427, pushed, PR description updated, 2026-10-14): all pre_/post_condition methods,
ConditionNotSatisfied, DesignatorParameters.bound_variables/_create_variables,
GripperIsFree/GripperIsNotFree/ToolFrameIsAtGrasp, test_action_conditions.py,
doc conditions.md (+toc, index/designators bullets). GripperHolds+GripperOccupancy kept
(PlaceAction uses them). ORM regenerated. Green: designator 139, test_plan+actions 223,
transporting/bowl/placing/orm 41, both demos.
Stretch mappings (2ba4e1974, pushed, PR description updated, 2026-10-14): investigated future_problems Stretch entries;
stale Stretch skips in test_transport/test_transport_open_container removed (8 passed, all
robots) + stale "No alternative motion mappings" comments; future_problems: both Stretch
entries merged into one real-robot-only check.
Next: remaining review points when asked.
