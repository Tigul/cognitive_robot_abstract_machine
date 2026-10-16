## cramph-carry-action — PR ichumuh/cognitive_robot_abstract_machine#25 (base: cramph)

Plan: CarryAction (NavigateAction + AlignPlanes per held object, z-axis up),
held = tool-frame child or earlier un-placed PickUpAction; MoveAndPlaceAction
drives with it.

Done (2026-10-15): action, integration, 5 tests in
test_actions/test_carrying.py, JSON round-trip case; ORM regenerated; tests
green except 2 pre-existing failures in test_bowl_and_cup.py (fail on cramph
too). Committed c9eb4dbb13, pushed to Tigul fork, draft PR opened.

Marked ready for review (2026-10-15).

Done (2026-10-16): moved robot-specific action tests (carrying, reattaching
bodies, both arms, converted-action sections of the statechart-node test) into
test_multi_robot_action_designator.py; fixed ParkArmsBeforeFirstAction
inserting the park inside a Parallel (now before the nearest Sequence step,
FirstActionNotRunInSequence otherwise); cup tests park first, two-arm cups at
0.8 m (Tiago stalls at joint limits otherwise). 4 commits on top of
51582297a8, pushed, PR description updated, PR back to draft.

Done (2026-10-16): SequentialInsertion / ParallelInsertion (wrap anchor in a
new Sequence/Parallel via cramph CramLanguageNode.take_place_of, incl. top-level;
failed modify() puts taken-over nodes back), LAST_CHILD and
FirstActionNotRunInSequence removed, PlanRewriting.rewrite returns the node in
root's place. Commits 19ca98991e, 2371fbf223, c06f98039d pushed; PR description
updated; PR still draft. All related suites green.

Done (2026-10-16): search sweeps a region - GoalBindingPolicy.Bind_on_tick
(e0eaf62893), SweepRegion + SweepRegionPlacement, LookAtAction binds on tick,
aim_offset/sideways_offset/PointingBeside removed (df406d6395). Pushed, PR
description updated. Open: region stays in the world after the search
(removing it mid-run would rebuild nodes that reference it); detection still
one-shot after each look (user stashed that question). Pre-existing failure:
giskardpy test_attached_self_collision_avoid_stick; flaky:
test_collisions_are_computed_with_collision_avoidance_node.

Next: further cleanup as the user asks; user marks ready when satisfied. Open follow-up ideas: keep held objects
upright during ParkArmsAction inside TransportAction; Tiago/HSR top-down reach
limits (joint-limit avoidance in giskardpy) - not in scope.
