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

In progress (2026-10-16): replace the anchor-only park fix with
SequentialInsertion / ParallelInsertion subclasses of InsertionTransformation
(each wraps the anchor in a Sequence/Parallel when its language node is not
one). Commits: (1) cramph op wrapping an expanded node (incl. top-level, with
executor plan_nodes following), (2) the two subclasses + migrate DetectBeforeGrasp,
DrawerOpening, ParkArmsBeforeFirstAction (Sequential) and Justin torso
(Parallel), drop LAST_CHILD, (3) revert FirstActionNotRunInSequence and the
Sequence wrapper in the two-arm test. TDD each; update PR description.

Next after that: user marks ready when satisfied. Open follow-up ideas: keep held objects
upright during ParkArmsAction inside TransportAction; Tiago/HSR top-down reach
limits (joint-limit avoidance in giskardpy) - not in scope.
