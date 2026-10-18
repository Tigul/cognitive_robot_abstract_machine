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

Done (2026-10-16): search sweeps a region - SweepRegion + SweepRegionPlacement;
LookAtAction targets a live FK expression (6d2a05aecb, Bind_on_tick reverted),
aim_offset/sideways_offset/PointingBeside removed (df406d6395). Pushed, PR
description updated. One region per robot, removed in SearchAction.cleanup
(b49016d524) - except on RealRobotPlanExecutor, which never calls cleanup on
client nodes (asked user). Detection still
one-shot after each look (user stashed that question). Pre-existing failure:
giskardpy test_attached_self_collision_avoid_stick; flaky:
test_collisions_are_computed_with_collision_avoidance_node.

Done (2026-10-16): PickUpAction.from_above (1dd6d43bb3), used by bowl-and-cup
and multi-robot tests. User dropped the real-robot cleanup question.

Done (2026-10-17): review - sweep region replaced by a cross search
(a894a367ca, cross_arm_length); pushed, 4 searching.py threads replied to and
resolved, PR description updated, PR still draft.

Done (2026-10-17): merged simon/cramph (68934be6c3; kw-only nodes, replace-by-holder,
child/closest_above renames, robot_adaptations); c488908516 fits multi-robot tests
(navigation tests skip differential drives; Stretch stands behind the cup).
Multi-robot file 257 passed. Pushed, PR description updated, PR still draft.

Done (2026-10-18): merged simon/cramph again (734ac2eaf, pushed, PR description
updated, PR still draft; actions are
the language node of their own steps, create_steps/create_goal). CarryAction is a
ParallelAction; upstream test changes to the moved robot-specific tests ported to
the multi-robot file. cramph tests 836 passed, test_plan 230, multi-robot 272 +
the one fixed Stretch case. Installed cramph editable into ~/envs/cram2 (was missing).

Done (2026-10-18, 70c0b8d05 pushed, PR description updated, PR still draft): one Statechart.replace(node, replacement) for
top-level nodes and children of plan language nodes (rollback via ReplacedNode),
replacing take_place_of, CramLanguageNode.replace, add_node_above,
NotATopLevelNodeError; Stretch base hold is a ParallelInsertion. cramph 831,
test_plan 230, multi-robot + test_actions 306 passed.

User keeps CramLanguageNode.closest_above. Done (2026-10-18, ebd39a92c pushed, PR description updated, PR still draft):
coraplex/config (ActionConfig) removed; CloseAction.closed_joint_position (0.01)
replaces closed_container_joint_state, execution_delay was unused. Test on all
robots; test_plan + test_orm 234 passed.

Next: further cleanup as the user asks; user marks ready when satisfied. Open follow-up ideas: keep held objects
upright during ParkArmsAction inside TransportAction; Tiago/HSR top-down reach
limits (joint-limit avoidance in giskardpy) - not in scope.
