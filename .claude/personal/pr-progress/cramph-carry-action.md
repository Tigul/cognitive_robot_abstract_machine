## cramph-carry-action — PR ichumuh/cognitive_robot_abstract_machine#25 (base: cramph)

Plan: CarryAction (NavigateAction + AlignPlanes per held object, z-axis up),
held = tool-frame child or earlier un-placed PickUpAction; MoveAndPlaceAction
drives with it.

Done (2026-10-15): action, integration, 5 tests in
test_actions/test_carrying.py, JSON round-trip case; ORM regenerated; tests
green except 2 pre-existing failures in test_bowl_and_cup.py (fail on cramph
too). Committed c9eb4dbb13, pushed to Tigul fork, draft PR opened.

Marked ready for review (2026-10-15). Next: await review / CI. Open follow-up idea: keep held objects upright
during ParkArmsAction inside TransportAction (not in scope).
