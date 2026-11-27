## door-leaf (Tigul#19, plan ra-fetch-upstream)

Plan:
- Port the frame + leaf door from ra-fetch-milestone (dbc7f9449d, own-leaf rule from 17f4a9d95d) onto main's API (ScrewAxisConnection, HasParts.add(part=...)).
- Update tests that assert the old meaning of door.root (api-break), then run semDT, coraplex, giskardpy door tests.

Done:
- test_door_frame.py (10 tests, incl. own-leaf rule and validate_position_range); all pass.
- Door.leaf/moving_body/entity_holding/from_leaf, frame + leaf specification; HasMovableJoint.moving_body; HasParts.entity_holding (PartWholeRelationship no longer exists on main); DegreeOfFreedomLimits.validate_position_range; DoorLeafNotFoundError.
- Callers: kitchen_environment, building_floor, robocasa loader (Door.from_leaf), sage-10k schema; test_sage10k colours the leaf.
- Old-contract tests updated in test_factories, test_specifications, test_semantic_annotations, test_annotation_mass: 455 pass. coraplex/giskardpy/sage door tests: 67 pass.
- Env: ~/envs/cram2 resynced to main's split krrood (uv sync --inexact); ORM regenerated.

Next:
- Remaining semDT test dirs (running), commit, push, update the PR description.
