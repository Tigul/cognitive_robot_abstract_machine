## door-leaf (Tigul#19, plan ra-fetch-upstream)

Plan:
- Tests first: port test_door_frame.py; add tests for "a recognised door is its own leaf" and DegreeOfFreedomLimits.validate_position_range.
- Source from ra-fetch-milestone (dbc7f9449d + the own-leaf rule from 17f4a9d95d): degree_of_freedom, exceptions, mixins (entity_holding, moving_body), Door (leaf, from_leaf, frame + leaf specification).
- Callers: kitchen_environment, building_floor, robocasa loader, sage-10k schema, plus any new door.root users on main (Elevator door poses, procthor).
- Update tests that assert the old meaning of door.root (api-break); run the semDT, ORM, ROS sync and coraplex/giskardpy door tests.

Done:
- Branch, draft PR #19, plan item recorded.

Next:
- Port test_door_frame.py and watch it fail.
