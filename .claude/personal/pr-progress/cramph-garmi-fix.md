Branch cramph-garmi-fix -> draft PR ichumuh/cognitive_robot_abstract_machine#22 (base cramph).

Plan: fix test_reach_action_multi[garmi] and test_grasping[garmi] by reverting
Garmi's arm park pose to the Franka ready pose (fr3_joint2 -0.785, fr3_joint3 0).

Done:
- 58c7321e8 park pose reverted in garmi.py; all 23 Garmi cases in
  test_multi_robot_action_designator.py pass (21/23 before), other Garmi tests pass.
- Pushed to origin (Tigul fork), draft PR #22 opened.

Next / outstanding:
- `bug` label could not be set: Tigul lacks permission to edit PRs on ichumuh's repo;
  needs a maintainer to add it.
- Open question for the author of b79469e5a: why was the tucked-in park pose chosen?
