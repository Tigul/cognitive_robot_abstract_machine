Branch cramph-garmi-fix -> draft PR ichumuh/cognitive_robot_abstract_machine#22 (base cramph).

Original plan: fix test_reach_action_multi[garmi] / test_grasping[garmi] by reverting
Garmi's park pose to the Franka ready pose (58c7321e8, pushed, draft PR #22).

Review (ichumuh, CHANGES_REQUESTED 2026-10-14): the pose change hides a cramph
regression; the tucked pose works on main. User agrees.

Investigation 2026-10-15 - root cause found:
- cramph's KinematicallySimulatedPlanExecutor.default_executor_extensions builds
  MotionControl() -> QPControllerConfig.create_with_simulation_defaults()
  (braking_time 0.3 s -> jerk limit 4*0.2/0.3^2 = 8.9 for Garmi's arm joints).
- main's GiskardExecutable uses create_with_fast_simulation_defaults()
  (braking_time 0.1 s -> jerk limit 80). On cramph that factory has no caller.
- Recorded QP commands: first reach tick already differs (jerk 80 vs 8.889).
- With fast defaults on the cramph base (796d546df) + tucked pose: all 23 Garmi
  tests in test_multi_robot_action_designator.py pass.
- Perturbed tucked start poses: main 7/7 pass, cramph 0/7 -> not luck on main.
- From cramph's stall configuration both controllers stall (local minimum near
  j2/j4 limits); the slower ramp just leads cramph into it.

Next (awaiting user decision):
- New bug PR on cramph: kinematic executor uses fast simulation defaults, with a
  failing test first; run coraplex test dirs to check nothing tuned to 0.3 s.
- Then drop/close PR #22 (revert park-pose change) and reply to the review.
- bug label still needs a maintainer (Tigul can't edit PRs on ichumuh's repo).
