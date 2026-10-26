## cramph-garmi-backport: reusable GARMI hackathon changes back to cramph

PR: https://github.com/ichumuh/cognitive_robot_abstract_machine/pull/45 (draft, base ichumuh:cramph)

Plan: backport the user's selection from cramph-garmi, one commit per item, demo code left out.

Done (pushed to origin/cramph-garmi-backport, 2026-10-26):
- StartedWhenTrue; AllowCollisionBetweenFingers; description_corrections (no robot uses it,
  GARMI lift stays movable; GARMI fingertip/link8 fixes + regenerated srdf)
- DriveCommandFormat (TwistFormat default, TwistStampedFormat for GARMI), GarmiGiskard base weight
- Franka gripper tasks in giskardpy/middleware/ros2/franka_gripper_tasks.py, GARMI
  transformations + RealGarmiPlanExecutor in coraplex/robot_plans/garmi_franka_grippers.py;
  franka_msgs optional (ORM scan skips, tests importorskip)
- child choice before tick callbacks (partial port of feabd7d7f4)
- tests: cramph 812, coraplex test_plan 236, GARMI designator pass except pre-existing
  test_move_to_reach[garmi]

Not included / open:
- trials retrying raw nodes: needs the underspecified.py side-by-side rewrite (separate PR)
- MotionControl rest-of-uncommanded-DoFs: no failing test found, ask user for a repro
- not run: rest of coraplex, SDT, giskardpy suites; nothing tried on the robot
