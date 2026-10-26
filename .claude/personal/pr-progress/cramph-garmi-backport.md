## cramph-garmi-backport: reusable GARMI hackathon changes back to cramph

PR: https://github.com/ichumuh/cognitive_robot_abstract_machine/pull/45 (draft, base ichumuh:cramph)

Plan: backport the user's selection from cramph-garmi, one commit per item, demo code left out.

Done (pushed to origin/cramph-garmi-backport, 2026-10-26):
- StartedWhenTrue; AllowCollisionBetweenFingers; GARMI fingertip/link8 fixes + regenerated srdf
- DriveCommandFormat (TwistFormat default, TwistStampedFormat for GARMI), GarmiGiskard base weight
- Franka gripper tasks in giskardpy/middleware/ros2/franka_gripper_tasks.py, GARMI
  transformations + RealGarmiPlanExecutor in coraplex/robot_plans/garmi_franka_grippers.py;
  franka_msgs optional (ORM scan skips, tests importorskip)
- child choice before tick callbacks (partial port of feabd7d7f4)
- review round 1 (2026-10-26, pushed 3a90c8acc8, 4d5199c0ef; threads replied + resolved):
  _step_holding -> ControlStructureNode.step_holding; twist docstring; description
  corrections dropped (user chose). Removal landed in the step_holding commit: force-push
  to split it was denied, left as is
- tests: cramph 812, coraplex test_plan 236, GARMI designator pass except pre-existing
  test_move_to_reach[garmi]

- CI round 1 (2026-10-26, pushed 7a57298d19, 195caf1786): garmi_franka_grippers moved to
  coraplex/ (keyword-only audit walks robot_plans); GARMI base full_body_controlled=True
  (fingertip pads put grasp point 4.37 cm further out, beyond reach with a fixed base).
  Locally vs garmi_description main via scratch AMENT prefix: designator 352 pass,
  test_plan + SDT robots 462 pass
- CI image rebuilt 2026-10-26 14:13 with garmi_description main + new iai_maps apartment

Not included / open:
- GARMI demo fails (CandidateLimitReached); plain cramph fails too on the new apartment
  (CollisionViolatedError). Adjusted demo is on cramph-garmi but needs GraspOpenAndRetractAction (#6)
- user's local ~/workspace/ros install of garmi_description is stale: colcon build
  --merge-install fails on unbuilt hardware_interface_testing / gz_ros2_control
- trials retrying raw nodes: needs the underspecified.py side-by-side rewrite (separate PR)
- MotionControl rest-of-uncommanded-DoFs: no failing test found, ask user for a repro
- not run: rest of coraplex, SDT, giskardpy suites; nothing tried on the robot
