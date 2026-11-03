## ra-fetch-merge-cram2-main: cram2/main merged, motions survive world changes, Tiago demos ported
Draft PR on AICOR-Solutions, base ra-fetch-milestone; includes #5 (left open, closing it is the user's call).
Done 2026-11-03: merge; ControlLoop + StatechartCompiler.run_once_held_still; demos on statecharts; door frame+leaf kept;
IRII waiting demo removed; real-path fixes (removed-DOF state ignored, client catches up with the report it started on, Tiago diff drive).
Verified by manual runs through RealRobotPlanExecutor against a real-time paced standalone Giskard; not in the suite.
Next: Gazebo run when the user wants; then path planning into the cabin around people; tracker should smooth flicker.
Environment: krrood_spatial_math and griplink_interfaces needed; regenerate ORM when switching branches. Mark ready only when told.
