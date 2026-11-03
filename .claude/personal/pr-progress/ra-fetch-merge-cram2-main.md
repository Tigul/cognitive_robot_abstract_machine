## ra-fetch-merge-cram2-main: cram2/main merged, motions survive world changes, Tiago demos ported
Draft PR on AICOR-Solutions, base ra-fetch-milestone; includes #5 (left open, closing it is the user's call).
Done 2026-11-03: merge; ControlLoop + StatechartCompiler.run_once_held_still; demos on statecharts; door frame+leaf kept;
IRII waiting demo removed; real-path fixes (removed-DOF state ignored, client catches up with the report it started on, Tiago diff drive).
Verified by manual runs through RealRobotPlanExecutor against a real-time paced standalone Giskard; not in the suite.
Next: Gazebo run when the user wants; then path planning into the cabin around people; tracker should smooth flicker.
Environment: krrood_spatial_math and griplink_interfaces needed; regenerate ORM when switching branches. Mark ready only when told.
2026-11-03: ported the say skill action (54b30c51e0 on ra-fetch-boarding-permission) and real_tiago_check.py here by hand, the plan layer differs: Speak(AwaitActionResult) in speech.py, SayAction waits, text_to_speech_action, RoomRequests uses the action. Uncommitted. Isolated domain: 54 demo tests, say through a local standalone Giskard with a stand-in skill pass. One full demo test run hit a 590 s timeout, not reproduced (boarding 3x, full 1x fine). Nothing run on the robot; robot was unreachable from the laptop at the end. Next: user follows the step-by-step guide; drive needs /elevator/joint_states and a building model.
