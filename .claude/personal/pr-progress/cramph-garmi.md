## cramph-garmi: interactive connection markers on a World

Plan (approved, see ~/.claude/plans/can-you-port-the-gleaming-brook.md):
- Port giskardpy tools/interactive_marker.py to SDT:
  adapters/ros/visualization/connection_markers.py
- ConnectionMarker family (6DoF, WheeledDrive, Prismatic, Revolute, Screw); soft connections skipped
- ConnectionMarkerServer (state callback) + model callback; live POSE_UPDATE writes state
- Extract the shared tf-publisher lookup from VizMarkerPublisher into a mixin (PositionedInTfTree)

Done (uncommitted in the working tree):
- connection_markers.py, PositionedInTfTree in tf_publisher.py, VizMarkerPublisher uses it
  (plus type-hint/docstring cleanup)
- test/semantic_digital_twin_test/test_ros/test_connection_markers.py (15 tests)
- test_ros directory: 324 passed, 1 skipped

Next:
- Manual RViz check (not done)
- Open: VizMarkerPublisher still raises ValueError in _select_shapes and assigns
  self.publisher while declaring _publisher - ask before changing
- Commit / PR only when asked

MJCF calibration (2026-10-20, uncommitted in the working tree):
- World.free_branch (swap parent connection for Connection6DoF, keep pose)
- adapters/mjcf_body_pose_writer.py (MJCFBodyPoseWriter, frame-aware, patches only
  changed <body> start tags) + adapters/exceptions.py
- semantic_digital_twin/scripts/calibrate_mjcf_body_poses.py (--free BODY..., Enter writes, q quits)
- tests: test_adapters/test_mjcf_body_pose_writer.py (7), test_world.py free_branch (2); all pass
- iai_maps commit 54ce90c (drawer unit + calibrated handles) not pushed: HTTPS remote has no credentials

Franka gripper tasks (2026-10-20, uncommitted in the working tree):
- ros_tasks.py: FrankaGripperTask (abstract), GripperMoveTask, GripperGraspTask, GripperHomingTask on franka_msgs
- tests: test_ros2_stuff/test_franka_gripper_tasks.py (4) + round-trip cases; 21 pass
- Open: CI image has no franka_msgs, so ros_tasks.py import fails there - ask how to provide it
