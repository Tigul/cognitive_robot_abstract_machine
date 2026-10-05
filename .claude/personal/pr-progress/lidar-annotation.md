## PR #600 (cram2) Lidar annotation — Luca's review round of 2026-10-02

Done (uncommitted in working tree, user commits themselves):
- LSP: HasFingers min 2 / HasArms min 1 (MINIMUM_FINGER_COUNT/MINIMUM_ARM_COUNT),
  narrowing mixins call plain super().validate()
- removed dead MessageConverter._load_converter_modules
- np.minimum.at in Lidar._nearest_hit_per_beam
- removed ScanPattern.beam_directions_in_frame (test-only)
- use_source closes the replaced source; InputSource.close no-op; subscribed
  sources list the ROS subscriber base first so its close wins
- WorldStateInputs.read raises InputAlreadyReadError on a repeat
- lidar sources are pure; Lidar.get_lidar_reading adopts the scan pattern
- dropped LASER_SCAN placeholder on PR2/HSRB/Stretch/Tiago (TiagoTopic removed),
  documented ODOMETRY members
- source_family unwraps the Optional source field

Decisions: keep ClassVars (no reply), user answers test_leaked_worlds + krrood
ImportError threads. Draft replies for close/use_simulated_source/rename given in chat.

Known unrelated failure: test_world_pr2::test_robots_and_validate fails on DAiSy
collision SRDF (left_gripper_side_cylinder_link missing).

Next: user reviews + commits, posts replies, re-requests review.
