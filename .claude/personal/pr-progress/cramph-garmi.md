## cramph-garmi: interactive connection markers on a World

Plan (approved, see ~/.claude/plans/can-you-port-the-gleaming-brook.md):
- Port giskardpy tools/interactive_marker.py to SDT:
  adapters/ros/visualization/connection_markers.py
- ConnectionMarker family (6DoF, WheeledDrive, Prismatic, Revolute/Screw); soft connections skipped
- ConnectionMarkerServer (state callback) + model callback; live POSE_UPDATE writes state
- Extract the shared tf-publisher lookup from VizMarkerPublisher into a mixin
- Tests in test/semantic_digital_twin_test/test_ros/test_connection_markers.py (written first)

Done:
- Branch synced with the fork (already up to date)

Next:
- Write the failing tests, then the implementation, then run the ros tests (memory-capped)
