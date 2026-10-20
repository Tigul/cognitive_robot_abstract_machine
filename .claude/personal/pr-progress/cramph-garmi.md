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
