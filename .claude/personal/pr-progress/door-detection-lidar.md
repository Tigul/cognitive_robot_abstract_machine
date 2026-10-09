## door-detection-lidar — ElevatorNavigation sees open doors with the lidar
Plan: LidarReading.distances_to_segment (sdt) -> PassageClear giskard monitor
(UNKNOWN when no beam crosses the passage, per user) -> ElevatorNavigation uses
PassageClear (doorway window of base width) + drive joint, turns to face doors
after boarding; MissingLidarError for lidarless bases. Door-joint test removed.
User-approved extra: SimulatedLidarSource ignores hits on its own robot's branch
(HSR/PR2/Stretch beams hit their own links); AbstractRobotPart._robot returns None
for parts in no world.
Done: code + unit tests pass (sdt lidar, lidar annotation, giskard monitor).
Next: coraplex -k elevator rerun (first run failed on self-hits + out-of-view
raise); then commit, push to origin fork, draft PR. Note: simulated lidar is
slow per tick (ray_tracer rebuilds scene.to_mesh each call).
