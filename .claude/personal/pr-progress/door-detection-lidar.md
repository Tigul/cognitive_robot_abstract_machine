## door-detection-lidar — ElevatorNavigation sees open doors with the lidar
Plan: LidarReading.distances_to_segment (sdt) -> PassageClear giskard monitor +
PassageOutsideLidarView -> ElevatorNavigation uses PassageClear (doorway window of
base width) + drive joint, turns to face doors after boarding; MissingLidarError
for lidarless bases. Door-joint test removed.
Done: all code + tests written; sdt lidar tests and giskard monitor tests pass.
Next: coraplex elevator tests (-k elevator) running; then commit, push to fork, draft PR.
