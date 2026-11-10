## tracy-gripper-motion-replacement: Robotiq grippers of the real Tracy and Tiago move through their action servers
Draft PR: https://github.com/cram2/cognitive_robot_abstract_machine/pull/919 (labels new-feature, api-break). Merged cram2/main in, pushed to origin (Tigul) as 874b03bcc7.
Done 2026-11-10: HasGripperActionServers base (GriplinkGripper, HasGripperCommandServer), Tracy and Tiago grippers get command topics, RobotiqGripperActionServerTask moved to ros2_nodes/robotiq.py with moving_gripper_to, DriveRobotiqGrippersThroughTheirActionServers first in RealRobotPlanExecutor defaults (RealTracyPlanExecutor dropped).
Open: Tracy and Tiago topics are placeholders; target_position read as width in metres to confirm with the driver; target_velocity still documented in rad/s; set_up waits for the server without a timeout.
Local failures: 3 tests fail only because the local env lacks causallearn (new upstream dependency).
Not run: real Tracy or Tiago drivers.
Next: user discusses the api-break with the coraplex, giskardpy and semantic_digital_twin code owners or in the Cramgineers channel before requesting review. Mark ready only when told.
