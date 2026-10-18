## localization-odom-connection — PR Tigul/cognitive_robot_abstract_machine#12 (base: semdt-localization)

Bottom PR of the SemDT lidar localization stack (design doc:
https://claude.ai/code/artifact/c8058971-9468-4c9c-a6f6-65c6875262e7).
Stack: semdt-localization <- localization-odom-connection <-
localization-raytracer-open3d <- localization-monte-carlo <- localization-ros-node.

Plan: RobotSpecification.localization_connection_type (default FixedConnection,
Connection6DoF opt-in because MuJoCo makes every Connection6DoF a free joint).
Scan stamps + odometry history moved to the ROS-node PR (their only consumer).

Done (2026-10-19): field + 2 tests (attach, ORM round trip; field typed
Type[A | B] because ORMatic skips Type[A] | Type[B]); test_specifications,
test_robots, test_orm green. Commit 2386f455ce, pushed, draft PR #12 opened.

Next: localization-raytracer-open3d on top of this branch.

