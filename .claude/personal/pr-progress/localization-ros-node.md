## localization-ros-node — PR Tigul/cognitive_robot_abstract_machine#15 (base: localization-monte-carlo)

Fourth PR of the SemDT lidar localization stack (design doc:
https://claude.ai/code/artifact/c8058971-9468-4c9c-a6f6-65c6875262e7).
Stack: semdt-localization <- #12 <- #13 <- #14 <- #15 this.

Plan: LidarReading.stamp; MotionModel generic over drive type + for_drive_type;
MonteCarloLocalization.restart_from; adapters/ros/localization.LocalizationNode
(scan topic from lidar.topic_name, odom->base from tf at scan stamp, publishes
root->odom on tf stamped +transform_tolerance, /initialpose restart).
User decisions (2026-10-19): odometry via tf lookup, not an own history;
tf only, the node does not write its twin.

Done (2026-10-19): commit e86a7d3ac4, pushed, draft PR #15. ORM regenerated
(LidarReading.stamp column; LocalizationNode ignored). Suites green:
test_localization, test_sensors, test_raytracer, test_ros, test_orm,
test_specifications, test_robots.

Commit bf5d0cfd11 (2026-10-19): executable semantic-digital-twin-localization
(LocalizationCommandLine.run/start -> LocalizationProcess): fetch twin,
WorldSynchronizer, pick robot (--robot), drive + base lidar, region from
environment collision, start at twin base pose (--search-whole-region).
User approved StretchBaseLidar.topic_name = RobotTopic.SCAN ("scan") and the
change to the unknown-topic test. PR body + design doc (rev 41) updated.

2026-10-19: merged main (b29c37fbfd) up the stack, all five branches pushed;
PRs #12-15 still drafts with only their own files.

Commit 5cd5353e4c (2026-10-19): GarmiBaseLidar on GarmiMobileBase (merged scan
r100_0603/sensors/lidar2d_merged/scan, frame base_link, -pi..pi, 0.004363 rad,
0.1-25 m, from a real message the user pasted); node + CLI take tf frames
(CoordinateFrame REP 105 defaults, --map/--odom/--base-frame) instead of twin
body names; robot bodies via get_kinematic_structure_entities_of_branch
(World.compute_descendent_child_kinematic_structure_entities hangs on GARMI:
extends the memoized list while iterating it; not fixed, reported to user).
All suites green; PR body updated.

Next: milestone 5 on GARMI. Unverified: odom tf frame names on GARMI
(defaults odom/base_link); merged scan's ranges length should be 1441.
