## ra-fetch-ros4hri-elevator-zones — PR AICOR-Solutions/cognitive_robot_abstract_machine#1 (base: ra-fetch-milestone, author maltehue)

Addressing Tigul's CHANGES_REQUESTED review (12 threads, head de0706436c).
Plan: /home/jdech/.claude/plans/there-is-a-review-humble-dewdrop.md
User decisions (2026-10-27): social nav = ROS 2 action via Giskard
ActionServerTask + monitor; interface pkg in repo (experiments/ros/
ra_fetch_interfaces, built via symlink into /home/jdech/workspace/ros/src,
colcon --merge-install); task lives in experiments; areas = Regions +
InsideRegion; ElevatorDrive = Gazebo-only InputSynchronizer
(experiments.tiago_arise_gazebo); ElevatorPeople.main fetches live world;
StillProgressing exempts AwaitingExternalResult (giskardpy, ActionServerTask);
keep RobotDemonstration.conclude; fixes as new commits, no force-push.

Local commits (not pushed yet): da6f0581f4 areas as Regions; 519d566ef9
TrackedPersons synchronizer (+TfFrameSynchronizer.untrack); 761eed1274
ElevatorDrive synchronizer; a34495d8d2 robot annotation; 87cd3ddc76
Elevator.doorway_pose + Level.under; 6a005845b2 giskardpy stall exemption;
(amended) BoardElevator action step 6.

Verified: semDT areas/office/ORM/specs/semantic_annotations/worlds (1
pre-existing fail on HEAD too), input sync, giskardpy statechart+orm,
experiments tiago_arise_gazebo + test_social_navigation (8).
NOT verified: experiments tests importing hri_msgs (not installed; user to
`sudo apt install ros-jazzy-hri-msgs`). CI does not run experiments tests.

Next: run hri_msgs tests once installed; push to aicor-solutions; PR body;
reply + resolve threads; draft state per notes (PR is maltehue's).
