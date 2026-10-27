## ra-fetch-ros4hri-elevator-zones — PR AICOR-Solutions/cognitive_robot_abstract_machine#1 (base: ra-fetch-milestone, author maltehue)

Addressing Tigul's CHANGES_REQUESTED review (12 threads, head de0706436c).
Plan: /home/jdech/.claude/plans/there-is-a-review-humble-dewdrop.md
User decisions (2026-10-27): social nav = ROS 2 action via Giskard
ActionServerTask + monitor (new interface package OK); elevator areas =
Regions + InsideRegion; ElevatorDrive = Gazebo-only InputSynchronizer; fixes as
new commits on top (no force-push).

Steps: 1 areas as Regions; 2 TrackedPersons owns Humans, InputSynchronizer via
TfFrameSynchronizer (+untrack); 3 ElevatorDrive synchronizer, drop from
ElevatorPeople; 4 robot annotation instead of frame string; 5 drop
ElevatorDescription/PersonNearElevator copies; 6 BoardElevator action +
Giskard task + coraplex action; 7 docstrings, PR body, thread replies.
Open (ask): interface package location/CI; task in giskardpy vs experiments;
remove RobotDemonstration.conclude?

Done: nothing yet. Next: step 1.
