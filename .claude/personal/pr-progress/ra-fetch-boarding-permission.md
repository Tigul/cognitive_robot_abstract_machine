## ra-fetch-boarding-permission: Tiago is sent to a room by a MOVE_TO intent and asks to board
Plan: /home/jdech/.claude/plans/moonlit-chasing-hollerith.md (approved 2026-11-02). Base: ra-fetch-milestone.
Setup done: hri_actions_msgs (apt); std_skills, chatbot_msgs, communication_skills built from
source in ~/workspace/ros (colcon --merge-install).
Done (uncommitted, in working tree): ActionServerTask reset fix + tests; Elevator.humans_in_cabin/
humans_waiting_on; AriseOfficeRoom.spoken_name + find_room_by_spoken_name; NavigateToRoomAction._ride_elevator;
experiments/tiago_room_request_demo (in-chart AskToBoardTask, BoardingPermissionRequest, RoomRequests, demo);
tests pass in kinematic simulation.
BLOCKED on a user decision: in REAL runs charts execute on the Giskard server (GiskardExecutable._execute_real)
and ControlLoop aborts on any world model change (WorldModelModifiedDuringMotionError). TrackedPersons adds/removes
Humans with published model changes, so a person appearing during a motion aborts it. Proposed: move the ask/wait
out of the chart into an ExecutionBoundaryNode run client-side, and keep tracked humans local
(modify_world(publish_changes=False)) - needs checking state sync of unpublished DOFs.
Open cleanups: docstrings on test helpers and RoomRequests._hear, remove BoardingPermissionRequest.action_name,
unused arise_office params in tests.
Next: get the decision, then finish, commit, draft PR (base ra-fetch-milestone).
