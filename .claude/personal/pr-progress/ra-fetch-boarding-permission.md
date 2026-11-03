## ra-fetch-boarding-permission: Tiago is sent to a room by a MOVE_TO intent and asks to board
Plan: /home/jdech/.claude/plans/moonlit-chasing-hollerith.md (approved 2026-11-02), then changed with the user's OK:
asking runs in the demo process between motions (ExecutionBoundaryNode), not in the chart; tracked people stay out of the world.
Done 2026-11-02, commit fb37e94a7a, draft PR AICOR-Solutions#5 (base ra-fetch-milestone):
experiments/tiago_room_request_demo (RoomRequests, PersonsAtElevator, AskToBoardAction, demo),
NavigateToRoomAction._ride_elevator, spoken room names. Kinematic simulation tests pass.
Setup: hri_actions_msgs (apt); std_skills, chatbot_msgs, communication_skills built from source in ~/workspace/ros.
Known: ra_fetch_elevator_people 2 failures ('x' field must be float), module unchanged here. Not run on Gazebo or the real robot.
Fallback the user named for further sync issues: consider merging cram2/main (1268 commits behind, 18 conflicting
files). At 1e92e8dfd3 the Giskard ControlLoop still raises on external model changes, so a merge alone won't fix that path.
Not in this PR: ActionServerTask doesn't clear _result on reset (a retried task reuses its old answer); offered to the user as a separate PR.
2026-11-03: the user dropped IRII social navigation for the milestone demo; everything is to run on PathPlanningNavigateAction.
Probed (no code changed), so boarding still ignores people: paths are computed at plan start; the cabin floor blocks the planner
(needs obstacle_height_clearance > 0.021); 0.6 m wide people block the cabin centre; no path gives None and _path then fails; no reaction during a leg.
Next: agree on boarding with run-time planned paths (people as planner obstacles, kept out of the world), then build it; try on Gazebo; mark ready only when told.
