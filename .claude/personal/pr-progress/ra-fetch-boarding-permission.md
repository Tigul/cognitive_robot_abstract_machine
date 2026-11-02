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
Next: try on Gazebo when the user wants; mark ready only when told.
