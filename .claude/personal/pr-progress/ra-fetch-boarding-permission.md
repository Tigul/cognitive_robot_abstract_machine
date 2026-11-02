## ra-fetch-boarding-permission: Tiago is sent to a room by a MOVE_TO intent and asks to board
Plan: /home/jdech/.claude/plans/moonlit-chasing-hollerith.md (approved 2026-11-02). Base: ra-fetch-milestone.
Setup done: hri_actions_msgs (apt); std_skills, chatbot_msgs, communication_skills built from
source in ~/workspace/ros (colcon --merge-install).
Steps:
1. Failing tests first (scripted ask server, elevator round stand-in, intents fixture).
2. AskToBoardTask / BoardingPermissionRequest goal / AskToBoardAction (+ exceptions, schema json).
3. NavigateToRoomAction ride hook + NavigateToRoomAskingToBoardAction.
4. RoomRequests (/intents MOVE_TO) + AriseOfficeRoom.spoken_name + demo.
5. Run tests per directory, ORM regen if needed, format docstrings, draft PR (base ra-fetch-milestone).
Status: setup done; starting step 1.
