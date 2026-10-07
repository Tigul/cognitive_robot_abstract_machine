## Review of ichumuh#8 by Simon (2026-10-06, 33 comments, head 48a01efef)

All six steps done, committed and pushed to Tigul/plan-cramp-second-iter
(ab618997e): 20d30cc1e park JSON/ROS2 tests (future_problems.md),
5afbc3be1 cramph cleanups, f7b665615 create_action_body + ParameterMetadata,
c6a9e3899 one StatechartContext + Simulated/RobotPlanExecutor (executor
builds context from extensions; one executor per plan), 8fcd72e98 hold still
by decelerating (MotionControl.speed_at_rest = 1e-3, chosen by me),
ab618997e PerceptionTask -> StatechartNode. All suites green (1900 + 759).

Replies posted on all 27 threads (2026-10-07); 24 resolved, 3 left open with
questions to Simon (Action observation order, structure-change rebuild, Match
JSON). PR description written (via REST; `gh pr edit` fails on the classic
Projects deprecation). PR stays draft.
Open questions for user: remove TransitionKind.of / node.py
ThreadPayloadMonitor (tests only)? querying.py has pre-existing undefined
names; speed_at_rest value.
