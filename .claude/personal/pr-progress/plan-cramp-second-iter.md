## Review of ichumuh#8 by Simon (2026-10-06, 33 comments, head 48a01efef)

All six steps done, committed and pushed to Tigul/plan-cramp-second-iter
(ab618997e): 20d30cc1e park JSON/ROS2 tests (future_problems.md),
5afbc3be1 cramph cleanups, f7b665615 create_action_body + ParameterMetadata,
c6a9e3899 one StatechartContext + Simulated/RobotPlanExecutor (executor
builds context from extensions; one executor per plan), 8fcd72e98 hold still
by decelerating (MotionControl.speed_at_rest = 1e-3, chosen by me),
ab618997e PerceptionTask -> StatechartNode. All suites green (1900 + 759).

Outstanding (gh token gets 403 on ichumuh): replies not posted, threads not
resolved, PR description not updated. Replies ready in session scratchpad
review_replies.md; post_replies.py posts + resolves once a token has access.
Open questions for user: remove TransitionKind.of / node.py
ThreadPayloadMonitor (tests only)? querying.py has pre-existing undefined
names; speed_at_rest value.
