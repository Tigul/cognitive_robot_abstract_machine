Branch `real-robot-statechart-sync` (from `cramph-review-fixes`, stacks on ichumuh#16).
Goal: coraplex mirrors Giskard's statechart states live on the real robot.
Plan: /home/jonas/.claude/plans/i-did-a-review-sleepy-engelbart.md
1. Split ExecutorExtension / TickingExecutorExtension; lift extensions into Executor
2. RealRobotPlanExecutor runs after_compile/after_run
3. Feedback carries tick_count
4. StatechartStateUpdate applied on every feedback + result
5. Aborted goals: deserialize error with world kwargs, apply final states before raising
Done (uncommitted): steps 1-5 implemented with tests. Passing: test/cramph_test,
test_state_updates.py, test_abort_exceptions.py, test_motion_server.py, coraplex visualization tests,
test/giskardpy_test/test_ros2_stuff (230 passed; 4 DAiSy errors = xacro ur_type env issue).
Next: run giskardpy test_motion_statechart, coraplex test_plan, cramera, segmind, stretch demo test;
commit, push to origin, draft PR into ichumuh:cramph (stacked on #16).
