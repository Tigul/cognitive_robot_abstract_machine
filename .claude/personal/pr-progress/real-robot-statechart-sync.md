Branch `real-robot-statechart-sync` (from `cramph-review-fixes`, stacks on ichumuh#16).
Goal: coraplex mirrors Giskard's statechart states live on the real robot.
Plan: /home/jonas/.claude/plans/i-did-a-review-sleepy-engelbart.md
1. Split ExecutorExtension / TickingExecutorExtension; lift extensions into Executor
2. RealRobotPlanExecutor runs after_compile/after_run
3. Feedback carries tick_count
4. StatechartStateUpdate applied on every feedback + result
5. Aborted goals: deserialize error with world kwargs, apply final states before raising
Done: branch created. Next: step 1.
