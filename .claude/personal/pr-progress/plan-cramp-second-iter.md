## Review of ichumuh#8 by Simon (2026-10-06, 33 comments, head 48a01efef)

Merge of cram2/main done and pushed (48a01efef). Plan approved by the user:
/home/jdech/.claude/plans/eventual-greeting-patterson.md. Commit straight onto
plan-cramp-second-iter (no new PRs); one commit per step, suites serially.

Decisions: hold-still redesign now; parameters via krrood FieldMetadata
marking non-parameters (StatechartNode.name); future_problems.md committed
at repo root.

Steps:
1. [x] future_problems.md + skip JSON/ROS2/feedback-publisher tests (20d30cc1e)
2. [x] cramph: drop LifeCycleChangeLog; visualize -> plotters; ThreadedNode
       base (FunctionCall into cramph); drop CandidateGenerator (5afbc3be1)
3. [x] actions: create_action_body() replaces _sub_nodes; ParameterMetadata (f7b665615)
4. [~] one StatechartContext + coraplex extensions; SimulatedPlanExecutor /
       RobotPlanExecutor replace PlanExecutor + ExecutionEnvironment;
       statecharts without root; build_statechart; trials into underspecified.py
5. [ ] hold still by decelerating (before_recompile -> at rest), one phase
6. [ ] PerceptionTask -> StatechartNode
Step 4 (user chose: executor builds the context from world + context
extensions; StatechartContext is single-use, so one executor per plan):
coraplex/plans/{context_extensions,executors}.py; Context, PlanExecutor/
PlanRun, ExecutionEnvironment, plan_execution.py deleted; trials+chooser in
underspecified.py (ActionTrial asks executor.for_trial(world)); locations
take robot+seed; evaluate_conditions + alternative_motion_mappings dropped;
tests via test/plan_running.py helpers (robot_extensions, run_plan,
simulated_executor, statechart_of, with_grounding...). Ported tests, demos,
experiments, docs. Final full-suite run pending, then commit.
Open questions for user: TransitionKind.of / node.py ThreadPayloadMonitor
only used by tests; querying.py undefined names pre-existing.
Then: push, reply on every thread (leave 4 open with explanations), update
PR description.
