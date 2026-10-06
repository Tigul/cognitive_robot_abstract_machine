## Review of ichumuh#8 by Simon (2026-10-06, 33 comments, head 48a01efef)

Merge of cram2/main done and pushed (48a01efef). Plan approved by the user:
/home/jdech/.claude/plans/eventual-greeting-patterson.md. Commit straight onto
plan-cramp-second-iter (no new PRs); one commit per step, suites serially.

Decisions: hold-still redesign now; parameters via krrood FieldMetadata
marking non-parameters (StatechartNode.name); future_problems.md committed
at repo root.

Steps:
1. [ ] future_problems.md + skip JSON/ROS2/feedback-publisher tests
2. [ ] cramph: drop LifeCycleChangeLog; visualize -> plotters; ThreadedNode
       base (FunctionCall into cramph); drop CandidateGenerator
3. [ ] actions: create_action_body() replaces _sub_nodes; ParameterMetadata
4. [ ] one StatechartContext + coraplex extensions; SimulatedPlanExecutor /
       RobotPlanExecutor replace PlanExecutor + ExecutionEnvironment;
       statecharts without root; build_statechart; trials into underspecified.py
5. [ ] hold still by decelerating (before_recompile -> at rest), one phase
6. [ ] PerceptionTask -> StatechartNode
Then: push, reply on every thread (leave 4 open with explanations), update
PR description.
