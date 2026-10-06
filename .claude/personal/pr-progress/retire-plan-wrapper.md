## retire-plan-wrapper (plan item statechart-unification/retire-plan-wrapper)

Goal: delete coraplex `Plan` + `coraplex/plans/factories.py`; plans are cramph
composites built on their own; `PlanExecutor(context).compile(plan)` /
`.execute()` (Giskard-like). `a(...)` stays explicit `UnderspecifiedNode`.
Coraplex repeat layer dropped for `RepeatOnStall` + `CountNodeResets`.

Done (uncommitted):
- PlanExecution -> PlanExecutor + PlanRun (Skipped/Simulated/Robot), PlanNotCompiled;
  tests in test_plan/test_plan_execution.py
- Plan, factories, ContextIsUnavailable, Context.from_world(plan=) deleted
- Action._sub_nodes -> List[StatechartNode]; transporting wraps a(...) explicitly;
  ElevatorNavigation uses PausedUntilTrue directly
- TrainingEnvironment.setup_episode -> TrainingEpisode(plan, context)
- tests/demos/experiments rewritten (libcst codemod in scratchpad + hand fixes);
  persistence stores root node DAO; querying.py split build_context/build_plan
- docs: examples/{plan,language,action_designator,...}.md, doc/source/{plan,
  quickstart,process_modules,troubleshooting}.md

Committed d5b518c60, pushed to origin/retire-plan-wrapper. Suites: coraplex
437 passed/7 skipped; experiments 183 passed, 1 failed (known MoveGripper JSON
round trip in test_real_stretch_demo_process_boundary, from #10).
PR NOT opened: gh token 403 (cram2 lifetime policy for gh pr create, REST
403 too). PR body ready in session scratchpad; user opens it by hand as a
draft into plan-cramp-second-iter, then set pull_request_number in plan.yaml.

Open questions for the user: force_torque_sensor.human_touch_monitoring is dead
and calls plan.root.resume() (no such method) - left untouched.
