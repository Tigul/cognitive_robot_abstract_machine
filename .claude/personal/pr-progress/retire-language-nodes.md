Tigul#7 "Run language nodes only through their statechart composite"
(retire-language-nodes -> plan-cramp-second-iter), draft. Plan item
statechart-unification/retire-language-nodes.

Plan (2026-09-25, scope chosen by the user: chart-only path):
1. Un-skip test_exception_try_in_order / test_exception_try_all (they fail today:
   notify() performs children, then the chart runs them again).
2. Remove the notify() overrides of ParallelNode, TryInOrderNode, TryAllNode and
   ExecutesInParallel._perform_parallel.
3. CodeNode -> a leaf that adds a statechart node running its function in a worker
   thread (ThreadedPredicateMonitor idiom). PlanFailure -> node fails; any other
   error is raised out of the tick.
4. Copy each plan node's status from the chart node it added, after execution
   (test_perform_parallel checks that all nodes SUCCEEDED).
5. Run test_language, test_graph_parsing, test_motion_state_chart_building,
   test_giskard_templates, test_underspecified_designator with -n 2.
Out of scope: deleting the LanguageNode classes (retire-plan-layer).

Done: branch and draft PR bootstrapped, manifest and roadmap recorded.
Next: step 1.
