Tigul#8 "Let a running statechart take new nodes" (growable-statechart ->
plan-cramp-second-iter), draft. Plan item statechart-unification/growable-statechart,
split out of persistent-motion-state-chart on 2026-09-25.

Plan:
1. Failing tests in test/cramph_test/test_statechart (new module): extend a compiled
   chart with a node that starts after an old one succeeded; the old nodes' states and
   history survive; the tick count is not reset; extend on an uncompiled chart raises.
2. Statechart.extend(nodes): register and expand; per-node compile steps for the new
   nodes only; rebuild edges and CompiledTick. add_node keeps raising after compile.
3. StatechartExecutor.extend(nodes): Statechart.extend plus every extension's after_compile.
4. A giskardpy test that a motion added by extend actually moves (MotionControl QP rebuild).
5. Run the cramph_test statechart tests and the giskardpy motion statechart tests serially
   (no -n).

Done: branch and draft PR bootstrapped, manifest and roadmap recorded.
Next: step 1.
