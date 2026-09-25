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

Done (2026-09-25): steps 1-5, committed and pushed as 7e68a6f33. add_node lets a child
through only while its parent is being added (parent_node_index >= _compiled_node_count).
History snapshots are compared by value (np.array_equal; != broke on different lengths).
811 cramph and giskard tests + 56 coraplex tests pass, run serially. PR description updated;
PR is still a draft.
Next: the user reviews. Then reattach-statechart-node, which builds on extend and recompile.
