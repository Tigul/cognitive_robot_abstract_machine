Branch statechart-plan-parity (plan item statechart-plan-parity), off plan-cramp-second-iter.
Pure cramph: Statechart.layers + Statechart.visualize().

Done:
- Tests: test/cramph_test/test_statechart/test_statechart_introspection.py (5, written failing first).
- cramph/statechart.py: layers (BFS over children from top_level_nodes), visualize() /
  _create_visualizer over a parent->child graph (_parent_child_graph), labels unique_name,
  colour life cycle state, details life cycle/observation/run ticks; all 4 backends mapped.
- cramph suite: 584 passed, 1 skipped. Committed 44caa51cb, pushed to origin.
- Env: cramph was not installed in /home/jdech/envs/cram; installed with pip -e --no-deps.
Decisions: drawn structure is fixed at visualize() time (state live); ticks not seconds in details.
Next: gh auth login -> open draft PR into plan-cramp-second-iter, then
  plan_item_bootstrap.py open --pull-request-number N, /plan-dashboard statechart-unification.
