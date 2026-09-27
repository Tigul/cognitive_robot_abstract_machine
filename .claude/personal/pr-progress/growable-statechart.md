Tigul#8 "Let a running statechart take new nodes, batched in modify()"
(growable-statechart -> plan-cramp-second-iter), draft. Plan item
statechart-unification/growable-statechart.

History: 7e68a6f33 added Statechart.extend / StatechartExecutor.extend. Benchmarked on request:
recompiling costs about a full compile (report https://claude.ai/artifact/GZPafsk73fc6Ng6tpcWtRb);
the user parked the fix as the deferred item statechart-compile-performance.

Review (2026-09-27, Tigul): extend is redundant with add_node / add_nodes; batch like
World.modify_world(). Follow-up from the user: a compiled chart must still accept add_node, and
the outermost block recompiles everything.
Done in cbd07f5f4:
- Statechart.modify() -> StatechartModification (nests; the outermost exit recompiles only if
  _changed_since_compile; a block that raises drops its nodes via _drop_nodes_from -> _renumber)
- add_node on a compiled chart wraps itself in a block; add_nodes is one block; a child of a
  compiled composite is still rejected (_joins_a_compiled_node)
- compile() completes and builds only nodes past _compiled_node_count; a recompile notifies
  RecompileCallbacks, and StatechartExecutor is one (after_recompile runs the extensions again)
- removed extend x2 and StatechartNotCompiledError; test_a_compiled_statechart_rejects_new_nodes
  -> test_a_compiled_statechart_takes_new_nodes (as asked)
- 815 cramph and giskard tests + 56 coraplex tests pass, run serially; ORM regenerated
- PR body updated; both threads replied to and resolved; the review summary answered with a PR
  comment; PR converted back to draft (it had been set to ready)
Next: the user's next review round. After the merge: reattach-statechart-node.
