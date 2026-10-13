Branch `cramph-statement-node` (off `simon/cramph`): plan language nodes accept a
krrood `Match` directly (future_problems.md, design 1, chosen 2026-10-13).
Plan: ~/.claude/plans/the-future-problems-md-sketches-a-parsed-iverson.md

Plan:
1. cramph: `StatementNode(CompositeNodeChoosingItsChild)` holding `statement` and the
   candidate iteration (state must outlive context cleanup, which precedes the
   executor's candidate-limit check); `ChildChooser.adopted_child` hook;
   `CramLanguageNode` wraps a `Match` in `__post_init__`/`_insert_at`/`replace`.
2. coraplex: remove `UnderspecifiedNode`; chooser grounds `StatementNode`s (trial,
   Attempt, PlanRewriting via hook); executor/transformations/training env use
   `StatementNode`; `ActionOfSteps._as_node` removed.
3. Migrate tests, experiments, docs; drop future_problems.md section.

Done: nothing yet.
Next: cramph tests + StatementNode.
