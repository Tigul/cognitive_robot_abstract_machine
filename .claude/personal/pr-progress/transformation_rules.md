PR cram2#579 "Plan Transformations".

Plan (2026-09-24): fix the failing `test_each_lib (coraplex)` CI job.
- Cause: 6 teardown `LeakedWorldsError`s (31 > 30 worlds per xdist worker), with no
  assertion failures. The PR's new test
  `test_the_opening_joins_the_sequence_an_underspecified_pick_up_runs` added 3 leaked
  worlds on top of 28 that already leak.
- Root cause (krrood, also on main): `SymbolicExpression._evaluate_` set the EQL
  evaluation context var and yielded while it was still set, so a suspended query
  iterator (underspecified grounding / locations) left the context set in the
  thread, pinning query -> locations -> copied Context -> worlds. It also leaked
  into later unrelated evaluations.
- The user chose to fix it on this PR (not a separate bug PR).

Done:
- Failing tests added in test/krrood_test/test_eql/test_core/test_evaluation_context_lifecycle.py.
- Fix: `EvaluationContext.iterate_as_current` sets the var only while results
  advance; `_evaluate_` split into `_evaluate_` + `_evaluate_within_`.
- Lifecycle tests pass. The PR test now leaves 0 worlds behind.
- The 12 failures / 50 errors in the local krrood run (`TypeError __init__ 11 args`)
  also happen without the fix, so they come from the local env.

Next: finish the full coraplex run locally, then commit, push, and set the PR to
draft; check CI.
