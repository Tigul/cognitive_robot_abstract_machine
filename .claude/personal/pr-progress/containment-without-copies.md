## containment-without-copies: draft PR cram2/cognitive_robot_abstract_machine#787 into main (label bug)

Part of the memory investigation (integration branch cramph-memory). One root cause.
Done: InsideOf.compute_containment_ratio: world-AABB min/max check on transformed vertices, no mesh copies.
Based on cram2/main 439a40116; new test fails on main without the fix, affected suites
pass with it (run in a worktree; ORM regeneration fails on main in this env, tests used
--orm-build never). Replaces closed ichumuh fork PR #34 (branch cramph-containment-without-copies).
Next: nothing pending; wait for review. Mark ready only when the user says so.
