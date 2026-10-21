## mjcf-compile-for-tendons: draft PR cram2/cognitive_robot_abstract_machine#790 into main (label bug)

Part of the memory investigation (integration branch cramph-memory). One root cause.
Done: MJCFParser.resolved_document built only when tendons are read (GARMI -1.7 GB).
Based on cram2/main 439a40116; new test fails on main without the fix, affected suites
pass with it (run in a worktree; ORM regeneration fails on main in this env, tests used
--orm-build never). Replaces closed ichumuh fork PR #37 (branch cramph-mjcf-compile-for-tendons).
Next: nothing pending; wait for review. Mark ready only when the user says so.
