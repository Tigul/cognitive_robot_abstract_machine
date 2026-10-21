## release-event-loggers: draft PR cram2/cognitive_robot_abstract_machine#788 into main (label bug)

Part of the memory investigation (integration branch cramph-memory). One root cause.
Done: Removed unused EventLogger.current_logger ClassVar that kept the first run's worlds alive.
Based on cram2/main 439a40116; new test fails on main without the fix, affected suites
pass with it (run in a worktree; ORM regeneration fails on main in this env, tests used
--orm-build never). Replaces closed ichumuh fork PR #35 (branch cramph-release-event-loggers).
Next: nothing pending; wait for review. Mark ready only when the user says so.
