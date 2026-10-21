## marker-texture-check: draft PR cram2/cognitive_robot_abstract_machine#786 into main (label bug)

Part of the memory investigation (integration branch cramph-memory). One root cause.
Done: Mesh.is_textured (file read as scene, kept as bool) for FileMeshToRos2Converter; no textured mesh kept for RViz markers.
Based on cram2/main 439a40116; new test fails on main without the fix, affected suites
pass with it (run in a worktree; ORM regeneration fails on main in this env, tests used
--orm-build never). Replaces closed ichumuh fork PR #33 (branch cramph-marker-texture-check).
Next: nothing pending; wait for review. Mark ready only when the user says so.
