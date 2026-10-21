## raytracer-geometry-only: draft PR cram2/cognitive_robot_abstract_machine#789 into main (label bug)

Part of the memory investigation (integration branch cramph-memory). One root cause.
Done: RayTracer.ray_test casts at scene.triangles instead of scene.to_mesh() (no texture packing; GARMI -3.5 GB peak).
Based on cram2/main 439a40116; new test fails on main without the fix, affected suites
pass with it (run in a worktree; ORM regeneration fails on main in this env, tests used
--orm-build never). Replaces closed ichumuh fork PR #36 (branch cramph-raytracer-geometry-only).
Next: nothing pending; wait for review. Mark ready only when the user says so.
