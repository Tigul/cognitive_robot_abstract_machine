## localization-raytracer-open3d — PR Tigul/cognitive_robot_abstract_machine#13 (base: localization-odom-connection)

Second PR of the SemDT lidar localization stack (design doc:
https://claude.ai/code/artifact/c8058971-9468-4c9c-a6f6-65c6875262e7).
Stack: semdt-localization <- #12 localization-odom-connection <- #13 this <-
localization-monte-carlo <- localization-ros-node.

Plan: RayTracer.ray_test on open3d RaycastingScene; rebuild only on model change
or a held shape moving; excluded_bodies; trimesh scene kept for camera/rendering.
open3d imported locally in _build_raycasting_scene (user approved, +1.1 s import).

Done (2026-10-19): refactor + 4 new tests, open3d>=0.19.0 in SemDT deps,
RayTracer/PlacedCollisionShape in generate_orm ignore list. Commit e197ef6d09,
pushed, draft PR #13. Pre-existing local failure: robokudo semdt_raytracer AE
tests (plane.py needs open3d 0.20 get_max_threads, env has 0.19).

Next: localization-monte-carlo on top of this branch.

