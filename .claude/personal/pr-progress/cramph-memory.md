## cramph-memory: cut the memory the demos use

Local commits on cramph-memory (not pushed), rebased onto simon/cramph f8844811f:
1. 2ea31a389 Mesh.is_textured for RViz markers (no textured mesh kept).
2. 73eb248b4 InsideOf containment without mesh copies.
3. 71cc0defa remove EventLogger.current_logger.
4. 05b32edd5 RayTracer.ray_test casts at scene.triangles (no texture packing; GARMI -3.5 GB peak).
5. ec9a0cf0a MJCFParser.resolved_document only built for tendons (GARMI -1.7 GB).
6. 10f0dff59 OccupancyCostmap.inflate_obstacles via sliding_window_view (-300 MB/costmap).
All with failing-test-first; affected suites green (multi_sim and tiago_mujoco skip locally).

Next: report final benchmarks; ask whether to push / open draft PRs (one per root cause;
note branch is based on cramph, not main).
Open, not started: warehouse demo crash (UnitreeG1LeftHand.tool_frame_goal_for_grasp
missing); glibc MALLOC_MMAP_THRESHOLD_ (GARMI settled 4.5 -> 1.9 GB) as a launcher/env
setting; smaller items (qp_data lru_cache, SymbolGraph growth, trimesh caches,
eager textured Mesh.from_file in mjcf.py, new RayTracer per costmap, 515 MB
interactive_marker process in tracy launch).
