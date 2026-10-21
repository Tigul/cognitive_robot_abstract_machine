## cramph-memory: cut the memory the demos use (integration branch, not pushed)

Six fixes, each split out into its own branch and draft PR into ichumuh:cramph
(bug label not addable: no permission):
#33 cramph-marker-texture-check, #34 cramph-containment-without-copies,
#35 cramph-release-event-loggers, #36 cramph-raytracer-geometry-only,
#37 cramph-mjcf-compile-for-tendons, #38 cramph-costmap-inflation-in-place.
Effect (all six): GARMI peak 5.69 -> 1.12 GB, settled 4.5 -> 1.0 GB;
bullet peak 1.55 -> 0.85 GB.

Open, not started: warehouse demo crash (UnitreeG1LeftHand.tool_frame_goal_for_grasp
missing); MALLOC_MMAP_THRESHOLD_=1048576 as launcher/env setting; new RayTracer per
costmap (costmaps.py); 515 MB interactive_marker process in the tracy giskard launch;
smaller: qp_data lru_cache, SymbolGraph growth, trimesh caches on collision meshes,
eager textured Mesh.from_file in mjcf.py.
