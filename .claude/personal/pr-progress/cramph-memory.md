## cramph-memory: cut the memory the demos use (integration branch, not pushed)

Six fixes, each moved to its own main-based branch and draft PR on cram2 (label bug):
#786 marker-texture-check, #787 containment-without-copies, #788 release-event-loggers,
#789 raytracer-geometry-only, #790 mjcf-compile-for-tendons, #791 costmap-inflation-in-place.
The earlier ichumuh fork PRs #33-#38 (cramph-* branches) are closed with a pointer.
Effect (all six, on cramph): GARMI peak 5.69 -> 1.12 GB, settled 4.5 -> 1.0 GB;
bullet peak 1.55 -> 0.85 GB.

Open, not started: warehouse demo crash (UnitreeG1LeftHand.tool_frame_goal_for_grasp
missing); MALLOC_MMAP_THRESHOLD_=1048576 as launcher/env setting; new RayTracer per
costmap (costmaps.py); 515 MB interactive_marker process in the tracy giskard launch;
smaller: qp_data lru_cache, SymbolGraph growth, trimesh caches on collision meshes,
eager textured Mesh.from_file in mjcf.py. ORM regeneration fails on cram2/main in this
env (CouldNotResolveType MetaData for WorldUpdateReferencesUnknownEntityError).
