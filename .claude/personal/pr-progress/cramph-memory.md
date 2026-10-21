## cramph-memory: cut the memory the demos use

Committed on cramph-memory (local only, not pushed; based on simon/cramph 0ce59aa4b):
- 5f1512b93 Mesh.is_textured: marker conversion no longer loads/keeps textured meshes;
  reads the file as a scene so parts' textures are not merged.
- fb4890697 InsideOf.compute_containment_ratio: box check on transformed vertices.
- 88be49110 remove unused EventLogger.current_logger ClassVar.
Bullet demo: peak 1.55 -> 1.31 GB, settled ~1.0 -> 0.8 GB.

Per-demo census (2026-10-21; scripts in session scratchpad, memory_census.py etc.):
- bullet 1.31 GB peak / 0.8 settled; RViz +0.56 GB.
- GARMI 5.5-5.7 GB peak / 4.5 settled; RViz +1.0 GB. MALLOC_MMAP_THRESHOLD_=1048576 -> 1.9 settled.
- tracy sim 0.47 GB; its giskard launch 1.43 GB (interactive_marker 515, giskard 469, rviz 372).
- warehouse crashes at start: UnitreeG1LeftHand lacks tool_frame_goal_for_grasp (pre-existing).

Candidate fixes, not started, awaiting user choice:
1. RayTracer.ray_test (raytracer.py:308) scene.to_mesh() packs all textures: +3.5 GB per
   costmap in GARMI; costmaps.py:395 makes a new RayTracer each time.
2. MJCFParser.__init__ (mjcf.py:113) spec.to_xml() compiles the whole model just for
   tendon coefficients: +1.7 GB (peak 2.3), ~1 GB kept while parser lives.
3. OccupancyCostmap.inflate_obstacles (costmaps.py:424) reshape copies all windows: ~300 MB.
4. Smaller: qp_data._cached_eyes lru_cache, SymbolGraph WrappedInstance growth,
   trimesh caches on collision meshes, eager textured Mesh.from_file in mjcf.py:531.
Note for user: MJCF textured meshes now drawn in shape color in RViz (OBJ has no mtl).
