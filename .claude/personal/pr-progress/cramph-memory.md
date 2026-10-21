## cramph-memory: cut the memory the bullet world demo uses

Plan (user chose: per-Mesh texture flag, world-AABB box check, drop current_logger):
1. Marker conversion no longer loads/keeps textured visual meshes (Mesh.is_textured).
2. InsideOf.compute_containment_ratio stops copying meshes (box check on transformed vertices).
3. Remove unused EventLogger.current_logger ClassVar (kept the first run's worlds alive).

Done (uncommitted, on cramph-memory, fast-forwarded to simon/cramph 0ce59aa4b):
- Failing tests first, then fixes 1-3; new tests pass. Suites green: sdt test_predicates,
  test_ros2_msg_converter, test_geometry, test_visualization_marker,
  test_collision_viz_marker; segmind_test; coraplex test_memory_leak,
  test_visualization_lifecycle, test_optional_visualization.
- format_docstrings run (black reflowed event_logger.py).

Next: re-benchmark demo with real fixes; await user on commit / split into one PR per
root cause (personal notes: one root cause per bug PR, `bug` label, draft).
Open smaller items (not started): qp_data._cached_eyes lru_cache on method, SymbolGraph
WrappedInstance growth, shared mesh loads across Mesh instances.
