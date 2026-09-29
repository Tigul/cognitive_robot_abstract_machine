Branch test-world-rollback: replace coraplex mutable/immutable world fixtures with
rollback (World.rollback_to_version) + DOF state restore.

Plan:
1. [done] test/coraplex_test/world_snapshot.py: WorldSnapshot (capture/restore).
2. [done] conftest: pr2_apartment_context, simple_pr2_context, stretch_apartment_context
   replace mutable_/immutable_ fixtures; call sites renamed in 22 files.
3. [done] Local fixtures converted: multiple_robot_simple_apartment_context,
   simple_pr2_holding_milk_context (test_grasp); unused mutable multi-robot fixture removed.
4. [done] test_world_snapshot.py (2 tests, pass).
5. [running] run each affected module serially; compare failures to a baseline on HEAD.
Next: triage failures (leaks the rollback misses -> report, no defensive guards),
format docstrings, commit only when asked. No PR opened yet.
