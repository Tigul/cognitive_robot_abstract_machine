Branch test-world-rollback: replace coraplex mutable/immutable world fixtures with
rollback (World.rollback_to_version) + DOF state restore.

Plan:
1. [done] test/coraplex_test/world_snapshot.py: WorldSnapshot (capture/restore).
2. [done] conftest: pr2_apartment_context, simple_pr2_context, stretch_apartment_context
   replace mutable_/immutable_ fixtures; call sites renamed in 22 files.
3. [done] Local fixtures converted: multiple_robot_simple_apartment_context,
   simple_pr2_holding_milk_context (test_grasp); unused mutable multi-robot fixture removed.
4. [done] test_world_snapshot.py (2 tests, pass).
5. [done] ran every affected module serially: all pass except 4 in test_perception.py
   (test_motion_history_recording.py: pre-existing ImportError cramera.live).
   Cause: World._remove_semantic_annotation uses list.remove (equality = class+root), so
   reverting a second Milk on the same body removes the original and leaves the new,
   detached annotation (_world None) in the world. SDT bug exposed by rollback.
6. [done] User chose: fix on this branch. Failing test
   test_removing_a_semantic_annotation_keeps_an_equal_one (test_world.py), then
   World._remove_semantic_annotation removes by identity. Perception + SDT world/
   modification/annotation modules pass.
7. [running] Missed earlier (truncated grep): test_multi_robot_action_designator.py
   (-> multiple_robot_apartment_context, still restores full_body_controlled; elevator test
   now stops its ElevatorOperator callback) and test_multi_stationary_robot_action_designator.py
   (-> stationary_block_context). Callbacks deliberately not stopped generically in
   WorldSnapshot (internal lazily-registered callbacks). Full grep: no (im)mutable fixtures left.
Next: confirm those two modules pass; commit when asked (two commits). No PR yet.
