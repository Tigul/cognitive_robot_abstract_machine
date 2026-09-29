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
   Results: multi_robot_action 100 passed; multi_stationary 7 passed + 7 DAiSy setup errors,
   pre-existing (daisy_world session fixture: collision SRDF names
   left_gripper_side_cylinder_link, which the DAiSy model lacks; same on old fixtures).
All of the above committed (dac5091c5, 9b25173b5). PR #686 (cram2 upstream) open.
8. [done] Timing branch vs main (24 changed modules, serial, 1 run each): 979s vs 999s (~2%);
   gain mostly test_multi_robot_action_designator (306 vs 321s) and test_grasp.
9. [done] CI failure test_replay_complex_plan_from_db (TypeError: cannot pickle 'generator').
   Cause: krrood InferenceRecorder tags existing instances returned by @symbolic_function
   (ViewManager.get_end_effector_view -> PR2LeftGripper) with _inference_explanation_; that
   explanation holds live query generators, so rebind_world_entities' deepcopy of later actions
   fails. Rollback keeps the same gripper object across tests (old fixture discarded it).
   Repro: test_action_conditions.py then test_ormatic_designator.py.
   User chose fix A+B on this branch: failing tests in test_explanation.py (3), then
   InstantiatedVariable._constructs_its_values_ + HasBoundValue._binds_constructed_instance_
   (A) and InferenceExplanation.__deepcopy__ returns self (B). krrood suite + SDT annotations +
   minimal repro + full gw0 sequence (333 passed; only the known local DAiSy setup errors) pass.
   Uncommitted.
Next: commit + push when asked (PR is upstream cram2 - do not comment/modify PR there).
