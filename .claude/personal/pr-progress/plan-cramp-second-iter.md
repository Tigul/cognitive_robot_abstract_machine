## Round 2 review of ichumuh#8 by Tigul (2026-10-07/08, 49 comments) - DONE

Pushed to Tigul/plan-cramp-second-iter (755dc70d9): 70c53b54a cramph
failures/tick/accessors, 2f8ddeca0 Executor base + ExecutionType removed,
2e67f85bd context everywhere + bare action bodies, 159c2f635 underspecified
node grounds itself, 755dc70d9 plan_callbacks -> cramph ExecutorExtension,
renames/docs/future_problems. Replies on all 51 (2026-10-09); left open:
orm_example (statechart context from executor, deferred), locations context
model, X|Match typing (ORMatic drops unions), Match in language nodes
(investigated, awaiting choice), MoveTorso keeps Sequence (user), hold-still
in compile (not feasible as is). PR description updated; PR stays draft.
Pre-existing: test_demonstrations fails after cramera_test in one session
(live server port in use) - same on 159c2f635.
User decisions this round: context always available; Executor base in cramph;
Match investigate only; delete alternative_motion_mapping.py but keep robot
mappings; keep MoveTorso Sequence; replace plan_callbacks now.

### Previous round (Simon, 2026-10-06): done, ab618997e; 3 threads open to Simon.
Open questions for user: remove TransitionKind.of / ThreadPayloadMonitor
(tests only)? querying.py undefined names; speed_at_rest.
