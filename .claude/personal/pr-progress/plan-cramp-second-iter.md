## Round 2 review of ichumuh#8 by Tigul (2026-10-07/08, 49 comments, head ab618997e)

Plan approved 2026-10-08 (~/.claude/plans/we-did-another-review-buzzing-walrus.md).
Decisions: context always available (nodes read self.context; locations and
factories take the context; trial copies context via copy_onto(world));
"statechart gets context via executor" deferred (future_problems);
new abstract cramph Executor base; Match in language nodes = investigate only.
Steps (one commit each):
1. cramph: drop 2nd rebuild in tick, hold-still into compile, move
   RepetitionsExhausted/PlanCancelled/StatechartHoldsNoPlan/CannotInsertBesideRoot
   to cramph, context/world accessors on StatechartNode  [done 70c53b54a; hold-still-in-compile NOT done: deferred compile would tick stale state arrays -> reply only]
2. cramph Executor base; Simulated/Robot executors on it; delete ExecutionType,
   PerceptionSource enum on PerceptionTask + ExecutionMode.simulated; executor_type;
   alternative_motion_mapping.py deleted, robot mappings kept (user)  [done 5a78516a6]
3. context: PlanRewriting as extension, delete WorldCopyableExtension/for_world,
   trial rebinds extensions (rebind_world_entities), ActionTrial w/o for_trial,
   locations/factories take context  [done 2e67f85bd]
4. actions: no wrapper Sequences (Action adopts self-deciding, reads observation),
   action_body computed, general language-node lookup, TCP check in final_approach,
   upright torso as transformation; MoveTorso keeps Sequence (user)  [done 2e67f85bd]
5. underspecified: merge UnderspecifiedCandidates into node, drop for_step  [todo]
6. delete plan_callbacks (cramera via ExecutorExtension, bridge holds statechart),
   orm/model.py, renames, docs, future_problems entries  [todo]
Then: full suites serially, reply on all threads, push, PR stays draft.

### Previous round (Simon, 2026-10-06): done, pushed ab618997e; 3 threads left
open with questions to Simon. Open questions for user: remove TransitionKind.of /
ThreadPayloadMonitor (tests only)? querying.py undefined names; speed_at_rest.
