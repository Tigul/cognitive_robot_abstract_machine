## Round 2 review of ichumuh#8 by Tigul (2026-10-07/08, 49 comments, head ab618997e)

Plan approved 2026-10-08 (~/.claude/plans/we-did-another-review-buzzing-walrus.md).
Decisions: context always available (nodes read self.context; locations and
factories take the context; trial copies context via copy_onto(world));
"statechart gets context via executor" deferred (future_problems);
new abstract cramph Executor base; Match in language nodes = investigate only.
Steps (one commit each):
1. cramph: drop 2nd rebuild in tick, hold-still into compile, move
   RepetitionsExhausted/PlanCancelled/StatechartHoldsNoPlan/CannotInsertBesideRoot
   to cramph, context/world accessors on StatechartNode  [todo]
2. cramph Executor base; Simulated/Robot executors on it; delete ExecutionType,
   perception source as context extension; demonstration executor_type  [todo]
3. context: PlanRewriting as extension, delete WorldCopyableExtension/for_world,
   StatechartContext.copy_onto, ActionTrial w/o for_trial, locations take context  [todo]
4. actions: no wrapper Sequences (Action adopts self-deciding, reads observation),
   action_body computed, general language-node lookup, TCP check in final_approach,
   upright torso as transformation  [todo]
5. underspecified: merge UnderspecifiedCandidates into node, drop for_step  [todo]
6. delete plan_callbacks (cramera via ExecutorExtension, bridge holds statechart),
   orm/model.py, renames, docs, future_problems entries  [todo]
Then: full suites serially, reply on all threads, push, PR stays draft.

### Previous round (Simon, 2026-10-06): done, pushed ab618997e; 3 threads left
open with questions to Simon. Open questions for user: remove TransitionKind.of /
ThreadPayloadMonitor (tests only)? querying.py undefined names; speed_at_rest.
