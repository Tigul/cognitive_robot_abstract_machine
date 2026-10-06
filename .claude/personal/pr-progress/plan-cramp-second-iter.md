## Merge of cram2/main (0515fcfc9) into plan-cramp-second-iter (2026-10-06)

User asked to resolve the conflicts; chose to PORT upstream features built on
the deleted plan layer (plan transformations #579, execution observers /
visualization #667, cramera package) onto cramph statecharts in this merge.

Approach: per file, keep our architecture (cramph Action / PlanExecutor),
port upstream semantics. Resolver helper: scratchpad resolve.py. Files to
delete at staging time are listed in scratchpad deleted.txt. An IDE runs git
status constantly -> index.lock races; stage everything at the end.

Done (unstaged):
- giskardpy: ErrorSignal -> Scalar; progress monitors ported (error at last
  progress); daisy/progress/cartesian tests ported; motion_statechart.py deleted.
- cramph: StateHistoryObserver + try/finally cancel; tests in
  test/cramph_test/test_statechart/test_state_history_observers.py.
- segmind: upstream detectors (#670) ported to cramph; event_segmentation uses
  StatechartExecutor + EpisodeSegmentation; new segmind tests ported.
- scripts/sync_version.py (theirs), version test (+cramera, cramph).

Next: coraplex (#588 grasp candidates, Arm instead of Arms, no ViewManager,
locations rewrite, ticks_per_motion -> StillProgressing), then plan
transformations / visualization / cramera on statecharts, experiments, docs,
tests; then run cramph, giskardpy statechart, segmind, coraplex suites.
