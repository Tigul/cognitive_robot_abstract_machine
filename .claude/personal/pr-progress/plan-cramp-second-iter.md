## Merge of cram2/main (0515fcfc9) into plan-cramp-second-iter (2026-10-06)

User asked to resolve the conflicts; chose to PORT upstream features built on
the deleted plan layer (plan transformations #579, execution observers /
visualization #667, cramera package) onto cramph statecharts in this merge.

Approach: per file, keep our architecture (cramph Action / PlanExecutor),
port upstream semantics. Resolver helper: scratchpad resolve.py. Files to
delete at staging time are listed in scratchpad deleted.txt. An IDE runs git
status constantly -> index.lock races; stage everything at the end.

Done (unstaged):
- giskardpy: ErrorSignal -> Scalar; progress monitors (error at last
  progress); daisy/progress/cartesian tests; motion_statechart.py deleted.
- cramph: StateHistoryObserver + try/finally cancel (+ tests).
- segmind: upstream detectors (#670) ported to cramph; event_segmentation.
- coraplex src: grasp candidates (#588), Arm/EndEffector instead of Arms/
  ViewManager, FaceAtAction/FaceAndLookAtAction, transporting with Match
  fields via UnderspecifiedNode.for_step; PlanExecutor: StillProgressing +
  simulation_time_limit + MotionMadeNoProgress/MotionViolatedCollision
  wrapping, candidate limit, shared ActionTrial with catch-up + debug RViz,
  chosen candidate = Attempt(Sequence([action]), Stalled); plan
  transformations as PlanRewriting (ContextExtension) applied after
  expansion + on adopt_chosen_child; PlanCallback(on_compile/on_start/
  on_end) via PlanCallbackDispatcher(StateHistoryObserver); visualization
  attach_plan(executor). Deleted backends/pose_validator/utils (upstream).
- cramera: bridge/visualization/chart_* on cramph Statechart; PlanNodeKind.
- experiments ported.

Next: coraplex demos/docs/examples; tests (coraplex conflicts + new upstream
tests incl. test_transporting/test_placing/test_plan_transformations/
execution observers/visualization; cramera tests); run suites.
