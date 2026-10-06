## parameter-mixins — merge of cram2 main (2026-10-06)

Plan (user chose "re-port onto upstream"): resolve every conflict to upstream's
API (Arm, GraspCandidate, HasGraspCandidates, composite transport actions) and
re-apply the parameter mixins over upstream's field names; no branch field
renames on top.

Done:
- conflicts taken from upstream; pose_validator + its test deleted as upstream did
- mixins.py rebuilt: branch leaves/bundles retyped (UsedArm->Arm, UsedGrasp,
  UsedGripper, ObjectActedOn->object_designator, TargetLookedAt->target), upstream
  HasApproachesGraspPoses/GraspPoseSequence kept; dropped mixins whose classes
  upstream removed (JointStatesKept, NavigationParameters, MobileManipulation...,
  PoseSequenceReversed, UsedGraspDescription, UsedTechnique, ToolUsage...)
- IsGraspable/GraspableObject dropped (upstream HasGraspCandidates / Parcel)
- actions/motions ported; HasGraspChoice -> GraspParameters
- codemod: positional ctor args -> keywords (293 calls, src/tests/demos/docs)
- test_parameter_mixins.py rewritten for the new API (10 pass)

Next:
- run affected coraplex test modules serially, fix failures
- run scripts/format_docstrings.py on changed files
- conclude merge commit (only when user asks)
