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

- affected test modules run serially: all pass except DAiSy fixture errors
  (environment: daisy.py collision SRDF names a link missing from the installed
  URDF; not touched by either branch); ORM errors fixed by regenerate_all_orm.py
- formatted; merge committed locally as 41579d52e8 (not pushed)

Next:
- push when the user asks
