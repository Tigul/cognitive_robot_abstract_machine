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

## Review round 1 (2026-10-09) — committed locally as b3a2c5c5c6 (not pushed)
- MoveJointsMotion: removed unused alignment fields + LinkAlignmentApplied
- DesignatorParameterMixin documented
- mixins renamed to ...Parameter / ...Parameters (e.g. UsedArm->ArmParameter,
  UsedGrasp->GraspCandidateParameter, ObjectActedOn->GraspableObjectParameter,
  HasApproachesGraspPoses->GraspApproachParameters); all mixins eq=False
- ORM regenerated; affected tests pass (DAiSy fixture errors = xacro ur_type env issue)

Next:
- push when the user asks; then reply to + resolve the 4 review threads
  (4227921325/4227928570 alignment, 4227930644 doc, 4228314292 naming), and
  keep the PR a draft
