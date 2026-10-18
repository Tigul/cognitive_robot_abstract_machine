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

## CI fix after main merge 78210f0e9d — committed locally (not pushed)
- pose_facing broke for Pose2D (Point2 position) -> every navigating plan failed
- stale to_pose()/to_position() replaced by upstream properties
- costmap tests -> Pose2D candidates; cramera test gripper= -> end_effector=
- all CI-failing modules + both demos pass locally

## plan_transformations.md notebook fix (2026-10-09) — uncommitted
- notebook compared Point3 (robot pose) with Point2 (Pose2D target); Point2 had
  no generic_vector/euclidean_distance
- euclidean_distance moved to Point base, abstract generic_vector, Point2 implements
  it; 2 new tests in test_point2.py; notebook measures the drive with
  Point2.from_pose; notebook passes via coraplex/scripts/test_notebook_examples.sh

## Review round 2 (2026-10-16) — flat Has*Parameter mixins, committed as 998cc4fd3 (not pushed)
- user decided (after reviews by ichumuh/LucaKro on bundle discoverability): no
  bundles; every mixin declares its own fields only and is named Has...Parameter(s)
  (plain Has<Subject> collides with semdt HasHandle/HasEndEffector). Reasoning
  modules may define their own combined concepts later.
- 8 bundles deleted; GripperStallTolerance split into HasMaxFingerVelocityParameter
  + HasStallToleranceParameters; CartesianVelocityLimit split from HasMovementType;
  tuning mixins no longer inherit each other / grasp detection threshold
- bases ordered so every designator's dataclass field order equals the old MRO
  (verified against HEAD for all 42 designators)
- test_parameter_mixins: new test that no mixin stacks another; bundle asserts moved
- ORM regenerated; test_designator, test_orm, test_actions, test_plan, placing,
  transporting, tool_based, grasp tests all pass

## Capabilities (2026-10-18) — uncommitted
- DesignatorCapability enum (MANIPULATES_OBJECTS, CONTROLS_AN_ARM,
  MOVES_THE_TOOL_CENTER_POINT) in datastructures/enums.py
- DesignatorParameterMixin.capabilities classproperty (empty); mixins add their member
  via cooperative super() (grasp/graspable/handle/tool -> manipulates; arm -> arm;
  goal thresholds/target pose/grasp approach/cartesian limits -> TCP)
- CapabilityTransformation(PlanTransformation[DesignatorNode]) in
  robot_plans/plan_transformations.py: abstract `capability`, matches designators
  whose type has it; no-mixin designators (composites, DetectAction) never match
- tests: test_parameter_mixins (capabilities), test_plan_transformations (mimic
  RecordsObjectManipulation: matches pick-up only; applied to PickUp, Grasping, Reach)
- ORM regenerated; test_plan, test_designator+test_orm, action modules pass

Next:
- commit + push when the user asks; set PR back to draft; reply to + resolve the
  naming threads (mixins.py:42, :181, :465, :442); grasp_candidates.py:86 rename
  revert, :150 LookTarget/TargetPose merge, :102 optional bool, :162 MovementType,
  robot_parts.py:873, semdt mixins.py:781, demo.py:157 kwargs still open
