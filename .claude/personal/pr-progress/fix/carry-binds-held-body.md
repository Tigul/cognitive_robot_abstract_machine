## fix/carry-binds-held-body: CarryAction takes holding_arms, binds the held body at start
Draft PR: https://github.com/cram2/cognitive_robot_abstract_machine/pull/879 (labels bug-fix, api-break; the repo has no `bug` label). Based on cram2/main, pushed to origin (Tigul) as 5e1fbce5d0.
Done 2026-11-05: KeepHeldBodyUpright (AlignPlanes ending at the tool frame, ForwardKinematicsBinding.change_tip at start), CarryAction.holding_arms, MoveAndPlaceAction.from_standing_position(arm=...), sage_10k demos and tests updated, process test against a real standalone Giskard for the Stretch.
Open for review: a listed arm with nothing below its tool frame does not raise, it keeps the up direction it has at start, because physics runs skip the reattach. Physics therefore does not turn the object upright.
Not run: MuJoCo, real robot, sage_10k demos beyond import. Not ported to ra-fetch-merge-cram2-main, which has the same bug.
Decided not to build: removing object_designator from PlaceAction (user, 2026-11-05: "another adventure"). A prototype worked but is out of scope here.
Next: user discusses the api-break with the coraplex and giskardpy code owners or in the Cramgineers channel before requesting review. Mark ready only when told.
