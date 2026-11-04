## fix/carry-binds-held-body: CarryAction takes holding_arms, binds the held body at start
Based on cram2/main, pushed to origin (Tigul) as 5e1fbce5d0. The PR is not open yet: opening it on cram2 was denied in auto mode, so the user opens it as a draft with labels bug-fix and api-break (the repo has no `bug` label); description in the session scratchpad and in the final message.
Done 2026-11-05: KeepHeldBodyUpright (AlignPlanes ending at the tool frame, ForwardKinematicsBinding.change_tip at start), CarryAction.holding_arms, MoveAndPlaceAction.from_standing_position(arm=...), sage_10k demos and tests updated, process test against a real standalone Giskard for the Stretch.
Open for review: a listed arm with nothing below its tool frame does not raise, it keeps the up direction it has at start, because physics runs skip the reattach. Physics therefore does not turn the object upright.
Not run: MuJoCo, real robot, sage_10k demos beyond import. Not ported to ra-fetch-merge-cram2-main, which has the same bug.
Next: user opens the PR and discusses the api-break with the coraplex, cramph and giskardpy code owners or in the Cramgineers channel. Mark ready only when told.
