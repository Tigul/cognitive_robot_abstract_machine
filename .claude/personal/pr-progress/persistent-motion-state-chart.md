# persistent-motion-state-chart

No pull request yet: creating one failed on the gh token (cram2 org lifetime
policy / 403); the user said to go ahead without it.

## Plan
1. Failing tests first: one chart per plan across a boundary; first segment's
   state survives extension; ReAttachNode mid-plan; underspecified after motion;
   real path sends the grown chart (mocked wrapper).
2. One chart + executor per plan execution, grown with Statechart.modify().
3. Segment ends when its root succeeds; one EndMotion at plan end.
4. ReAttachNode -> cramph MoveBranch node, no longer a boundary.
5. UnderspecifiedExecutable extends the same chart.
6. Real-robot path: settle whether giskard accepts a grown chart; ask user if not.

## Done
- Branch created off plan-cramp-second-iter (d8cd3dcc4), bootstrap commit pushed.

## Next
- Read Plan.perform / executables flow, write failing tests.
