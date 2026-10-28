## fix/garmi-demo-new-description — PR cram2/cognitive_robot_abstract_machine#815 (draft, bug, base main)

Fixes the GARMI demo CI failure on main (Examples and Demos) after
garmi_description (fingertip links, TCP 129.5 mm) and iai_garmi_apartment (map
frame, remeasured drawers, handles pointing out of drawers) changed upstream.
Plan: /home/jdech/.claude/plans/the-ci-here-on-cheeky-otter.md
User decisions (2026-10-28): backport cramph's four-sided Handle grasps;
ContainerAction takes `grasp` (not `handle`) + from_handle(handle, arm,
standing_position); transformation ranks with IsAmongTheClosestGraspsTo(n=1).

Done (pushed 2026-10-28): f2cc0cf131 garmi.py/srdf/demo backport from cramph;
5a09134df3 handle grasps + ContainerAction.grasp, callers (tests, docs,
sage10k) use GraspCandidate.from_body_origin. Verified locally: demo passes
(fails on main), semDT grasp tests, test_robots, test_collision_checking,
coraplex_test full (504 + designator 219).

Merged cram2/main (2026-10-28, 3bbdc3bbb3): one conflict, container.py import
(upstream typing cleanup dropped Dict; kept Self). Re-verified demo + container/
transporting/transformations/grasp tests + designator open/close. PR set back
to draft after push (someone had marked it ready).

Next: wait for CI / review; nothing outstanding.
