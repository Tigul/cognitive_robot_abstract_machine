Branch `new-visualization` (base: simon/cramph). Plan: turn coraplex visualizations
into ExecutorExtensions, one renderer run per executor (start after_compile, stop
after_run), cramera LiveVisualization collapsed into one extension. Full plan:
~/.claude/plans/the-visualization-in-coraplex-src-corapl-lively-blanket.md
- [x] tests for new behaviour (test_optional_visualization, test_visualization_lifecycle)
- [x] rewrite coraplex/visualization.py (WorldVisualization extension, Rviz, Rerun)
- [x] cramera LiveVisualization + runner (session/pause removed)
- [x] callers: demonstrations, underspecified trial, testing.selected_visualizations, demos
- [x] cramera tests ported; coraplex vis tests 49 pass, cramera 1059 pass
- [x] test_plan 199 pass
- [x] committed 529db839a, draft PR ichumuh#21 (base cramph)
- [x] 95a14b50c: RViz thin wrapper using executor RosContextExtension node; demos open RobotDemonstrationRosSession; PR body updated. Next: review / CI
- [x] review (ichumuh, CHANGES_REQUESTED) addressed in 10f789f4f after merging simon/cramph: RvizVisualization.ros_node field, trials get no node, trial tests ported, cramera-live inspection restored (LiveVisualization keeps server/recording until stop, demo shares one viz, runner sigwait), RViz-dark accepted+documented; PR body updated, reply comment posted, PR still draft. Next: CI / re-review
