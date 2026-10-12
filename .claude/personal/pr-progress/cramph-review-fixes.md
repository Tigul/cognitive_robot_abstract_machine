Branch `cramph-review-fixes` (from `simon/cramph`), draft PR into `ichumuh:cramph`
addressing Jonas's review threads on ichumuh PR #14. Plan:
/home/jonas/.claude/plans/i-did-a-review-sleepy-engelbart.md

1. Executor.default_context_extensions  2. RealRobotPlanExecutor  3. delete querying/__init__.py
4. _node_running -> _as_node  5. step fields X | Match[X]  6. navigation subclasses override create_action_body
7. MovesToolCenterPoint Action superclass w/ MotionToleranceConfig  8. robot_specific_plan_transformations.py
9. ExecutorExtensionAttachment -> ObservedExecutor  10. presets reply only
Questions left to Simon: transporting.py:40, actions/base.py:106, open_close.py:21.

Done: branch created.
Next: implement 1-9, regen ORM, run tests, push, draft PR, reply (not resolve) on threads.
