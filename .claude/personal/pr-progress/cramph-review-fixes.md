Branch `cramph-review-fixes` (from `simon/cramph`), draft PR
https://github.com/ichumuh/cognitive_robot_abstract_machine/pull/16 into `ichumuh:cramph`,
addressing Jonas's review threads on ichumuh PR #14.

Done (commit 73d947b78): executor default_context_extensions/default_extensions,
RealRobotPlanExecutor, querying/__init__.py deleted, _as_node + X | Match[X] step types,
navigation subclasses override create_action_body (drives_differentially flag),
MovesToolCenterPoint is an Action superclass requiring MotionToleranceConfig (user OK'd
updating key order in test_get_bound_variables), robot_specific_plan_transformations.py,
ObservedExecutor. Replied (not resolved) on 10 threads on #14.

Left for Simon: transporting.py:40, actions/base.py:106, open_close.py:21.
Known failures not caused here: test_tool_motion_frees_the_manipulator_holding_the_tool;
DAiSy xacro ur_type errors.
Next: nothing until the user asks (CI/review feedback).
