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
Merged ichumuh/cramph b897d8812 (commit d78520a5a): kept robot-specific module,
FreeEndEffectorsOfParallelStepsTogether stays in plan_transformations.py,
GiskardClientExecutor.default_context_extensions -> []. PR set back to draft.
Stretch process-boundary test also fails on plain ichumuh/cramph.
Commit 67b55db54: default_extensions -> default_executor_extensions; reverted
X | Match[X] step typing (ORMatic can't handle it). Replied on both #16 threads + #14 typing thread.
Next: nothing until the user asks (CI/review feedback).
