Branch reattach-statechart-node (plan item reattach-statechart-node), off plan-cramp-second-iter.
Approved plan: /home/jdech/.claude/plans/staged-orbiting-axolotl.md (session e9226dc7).
User decisions: rebuild every node on a world-structure change (not only add-after-move);
detection independent of Statechart.modify() (world change != statechart change);
robot stands still while rebuilding (before_recompile hook).

Done, committed 89818aa1f and pushed:
- cramph: StatechartNode.set_up (once per context, on first build; _set_up_context flag on node,
  deviates from plan's "track in statechart" because tests build nodes standalone and reuse
  nodes across contexts). Statechart records world revision at compile; tick() checks before
  settle and after callbacks -> rebuild all nodes + recompile tick; compile() after a structure
  change also rebuilds old nodes. RecompileCallback/ExecutorExtension.before_recompile.
  MoveBranch node (cramph/world_modification_nodes.py).
- giskardpy: registrations moved to set_up (CartesianTask/trajectory/straight bindings,
  RootRelativeGoalMonitor binding, velocity-convergence start variable, wiggle insert, ROS
  topic/action nodes); MotionControl.before_recompile zeroes derivatives; ControlLoop registers
  HaltWhileRecompiling (ControlLoop.stop()). Collision registration stays in build (idempotent,
  group-dependent). coraplex perception source -> set_up.
- Tests: cramph test_moving_a_branch.py (9), giskardpy test_moving_a_branch.py (3),
  test_world_updates.py TestModelChangesOfTheMotionItself (1, own ROS-free fixture).
- Pre-existing: test_world_updates control_loop fixture calls Statechart() without context
  (3 errors on base too) - not touched.
- Review round 1 (PR #9): _build_again and _tell_recompile_callbacks_* removed, the
  world-structure rebuild goes through compile() (commit 1a83f1574). motion_control comment withdrawn.
  Replies/resolve/draft NOT done: token gets 403 on PR writes.
- FOLLOW-UP (separate change, in krrood): leak in FloatVariableData._bound_arguments.
  CartesianPositionTrajectory.compile_current_point_on_tick (giskardpy cartesian_tasks.py) calls
  bind_argument() from build_artifacts, so every rebuild appends a BoundArgument and the previous
  compiled function is never released; register_expression re-points the dead ones too. Harmless to
  correctness, costs memory + work per registration. Fix idea: make the binding replaceable (unbind,
  or key the BoundArgument by owning node so a rebuild replaces its entry); needs its own failing
  test first (count _bound_arguments after N rebuilds). Check other bind_argument callers too.
- Passing: cramph + giskardpy test_motion_statechart (832 before last fix).
Verified: cramph+giskard motion statechart 833 passed; coraplex plan/actions/designator 267 passed;
ROS2+SDT world 44 failed/56 errors, all identical on base (none new).
Next: open draft PR (body at scratchpad pr-body-reattach.md); PR: gh token lacks pull-request write on Tigul fork (403) and cram2 org rejects
token for gh pr create -> user must fix token; then bootstrap open --pull-request-number.
