Branch cramph-kwonly (based on simon/cramph). Goal: every StatechartNode subclass
(cramph, giskardpy, coraplex, experiments, test mimics) gets keyword-only constructor
parameters (`@dataclass(..., kw_only=True)`), so PyCharm stops flagging missing params.
Plan:
1. [x] guard tests (cramph/giskardpy/coraplex) via test/cramph_test/test_statechart/keyword_only_constructors.py
2. [x] kw_only=True on 205 src + 26 test node/mixin decorators; field-level kw_only stripped
3. [x] 770 positional call sites rewritten (resolved against old signatures from a HEAD worktree); robokudo untouched (py_trees)
4. [x] docs md examples rewritten, ORM regenerated, format_docstrings run
5. [x] tests: cramph 835 ok; giskardpy 1078 ok + 4 pre-existing failures (also fail on HEAD); coraplex 710 ok; cramera 1064 ok; experiments 328 ok. robokudo not runnable locally (cv_bridge/numpy), no robokudo files changed
6. [ ] commit + PR when user asks (all changes uncommitted)
