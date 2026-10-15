Branch cramph-kwonly (based on simon/cramph). Goal: every StatechartNode subclass
(cramph, giskardpy, coraplex, robokudo/experiments) gets keyword-only constructor
parameters (`@dataclass(..., kw_only=True)`), so PyCharm stops flagging missing params.
Plan:
1. [ ] guard tests (cramph/giskardpy/coraplex) asserting every init field is kw_only
2. [ ] add kw_only=True to every node + mixin dataclass decorator; drop field-level kw_only
3. [ ] rewrite ~714 positional call sites (import-resolved; skip py_trees Sequence/Parallel)
4. [ ] docs/notebooks, regenerate ORM, format_docstrings
5. [ ] run test packages one at a time
