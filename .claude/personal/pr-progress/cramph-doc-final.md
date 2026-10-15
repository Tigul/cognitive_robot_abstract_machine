PR: https://github.com/ichumuh/cognitive_robot_abstract_machine/pull/24 (draft, base ichumuh:cramph, head Tigul:cramph-doc-final)
Plan: bring cramph/doc (intro, statecharts, running_a_statechart example) in line with the implementation.
Done: checked every documented identifier and behaviour against the source; fixed condition-check errors, CancelStatechart factory exceptions, Gantt description; added sections on plan-language child editing, choosing a child while running, writing a node (+ shipped nodes), changing a running statechart, pacers/executor extensions. Example run verified, book builds. Commit 6481b749d5.
Next: await review. Open point not in scope: RepeatUntil docstring references giskardpy's RepeatOnStall.
