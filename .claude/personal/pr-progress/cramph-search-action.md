Branch cramph-search-action -> draft PR ichumuh/cognitive_robot_abstract_machine#23 (base cramph).

Plan: port SearchAction from Tigul:search-action-again onto cramph; language/giskard
template changes of that branch are superseded by cramph.composites and not ported.

Done:
- 5f1b29d6d SearchAction, PerceptionTask fails on nothing seen (look_for split),
  DetectAction.detection_box (Stretch's sideways camera fell outside the default
  base-frame box y in [-1, 3]). test_search_action passes for all 5 robots.
- Pushed to origin (Tigul fork), draft PR #23 opened.

Next / outstanding:
- Pre-existing failure on cramph: test_plan_execution.py::
  test_a_simulated_plan_waits_for_its_pacer_after_every_tick (duplicate 'pacer' kwarg).
- DetectAction._build_query still raises a bare AttributeError (untouched).
