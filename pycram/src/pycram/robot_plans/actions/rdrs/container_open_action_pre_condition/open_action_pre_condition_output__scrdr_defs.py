from ripple_down_rules.datastructures.case import Case
from semantic_digital_twin.reasoning.predicates import reachable
from typing_extensions import Dict, Optional, Union
from pycram.robot_plans.actions.core.container import OpenAction
from types import NoneType
from ripple_down_rules import *


def conditions_60982077575212225471015205799153969671(case) -> bool:
    def conditions_for_open_action_pre_condition(self_: OpenAction, **kwargs) -> bool:
        """Get conditions on whether it's possible to conclude a value for open_action_pre_condition.output_  of type ."""
        return True
    return conditions_for_open_action_pre_condition(**case)


def conclusion_60982077575212225471015205799153969671(case) -> bool:
    def open_action_pre_condition(self_: OpenAction, **kwargs) -> bool:
        """Get possible value(s) for open_action_pre_condition.output_  of type ."""
        # Write your code here
        handle = self_.object_designator
        return reachable(handle.global_pose, self_.robot_view.root, handle)
    return open_action_pre_condition(**case)


