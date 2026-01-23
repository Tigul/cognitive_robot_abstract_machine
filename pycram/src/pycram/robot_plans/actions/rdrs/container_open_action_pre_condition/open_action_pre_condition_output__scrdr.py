from ripple_down_rules.utils import copy_case
from typing_extensions import Dict, Optional, Union
from ripple_down_rules.datastructures.case import Case
from types import NoneType
from ripple_down_rules.datastructures.case import create_case
from .open_action_pre_condition_output__scrdr_defs import *


attribute_name = "output_"
conclusion_type = (bool,)
mutually_exclusive = True
name = "output_"
case_type = Dict
case_name = "open_action_pre_condition"


def classify(case: Dict, **kwargs) -> Optional[bool]:
    if not isinstance(case, Case):
        case = create_case(case, max_recursion_idx=3)
    else:
        case = copy_case(case)

    if conditions_60982077575212225471015205799153969671(case):
        return conclusion_60982077575212225471015205799153969671(case)
    else:
        return None
