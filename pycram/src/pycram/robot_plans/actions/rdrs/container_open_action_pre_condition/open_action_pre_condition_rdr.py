from ripple_down_rules.datastructures.case import create_case
from typing_extensions import Any, Dict
from ripple_down_rules.datastructures.case import Case
from ripple_down_rules.helpers import general_rdr_classify
from . import open_action_pre_condition_output__scrdr as output__classifier

name = "output_"
case_type = Dict
case_name = "open_action_pre_condition"
classifiers_dict = dict()
classifiers_dict["output_"] = output__classifier


def classify(case: Dict, **kwargs) -> Dict[str, Any]:
    if not isinstance(case, Case):
        case = create_case(case, max_recursion_idx=3)
    return general_rdr_classify(classifiers_dict, case, **kwargs)
