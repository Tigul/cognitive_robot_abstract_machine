"""
Tests for the failures a plan raises about the motions it ran.
"""

from giskardpy.motion_statechart.graph_node import MotionStatechartNode
from cramph.statechart import Statechart
from cramph.nodes_for_testing import ConstFalseNode
from semantic_digital_twin.world import World

from coraplex.exceptions import MotionDidNotFinish
from giskardpy.motion_control import MotionControl
from cramph.context import StatechartContext
from cramph.executor import StatechartExecutor


def _running_motion() -> MotionStatechartNode:
    """
    :return: A node of a compiled statechart that has been ticked, so it is in a life
        cycle state a failure can report.
    """
    motion = ConstFalseNode(name="motion")
    executor = StatechartExecutor(
        context=StatechartContext(world=World()), extensions=[MotionControl()]
    )
    motion_statechart = Statechart(context=executor.context)
    motion_statechart.add_node(motion)
    executor.compile(statechart=motion_statechart)
    executor.tick()
    return motion


def test_the_failure_names_the_state_each_unfinished_motion_is_in():
    """
    A plan reports its motions in the vocabulary the motion statechart itself uses, so
    nothing translates between two sets of state names.
    """
    motion = _running_motion()

    message = MotionDidNotFinish([motion]).error_message()

    assert f"{motion.unique_name} ({motion.life_cycle_state.name})" in message
