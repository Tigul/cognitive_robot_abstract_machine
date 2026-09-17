from giskardpy.executor import Executor
from cramph.executor import SimulationPacer
from giskardpy.motion_statechart.context import MotionStatechartContext
from giskardpy.motion_statechart.graph_node import EndMotion
from cramph.monitors import CountSeconds
from giskardpy.motion_statechart.motion_statechart import MotionStatechart
from giskardpy.qp.qp_controller_config import QPControllerConfig
from semantic_digital_twin.world import World


def test_with_executor():
    msc = MotionStatechart()
    msc.add_node(counter := CountSeconds(seconds=1.0))
    msc.add_node(EndMotion.when_true(counter))

    kin_sim = Executor(
        context=MotionStatechartContext(
            world=World(),
            qp_controller_config=QPControllerConfig.create_with_simulation_defaults(),
        ),
        pacer=SimulationPacer(real_time_factor=2.0),
    )
    kin_sim.compile(msc)
    kin_sim.tick_until_end(timeout=1000)
    # we tick 20 (hz) * 2 (real_time_factor) per second and sleep for 1s.
    # +2 because the endmotion needs to extra ticks
    assert kin_sim.tick_count == 42
