"""
Plan and history fixtures for browser execution observers.
"""

from __future__ import annotations

from dataclasses import dataclass

import pytest

from cramph.composites import Sequence
from cramph.context import StatechartContext
from cramph.data_types import LifeCycleValues
from cramph.node import StatechartNode
from cramph.nodes_for_testing import ConstTrueNode
from cramph.statechart import StateHistoryItem, Statechart
from semantic_digital_twin.world import World

from cramera.live.bridge import Bridge
from cramera.live.visualization import BridgePlanCallback

# %% motion execution fixture


@dataclass
class MotionExecution:
    """
    A plan and the statechart running it, observed by one visualization callback.
    """

    plan: Sequence
    """
    The root of the plan.
    """

    motion: StatechartNode
    """
    The step the plan runs.
    """

    chart: Statechart
    """
    The statechart that records life cycle changes.
    """

    bridge: Bridge
    """
    The published plan and chart state.
    """

    callback: BridgePlanCallback
    """
    The subscriber observing this plan's history.
    """

    def compile(self) -> None:
        """
        Tell the callback the plan is about to run, as an executor does on compile.
        """
        self.callback.on_compile(self.plan, self.chart)

    def record(self, state: LifeCycleValues) -> None:
        """
        Record one life cycle state of every node as a snapshot.

        :param state: The state assigned to every chart node.
        """
        self.chart.life_cycle_state.data[:] = state
        self.chart.history.append(
            StateHistoryItem(
                tick_count=len(self.chart.history),
                life_cycle_state=self.chart.life_cycle_state,
                observation_state=self.chart.observation_state,
            )
        )


@pytest.fixture()
def motion_execution() -> MotionExecution:
    """
    A plan of one step in a statechart, without anything ticking it.
    """
    motion = ConstTrueNode(name="Transport")
    plan = Sequence([motion])
    chart = Statechart(context=StatechartContext(world=World()))
    chart.add_node(plan)
    bridge = Bridge()
    callback = BridgePlanCallback(bridge=bridge)
    return MotionExecution(plan, motion, chart, bridge, callback)
