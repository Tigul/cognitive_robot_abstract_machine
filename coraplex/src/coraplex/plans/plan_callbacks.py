from __future__ import annotations

from dataclasses import dataclass, field

from typing_extensions import List

from cramph.data_types import LifeCycleValues
from cramph.node import StatechartNode
from cramph.statechart import StateHistory, StateHistoryObserver, Statechart

# %% observing a plan


@dataclass
class PlanCallback:
    """
    Observe plan execution; unimplemented events leave execution unchanged.
    """

    def on_compile(self, plan: StatechartNode, statechart: Statechart) -> None:
        """
        Observe a plan about to be compiled, before any of its nodes has started.

        :param plan: The root node of the plan.
        :param statechart: The statechart running the plan.
        """

    def on_start(self, node: StatechartNode) -> None:
        """
        Observe a node of the plan whose execution has begun.

        :param node: The started node.
        """

    def on_end(self, node: StatechartNode) -> None:
        """
        Observe a node of the plan after its execution ended.

        :param node: The ended node, in the life cycle state it ended in.
        """


@dataclass
class PlanCallbackDispatcher(StateHistoryObserver):
    """
    Reads the start and the end of every node of a plan off the snapshots its statechart
    records, and reports them to the callbacks.
    """

    plan: StatechartNode
    """
    The root node of the observed plan.
    """

    callbacks: List[PlanCallback] = field(default_factory=list)
    """
    The callbacks the starts and ends are reported to.
    """

    def on_state_change(self, history: StateHistory) -> None:
        """
        Report the nodes of the plan that started or ended in the newest snapshot.

        :param history: The history of the statechart running the plan.
        """
        current = history.history[-1]
        previous = history.history[-2] if len(history) > 1 else None
        for node in [self.plan, *self.plan.descendants]:
            if not current.records(node):
                continue
            current_state = current.life_cycle_state[node]
            previous_state = (
                previous.life_cycle_state[node]
                if previous is not None and previous.records(node)
                else LifeCycleValues.NOT_STARTED
            )
            if current_state == previous_state:
                continue
            if previous_state == LifeCycleValues.NOT_STARTED:
                for callback in self.callbacks:
                    callback.on_start(node)
            if current_state.is_terminal:
                for callback in self.callbacks:
                    callback.on_end(node)
