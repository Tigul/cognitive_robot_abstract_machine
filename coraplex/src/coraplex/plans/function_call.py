from __future__ import annotations

import logging
import threading
from dataclasses import dataclass, field

from typing_extensions import Any, Callable, Optional

from coraplex.plans.failures import PlanFailure
from cramph.context import StatechartContext
from cramph.data_types import ObservationStateValues, SuccessDecider
from cramph.node import StatechartNode

logger = logging.getLogger(__name__)


@dataclass(eq=False, repr=False)
class FunctionCall(StatechartNode):
    """
    Calls a function once when it starts, in a thread of its own.

    It succeeds once the function returned and fails if the function raised a
    :class:`~coraplex.plans.failures.PlanFailure`, so a surrounding node can react to
    that failure. Any other exception is raised out of the tick.

    The tick after the start waits for the function, so no control cycle passes while
    it runs, and functions started in the same tick run at the same time.

    .. warning:: The function is not serializable, so this node only works in a locally
        ticked statechart.
    """

    success_decided_by = SuccessDecider.ITSELF
    fails_when_observing_false = True

    function: Callable[[], Any] = field(kw_only=True)
    """
    The function to call.
    """

    _thread: Optional[threading.Thread] = field(default=None, init=False, repr=False)
    """
    The thread the function runs in, while it runs.
    """

    _error: Optional[BaseException] = field(default=None, init=False, repr=False)
    """
    What the function raised, if it raised.
    """

    def _call_function(self) -> None:
        """
        Call :attr:`function` and keep what it raises, since a thread cannot raise into
        the tick.
        """
        try:
            self.function()
        except BaseException as error:  # noqa: BLE001 - handed to the tick
            self._error = error

    def on_start(self, context: StatechartContext) -> None:
        self._error = None
        self._thread = threading.Thread(
            target=self._call_function, name=self.unique_name, daemon=True
        )
        self._thread.start()

    def on_tick(self, context: StatechartContext) -> Optional[ObservationStateValues]:
        """
        Wait for the function and observe whether it succeeded.

        :raises BaseException: What the function raised, unless it is a
            :class:`~coraplex.plans.failures.PlanFailure`.
        """
        self._thread.join()
        if self._error is None:
            return ObservationStateValues.TRUE
        if isinstance(self._error, PlanFailure):
            logger.info("%s failed: %s", self.unique_name, self._error)
            return ObservationStateValues.FALSE
        raise self._error
