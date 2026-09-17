import time
from abc import ABC, abstractmethod
from dataclasses import dataclass, field


from cramph.context import StatechartContext
from cramph.exceptions import NonPositiveRealTimeFactorError
from cramph.statechart import Statechart
from krrood.symbolic_math.symbolic_math import FloatVariable


@dataclass
class Pacer(ABC):
    """
    Decides how long a loop waits between two cycles.
    """

    target_frequency: float = field(init=False)
    """
    Frequency of the loop in hertz, set by whoever runs the loop.
    """

    def pace_ticks_of(self, context: StatechartContext) -> None:
        """
        Sets :attr:`target_frequency` to one cycle per tick of `context`.

        :param context: The context whose ticks this pacer paces.
        :raises TickDurationUnknownError: If `context` does not know how long a tick
            lasts.
        """
        self.target_frequency = 1 / context.require_tick_duration()

    @abstractmethod
    def sleep(self) -> None:
        """
        Wait until the loop may start its next cycle.
        """


@dataclass
class NoPacing(Pacer):
    """
    Lets a loop run as fast as the hardware allows.
    """

    def pace_ticks_of(self, context: StatechartContext) -> None:
        """
        Does nothing, since this pacer never waits.
        """

    def sleep(self) -> None:
        pass


@dataclass
class ScheduledPacer(Pacer, ABC):
    """
    Holds a loop at a fixed cycle duration by sleeping until the next slot.

    A cycle that overruns its slot is not compensated by a shorter following one; the
    schedule simply skips to the next slot after the current time.
    """

    _next_target_time: float | None = field(default=None, init=False)
    """
    Point in time the next cycle may start at, None until the first sleep.
    """

    @property
    @abstractmethod
    def cycle_duration(self) -> float:
        """
        How many seconds one cycle should take.
        """

    def sleep(self) -> None:
        cycle_duration = self.cycle_duration
        now = time.monotonic()
        if self._next_target_time is None:
            self._next_target_time = now + cycle_duration
        sleep_time = self._next_target_time - now
        if sleep_time > 0:
            time.sleep(sleep_time)
            now = self._next_target_time
        while self._next_target_time <= now:
            self._next_target_time += cycle_duration


@dataclass
class RealTimePacer(ScheduledPacer):
    """
    Holds a loop at its target frequency in wall clock time.
    """

    @property
    def cycle_duration(self) -> float:
        return 1 / self.target_frequency


@dataclass
class SimulationPacer(ScheduledPacer):
    """
    Runs a loop at a multiple of its target frequency to speed up or slow down a
    simulation.
    """

    real_time_factor: float = 1.0
    """
    How much faster than real time the loop runs; ``2.0`` is twice as fast.
    """

    def __post_init__(self):
        if self.real_time_factor <= 0:
            raise NonPositiveRealTimeFactorError(self.real_time_factor)

    @property
    def cycle_duration(self) -> float:
        return 1 / (self.target_frequency * self.real_time_factor)


@dataclass
class StatechartExecutor:
    """
    Compiles a statechart and ticks it, counting the ticks.
    """

    context: StatechartContext
    """
    The context handed to every node of the statechart.
    """

    pacer: Pacer = field(default_factory=NoPacing, kw_only=True)
    """
    Paces the loop that ticks this executor.
    """

    # %% init False
    statechart: Statechart | None = field(init=False, default=None)
    """
    The statechart that is executed, set by :meth:`compile`.
    """

    def __post_init__(self):
        self.pacer.pace_ticks_of(self.context)
        self._create_tick_variable()

    def _create_tick_variable(self):
        """
        Registers the variable counting the ticks in the context.
        """
        self.context.tick_variable = FloatVariable("tick_count")
        self.context.float_variable_data.register_expression(self.context.tick_variable)

    @property
    def time(self) -> float:
        """
        :return: How many seconds the ticks run so far stand for.
        """
        return self.tick_count * self.context.require_tick_duration()

    @property
    def tick_count(self) -> int:
        """
        :return: The number of ticks run since :meth:`compile`.
        """
        return self.context.tick_count

    @tick_count.setter
    def tick_count(self, value: int):
        self.context.float_variable_data.set_value(self.context.tick_variable, value)

    def compile(self, statechart: Statechart):
        """
        Compiles `statechart` and ticks it once, so that nodes whose start condition is
        constant true start immediately.

        :param statechart: The statechart to execute.
        """
        self.statechart = statechart
        self.tick_count = 0
        self.statechart.compile(self.context)
        self._after_compile()
        self.statechart.tick(self.context)

    def tick(self):
        """
        Advances the statechart by one tick.
        """
        self._before_tick()
        self.tick_count += 1
        self.statechart.tick(self.context)
        self._after_tick()

    def tick_until_end(self, timeout: int = 1_000):
        """
        Calls tick until
        :meth:`~cramph.statechart.Statechart.is_ended`
        returns True.

        :param timeout: Max number of ticks to perform.
        """
        try:
            for i in range(timeout):
                self.tick()
                self.pacer.sleep()
                if self.statechart.is_ended():
                    return
            raise TimeoutError("Timeout reached while waiting for end of statechart.")
        finally:
            self._after_run()
            self.statechart.cleanup_nodes(context=self.context)
            self.context.cleanup()

    def _after_compile(self):
        """
        Called once the statechart is compiled, before its first tick.
        """

    def _before_tick(self):
        """
        Called at the start of every tick, before the tick count is advanced.
        """

    def _after_tick(self):
        """
        Called at the end of every tick, after the statechart was ticked.
        """

    def _after_run(self):
        """
        Called once :meth:`tick_until_end` stops, before the nodes are cleaned up.
        """
