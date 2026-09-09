"""
Importing the generated ORM interfaces of a checkout, one after another in one
interpreter.

An interface that no longer matches the classes it maps is the one thing a checkout has
to rebuild for, and the only way to find out is to import it. Doing so here keeps what
the interfaces pull in out of the interpreter that asked, which is what lets a build
follow a failed import without leaving a stale interface behind in it.

Started as ``python -P -m cognitive_robot_abstract_machine.orm_import <module>...`` by
:mod:`cognitive_robot_abstract_machine.orm_interfaces`, which reads the report below back
from this run's output.
"""

from __future__ import annotations

import argparse
import importlib
import sys
from dataclasses import dataclass
from enum import IntEnum

from typing_extensions import Optional, Sequence, Tuple, Type

LINE_PREFIX = "cram-orm-interface-stale "
"""
What marks a line of output as this run's report rather than as something an interface
wrote while it was imported.
"""

FIELD_SEPARATOR = " | "
"""
What separates the parts of the report on one line.
"""

STALENESS_ERRORS: Tuple[Type[Exception], ...] = (ImportError, AttributeError)
"""
What an interface raises once it no longer matches the classes it maps: a module that has
been renamed or removed raises :class:`ImportError`, and a class raises
:class:`AttributeError`, because generated code reaches every class it maps as an
attribute of the module holding it.
"""


class InterfaceImportResult(IntEnum):
    """
    What importing the interfaces of a checkout ended in, as the exit code of a run.
    """

    IMPORTED = 0
    """
    Every interface imported.
    """

    STALE = 3
    """
    One of them no longer matches the classes it maps, held apart from the ``1`` an
    interface failing for any other reason exits with.
    """


# %% what a run reports about itself


@dataclass
class StaleInterface:
    """
    An interface that no longer matches the classes it maps.
    """

    module_name: str
    """
    The interface, as it is imported.
    """

    error_type: str
    """
    Name of what it raised.
    """

    error_description: str
    """
    What it said, on one line.
    """

    @classmethod
    def from_error(cls, module_name: str, error: Exception) -> StaleInterface:
        """
        Report an interface by what importing it raised.

        :param module_name: The interface that was being imported.
        :param error: What it raised.
        """
        return cls(module_name, type(error).__name__, " ".join(str(error).split()))

    @classmethod
    def from_line(cls, line: str) -> Optional[StaleInterface]:
        """
        Read a report back from a line of this run's output.

        :param line: One line of output, of any kind.
        :return: The report the line carries, or nothing when it carries none.
        """
        if not line.startswith(LINE_PREFIX):
            return None
        module_name, error_type, error_description = line[len(LINE_PREFIX) :].split(
            FIELD_SEPARATOR, maxsplit=2
        )
        return cls(module_name, error_type, error_description.strip())

    @classmethod
    def from_output(cls, output: str) -> Optional[StaleInterface]:
        """
        Read the report back from everything this run wrote.

        :param output: The output of the run, reports and interface writing alike.
        :return: The report it carries, or nothing when it carries none.
        """
        for line in output.splitlines():
            reported = cls.from_line(line)
            if reported is not None:
                return reported
        return None

    def to_line(self) -> str:
        """
        Render this report as one line of output.

        :return: The line, without its terminating line break.
        """
        return LINE_PREFIX + FIELD_SEPARATOR.join(
            (self.module_name, self.error_type, self.error_description)
        )

    def report(self) -> str:
        """
        Say which interface has to be built again, and what it raised.
        """
        return (
            f"{self.module_name} no longer matches the classes it maps: "
            f"{self.error_type}: {self.error_description}"
        )


# %% importing the interfaces


def import_interfaces(module_names: Sequence[str]) -> Optional[StaleInterface]:
    """
    Import interfaces in the order given, stopping at the first that no longer matches
    the classes it maps.

    :param module_names: The interfaces to import, in dependency order.
    :return: The first interface that no longer matches, or nothing when every one of
        them imported.
    """
    for module_name in module_names:
        try:
            importlib.import_module(module_name)
        except STALENESS_ERRORS as error:
            return StaleInterface.from_error(module_name, error)
    return None


def main() -> None:
    """
    Import every interface named on the command line, and report the first that no
    longer matches the classes it maps.
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "interfaces",
        nargs="+",
        help="The interface modules to import, in dependency order.",
    )
    stale = import_interfaces(parser.parse_args().interfaces)
    if stale is None:
        return
    print(stale.to_line(), flush=True)
    sys.exit(InterfaceImportResult.STALE)


if __name__ == "__main__":
    main()
