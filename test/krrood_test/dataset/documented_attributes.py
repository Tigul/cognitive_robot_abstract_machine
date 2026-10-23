"""
Classes and a function whose attributes and parameters are documented in their source.
"""

from __future__ import annotations

from dataclasses import dataclass

# %% classes


@dataclass
class DocumentedBase:
    """
    Declares an attribute its subclasses inherit.
    """

    inherited: int
    """
    Declared on the base.
    """

    undocumented: int = 0


@dataclass
class DocumentedSubclass(DocumentedBase):
    """
    Declares an attribute of its own next to the inherited one.
    """

    own: str = ""
    """
    Declared on the subclass, over two lines.
    """

    class Nested:
        """
        A class declared inside another.
        """

        inner: int
        """
        Declared on a nested class.
        """


# %% functions


def documented_function(first: int, second: str) -> None:
    """
    Does nothing.

    :param first: The first parameter, continued on a second line.
    :param second: The second parameter.
    :return: Nothing.
    """
