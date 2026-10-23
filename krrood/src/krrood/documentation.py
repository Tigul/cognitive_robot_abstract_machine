"""
Documentation of attributes and parameters that Python does not keep at runtime, read
from the source it is written in.
"""

from __future__ import annotations

import ast
import inspect
import re
import sys
from dataclasses import dataclass
from functools import cache
from pathlib import Path

from typing_extensions import Callable, Iterator, Self

from krrood.ormatic.utils import classproperty

# %% attributes


@dataclass(frozen=True)
class AttributeDocstrings:
    """
    The docstrings of the attributes of the classes of one module: the string written
    directly below the assignment or annotation of an attribute in a class body.
    """

    docstrings: dict[tuple[str, str], str]
    """
    The docstring of each documented attribute, by the qualified name of its class and
    its own name.
    """

    @classmethod
    @cache
    def of_module(cls, module_name: str) -> Self:
        """
        :param module_name: The name of an imported module.
        :return: The docstrings of the attributes of the module's classes; none if the
            module has no Python source.
        """
        source_file = vars(sys.modules[module_name]).get("__file__")
        if source_file is None or Path(source_file).suffix != ".py":
            return cls({})
        module = ast.parse(Path(source_file).read_text())
        return cls(dict(cls.docstrings_in(module.body, qualified_prefix="")))

    @classmethod
    def docstrings_in(
        cls, statements: list[ast.stmt], qualified_prefix: str
    ) -> Iterator[tuple[tuple[str, str], str]]:
        """
        :param statements: The statements of a module or a class body.
        :param qualified_prefix: The qualified name of the class whose body holds
            ``statements``, followed by a dot; empty for a module.
        :return: The docstring of each documented attribute of the classes defined in
            ``statements``, nested ones included.
        """
        for statement in statements:
            if not isinstance(statement, ast.ClassDef):
                continue
            qualified_name = f"{qualified_prefix}{statement.name}"
            for assignment, following in zip(statement.body, statement.body[1:]):
                attribute_name = cls.assigned_name(assignment)
                if attribute_name is None or not cls.is_string(following):
                    continue
                yield (qualified_name, attribute_name), inspect.cleandoc(
                    following.value.value
                )
            yield from cls.docstrings_in(statement.body, f"{qualified_name}.")

    @staticmethod
    def assigned_name(statement: ast.stmt) -> str | None:
        """
        :param statement: A statement of a class body.
        :return: The name of the attribute ``statement`` assigns or annotates, or
            ``None`` if it declares no single attribute.
        """
        if isinstance(statement, ast.AnnAssign) and isinstance(
            statement.target, ast.Name
        ):
            return statement.target.id
        if (
            isinstance(statement, ast.Assign)
            and len(statement.targets) == 1
            and isinstance(statement.targets[0], ast.Name)
        ):
            return statement.targets[0].id
        return None

    @staticmethod
    def is_string(statement: ast.stmt) -> bool:
        """
        :param statement: A statement of a class body.
        :return: Whether ``statement`` is a string literal and nothing else.
        """
        return (
            isinstance(statement, ast.Expr)
            and isinstance(statement.value, ast.Constant)
            and isinstance(statement.value.value, str)
        )

    @classmethod
    def docstring_of(cls, owner: type, attribute_name: str) -> str | None:
        """
        :param owner: A class with the attribute, declared on it or inherited.
        :param attribute_name: The name of the attribute.
        :return: The docstring of the attribute in the first class of ``owner``'s method
            resolution order that documents it, or ``None`` if none does.
        """
        for declaring_class in owner.__mro__:
            docstring = cls.of_module(declaring_class.__module__).docstrings.get(
                (declaring_class.__qualname__, attribute_name)
            )
            if docstring is not None:
                return docstring
        return None


# %% parameters


@dataclass(frozen=True)
class ParameterDescriptions:
    """
    The descriptions a function's docstring gives its parameters, in reStructuredText
    ``:param name:`` fields.
    """

    descriptions: dict[str, str]
    """
    The description of each described parameter, on one line, by its name.
    """

    @classproperty
    def parameter_field(cls) -> re.Pattern[str]:
        """
        :return: A ``:param name:`` field, up to the next field or the end of the
            docstring.
        """
        return re.compile(
            r"^:param (?P<name>\w+):(?P<description>.*?)(?=^:|\Z)",
            re.MULTILINE | re.DOTALL,
        )

    @classmethod
    def of_function(cls, function: Callable[..., object]) -> Self:
        """
        :param function: A documented function or method.
        :return: The descriptions its docstring gives its parameters.
        """
        docstring = inspect.getdoc(function) or ""
        return cls(
            {
                match["name"]: " ".join(match["description"].split())
                for match in cls.parameter_field.finditer(docstring)
            }
        )
