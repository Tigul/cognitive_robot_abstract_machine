"""
Coverage for reading the documentation of attributes and parameters from source.
"""

from __future__ import annotations

from krrood.documentation import AttributeDocstrings, ParameterDescriptions

from .dataset.documented_attributes import (
    DocumentedBase,
    DocumentedSubclass,
    documented_function,
)

# %% attributes


def test_an_attribute_is_documented_by_the_string_below_it():
    docstring = AttributeDocstrings.docstring_of(DocumentedSubclass, "own")

    assert docstring == "Declared on the subclass, over two lines."


def test_an_inherited_attribute_is_documented_where_it_is_declared():
    docstring = AttributeDocstrings.docstring_of(DocumentedSubclass, "inherited")

    assert docstring == AttributeDocstrings.docstring_of(DocumentedBase, "inherited")
    assert docstring == "Declared on the base."


def test_an_attribute_without_a_string_below_it_is_undocumented():
    assert AttributeDocstrings.docstring_of(DocumentedBase, "undocumented") is None


def test_an_attribute_of_a_nested_class_is_documented():
    docstring = AttributeDocstrings.docstring_of(DocumentedSubclass.Nested, "inner")

    assert docstring == "Declared on a nested class."


def test_a_class_without_source_finds_the_documentation_of_its_bases():
    generated = type(
        "GeneratedSubclass",
        (DocumentedBase,),
        {"__annotations__": {"inherited": int}},
    )

    docstring = AttributeDocstrings.docstring_of(generated, "inherited")

    assert docstring == AttributeDocstrings.docstring_of(DocumentedBase, "inherited")


# %% parameters


def test_the_parameters_of_a_function_are_described_by_its_param_fields():
    descriptions = ParameterDescriptions.of_function(documented_function)

    assert descriptions.descriptions == {
        "first": "The first parameter, continued on a second line.",
        "second": "The second parameter.",
    }
