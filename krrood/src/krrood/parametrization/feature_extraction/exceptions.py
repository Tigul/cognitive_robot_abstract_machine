from __future__ import annotations

from dataclasses import dataclass

from typing_extensions import Type

from krrood.exceptions import DataclassException, InputError


@dataclass
class NoInstancesProvidedError(InputError):
    """
    Raised when a :class:`~krrood.parametrization.feature_extraction.feature_extractor.FeatureExtractor`
    is constructed from an empty instance list.
    """

    def error_message(self) -> str:
        return "At least one instance must be provided to build a FeatureExtractor."

    def suggest_correction(self) -> str:
        return "Pass a non-empty list of DAO instances to FeatureExtractor.from_instances."


@dataclass
class UnsupportedFeatureTypeError(DataclassException):
    """
    Raised when a feature column has a type that cannot be preprocessed for a
    :class:`~probabilistic_model.learning.jpt.jpt.JointProbabilityTree`.
    """

    feature_type: Type
    """
    The unsupported type encountered during preprocessing.
    """

    column_name: str
    """
    The name of the dataframe column that triggered the error.
    """

    def error_message(self) -> str:
        return f"Unsupported type {self.feature_type!r} for column '{self.column_name}'."

    def suggest_correction(self) -> str:
        return (
            "Only bool, enum, and random_events compatible types are supported. "
            "Ensure the feature column maps to one of those types."
        )
