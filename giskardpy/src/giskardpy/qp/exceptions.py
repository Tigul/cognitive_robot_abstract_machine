"""
Exceptions raised while building and solving the quadratic program.
"""

from giskardpy.data_types.exceptions import GiskardException, DontPrintStackTrace


class QPSolverException(GiskardException):
    """Base class for errors raised by the QP solvers."""


class InfeasibleException(QPSolverException):
    """Raised when the QP has no feasible solution."""


class VelocityLimitUnreachableException(QPSolverException):
    """Raised when a degree of freedom cannot reach its velocity limit within the prediction horizon."""


class OutOfJointLimitsException(InfeasibleException):
    """Raised when a degree of freedom is outside its position limits and cannot recover."""


class HardConstraintsViolatedException(InfeasibleException):
    """Raised when hard constraints cannot be satisfied."""


class EmptyProblemException(InfeasibleException, DontPrintStackTrace):
    def __init__(self):
        super().__init__("Empty QP problem.")


class MismatchedLimitLengthsError(GiskardException):
    """Raised when the bounds, weights, and names of a DirectLimits do not all share the same length."""


class ConstraintTypeMismatchError(QPSolverException):
    """Raised when an enforcement strategy receives a constraint of the wrong type for the requested bounds."""


class NoFactoryForQPDataTypeError(QPSolverException):
    """Raised when no registered factory handles the requested QPData type."""
