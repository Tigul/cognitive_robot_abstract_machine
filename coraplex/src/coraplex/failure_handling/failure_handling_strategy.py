from dataclasses import dataclass


@dataclass
class FailureResolution:
    """
    The resolution a :class:`FailureHandlingStrategy` decided on for a handled failure.

    Fully defined by work package WP3 of the coraplex failure-handling roadmap
    (`coraplex/ROADMAP.md`): a hierarchy with an `apply` method that either retries the
    failing frame or propagates the failure further up the plan.
    """


@dataclass
class FailureHandlingStrategy:
    pass
