from copy import copy
from dataclasses import dataclass, field

from krrood.utils import (
    clear_memoization_cache,
    detach_memoization_cache,
    memoize,
)

# %% subject


@dataclass(eq=False)
class CountsItsCalls:
    """
    Records how often the memoized member actually ran, so a cache hit is observable.

    ``memoize`` keys its cache on the instance, so this must stay hashable.
    """

    calls: int = field(default=0)
    """
    The number of times :attr:`value` computed a result instead of serving a cached one.
    """

    @property
    @memoize
    def value(self) -> int:
        self.calls += 1
        return self.calls


# %% caching


def test_memoize_computes_once_per_instance():
    subject = CountsItsCalls()

    assert subject.value == 1
    assert subject.value == 1
    assert subject.calls == 1


def test_clear_memoization_cache_forces_a_recomputation():
    subject = CountsItsCalls()
    subject.value

    clear_memoization_cache(subject)

    assert subject.value == 2


# %% shallow copies


def test_a_shallow_copy_shares_the_cache_until_it_is_detached():
    """
    ``copy`` carries the cache dict over by reference, so whatever the copy memoizes is
    retained by the original as well.

    That matters when the cached value holds a large object the original should not keep
    alive.
    """
    original = CountsItsCalls()
    original.value

    shared = copy(original)
    assert shared.__memo__ is original.__memo__

    detached = copy(original)
    detach_memoization_cache(detached)

    assert detached.__memo__ is not original.__memo__


def test_detached_copy_does_not_fill_the_original_cache():
    original = CountsItsCalls()
    original.value
    cached_entries = len(original.__memo__)

    detached = copy(original)
    detach_memoization_cache(detached)
    detached.value

    assert len(original.__memo__) == cached_entries


def test_detaching_leaves_the_original_cache_intact():
    """
    Unlike :func:`clear_memoization_cache`, which empties the dict both instances share,
    detaching only replaces the copy's reference.
    """
    original = CountsItsCalls()
    original.value

    detach_memoization_cache(copy(original))

    assert original.value == 1
    assert original.calls == 1
