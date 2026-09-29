"""effective_sample_size and systematic_resample (Task 4)."""
import numpy as np
import pytest

from r7021e_fast_slam.resampling import (effective_sample_size,
                                         systematic_resample)


def test_neff_equal_weights_is_exactly_n():
    ## 1/64 and its square are exact in binary floating point, so this one
    ## can be checked with ==, not approx.
    n = 64
    assert effective_sample_size(np.full(n, 1.0 / n)) == float(n)


@pytest.mark.parametrize('n', [1, 7, 50, 100])
def test_neff_equal_weights_is_n(n):
    assert effective_sample_size(np.full(n, 1.0 / n)) == pytest.approx(n, rel=1e-12)


def test_neff_one_hot_is_exactly_one():
    w = np.zeros(20)
    w[13] = 1.0
    assert effective_sample_size(w) == 1.0


def test_neff_zero_weights_does_not_divide_by_zero():
    assert effective_sample_size(np.zeros(5)) == 0.0


@pytest.mark.parametrize('seed', range(20))
def test_systematic_counts_are_floor_or_ceil(seed):
    rng = np.random.default_rng(seed)
    n = 37
    w = rng.random(n) ** 3              # deliberately uneven
    w /= w.sum()
    idx = systematic_resample(w, rng)
    assert idx.shape == (n,)
    assert idx.dtype.kind == 'i'
    assert idx.min() >= 0 and idx.max() < n
    counts = np.bincount(idx, minlength=n)
    expected = n * w
    assert np.all(counts >= np.floor(expected) - 1e-9)
    assert np.all(counts <= np.ceil(expected) + 1e-9)


def test_systematic_one_hot_returns_that_index_n_times(rng):
    n = 25
    w = np.zeros(n)
    w[17] = 1.0
    idx = systematic_resample(w, rng)
    assert np.array_equal(idx, np.full(n, 17))


def test_systematic_never_picks_a_zero_weight_particle(rng):
    w = np.array([0.0, 0.5, 0.0, 0.5, 0.0])
    for _ in range(200):
        idx = systematic_resample(w, rng)
        assert set(idx.tolist()) <= {1, 3}


def test_systematic_survives_weights_that_sum_slightly_below_one(rng):
    ## Floating-point drift in the normalized weights must not produce an
    ## index of N.
    w = np.full(10, 0.1) * (1.0 - 1e-12)
    idx = systematic_resample(w, rng)
    assert idx.max() <= 9
