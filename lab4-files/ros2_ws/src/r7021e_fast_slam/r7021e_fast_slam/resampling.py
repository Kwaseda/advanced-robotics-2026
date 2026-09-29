"""Effective sample size and systematic resampling.

Systematic resampling (stochastic universal sampling (SUS)) draws a single uniform and spaces the remaining N-1
positions deterministically. That gives lower variance than drawing N
independent uniforms, and it guarantees a particle holding weight w is
selected either floor(N*w) or ceil(N*w) times.
"""
import numpy as np


def effective_sample_size(weights) -> float:
    """N_eff = 1 / sum(w^2), for NORMALIZED weights.

    Args:
        weights: (N,) array of normalized particle weights.
    Returns:
        float: the effective sample size, between 1 and N.

    Returns 0.0 for all-zero weights, so the caller resamples.
    """
    w = np.asarray(weights, dtype=float)
    denom = float(np.sum(w * w))
    if denom <= 0.0:
        return 0.0
    return 1.0 / denom


def systematic_resample(weights, rng) -> np.ndarray:
    """Systematic resampling of particles based on their weights.

    Args:
        weights: (N,) array of normalized particle weights.
        rng: a random number generator with a `random()` method.
    Returns:
        np.ndarray: (N,) array of ancestor indices.

    Returns ancestor indices only; the caller rebuilds and deep-copies.
    """
    w = np.asarray(weights, dtype=float)
    n = w.size
    positions = (rng.random() + np.arange(n)) / n
    cum = np.cumsum(w)
    ## Guards against float drift pushing a pointer past the end.
    cum[-1] = 1.0
    ## Half-open stretches: a zero-weight particle can never be picked.
    idx = np.searchsorted(cum, positions, side='right')
    return np.clip(idx, 0, n - 1).astype(int)
