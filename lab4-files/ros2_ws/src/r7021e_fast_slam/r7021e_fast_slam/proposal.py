"""The improved proposal distribution of Grid-FastSLAM 2.0.

The Gaussian approximation derived in hello-slam's `4_grid_based_slam.ipynb`:
sample K poses around the scan match, weight each by

    tau_j = p(z_t | x_j, m_t-1) * p(x_j | x_t-1, u_t),

and fit a Gaussian to the weighted cloud,

    mu = (1/eta) sum_j x_j tau_j,   Sigma = (1/eta) sum_j (x_j - mu)(x_j - mu)^T tau_j.

The normalizer eta = sum_j tau_j of that same product is the particle's
importance weight, which is why it is returned alongside the moments.

Everything is in log space. The raw products underflow within a handful of
beams.
"""
import numpy as np

from .measurement_model import measurement_log_likelihood
from .motion_model import motion_model_log_pdf
from .utils import logsumexp, wrap_angle

## Regularization of the fitted covariance. With all the tau mass on a single
## candidate the scatter matrix is exactly singular and the sampler's Cholesky
## would fail. SIGMA_REG is an additive variance floor per axis (x, y, theta);
## SIGMA_EIG_MIN floors the eigenvalues afterwards.
SIGMA_REG = (1.0e-4, 1.0e-4, 1.0e-5)
SIGMA_EIG_MIN = 1.0e-6


def _lattice_size(num_candidates) -> int:
    """Largest n with n**3 <= num_candidates, at least 1. Integer count,
    not a rounded cube root."""
    k = int(num_candidates)
    n = 1
    while (n + 1) ** 3 <= k:
        n += 1
    while n > 1 and n ** 3 > k:
        n -= 1
    return n


def proposal_candidates(x_star, cfg) -> np.ndarray:
    """K candidate poses around the scan-matched pose.

    A regular lattice: it covers the small search window evenly.
    K is rounded down to a perfect cube, e.g. 27 gives 3 x 3 x 3.

    Args:
        x_star:     (3,) [x, y, theta] the scan-matched pose at the current time step.
        cfg:        configuration object with proposal parameters.

    Returns:
        (K, 3) numpy array of candidate poses, where K is the number of candidates.

    The windows are the FULL width: the lattice spans -w/2..+w/2 around
    x_star. scan_matcher.candidate_grid spans -win..+win instead.
    """
    x_star = np.asarray(x_star, dtype=float)
    n = _lattice_size(cfg.num_candidates)
    if n == 1:
        d_xy = np.zeros(1)
        d_th = np.zeros(1)
    else:
        w_xy = float(cfg.proposal_window_xy)
        w_th = float(cfg.proposal_window_theta)
        d_xy = np.linspace(-0.5 * w_xy, 0.5 * w_xy, n)
        d_th = np.linspace(-0.5 * w_th, 0.5 * w_th, n)
    dx, dy, dth = np.meshgrid(d_xy, d_xy, d_th, indexing='ij')
    return np.stack([x_star[0] + dx.ravel(),
                     x_star[1] + dy.ravel(),
                     wrap_angle(x_star[2] + dth.ravel())], axis=1)


def improved_proposal(x_star, x_prev, u, endpoints, grid_map, cfg):
    """Return (log_eta, mu, Sigma) for one particle.

    Args:
        x_star:     (3,) [x, y, theta] the scan-matched pose at the current time step.
        x_prev:     (3,) [x, y, theta] the previous pose of the particle.
        u:          (3,) [rot1, trans, rot2] the odometry increment.
        endpoints:  (N, 2) array of laser scan endpoints in the robot frame.
        grid_map:   occupancy grid map.
        cfg:        configuration object with proposal parameters.

    Returns:
        log_eta:    float, the importance weight factor for the particle.
        mu:        (3,) numpy array, the mean of the candidate poses.
        Sigma:     (3, 3) numpy array, the covariance of the candidate poses.

    log_eta is log sum_j tau_j -- the importance weight factor the filter
    applies to the particle. It is the sum, NOT the likelihood at the sampled
    pose: it approximates the integral p(z | x_prev, m, u), which is what the
    weight is supposed to be.

    Heading mean is circular and the heading residual is wrapped. Sigma is
    regularised so it stays positive definite when tau sits on one
    candidate. All log tau at -inf gives (-inf, x_star, diag(SIGMA_REG)).
    """
    x_star = np.asarray(x_star, dtype=float)
    cands = proposal_candidates(x_star, cfg)                      # (K, 3)
    log_tau = (measurement_log_likelihood(cands, endpoints, grid_map, cfg)
               + motion_model_log_pdf(cands, x_prev, u, cfg.odometry_sigmas))

    log_eta = logsumexp(log_tau)
    if not np.isfinite(log_eta):
        return float('-inf'), x_star.copy(), np.diag(SIGMA_REG).astype(float)

    w = np.exp(log_tau - log_eta)

    mu = np.empty(3)
    mu[:2] = (w[:, None] * cands[:, :2]).sum(axis=0)
    mu[2] = np.arctan2((w * np.sin(cands[:, 2])).sum(),
                       (w * np.cos(cands[:, 2])).sum())

    d = cands - mu
    d[:, 2] = wrap_angle(d[:, 2])
    sigma = np.einsum('i,ij,ik->jk', w, d, d)

    sigma += np.diag(SIGMA_REG)
    sigma = 0.5 * (sigma + sigma.T)
    vals, vecs = np.linalg.eigh(sigma)
    sigma = vecs @ np.diag(np.maximum(vals, SIGMA_EIG_MIN)) @ vecs.T
    sigma = 0.5 * (sigma + sigma.T)
    return float(log_eta), mu, sigma
