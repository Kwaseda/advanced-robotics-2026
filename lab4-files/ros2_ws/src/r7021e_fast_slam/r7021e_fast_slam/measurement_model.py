"""Likelihood field (endpoint) measurement model.

Scores a batch of candidate poses against ONE particle's map. Called N times
per filter step, once per particle, with K candidates each time -- so the K
dimension must be vectorised, never looped.

This is a proper likelihood, not a correlation. Summing occupancy
probabilities at the beam endpoints would be an unnormalized correlation
score; in Grid-FastSLAM 2.0 that quantity enters tau, so it would corrupt the
proposal mean and covariance, not merely the weights.
"""
import numpy as np

from .utils import transform_points


def measurement_log_likelihood(poses, endpoints, grid_map, cfg) -> np.ndarray:
    """log p(z | x, m) for each of K candidate poses.

    Args:
        poses:     (3,) or (K, 3) in the map frame.
        endpoints: (B, 2) beam endpoints in the robot base frame.
        grid_map:  the particle's GridMap.
        cfg:       Config; uses z_hit, z_rand, sigma_hit, max_dist, occ_thresh
                and range_max.

    Returns:
        (K,) array of log-likelihoods for each candidate pose.

    Off-map endpoints score as max_dist (the worst case) rather than being
    dropped, so a pose cannot gain by throwing beams off the map.
    """
    poses = np.atleast_2d(np.asarray(poses, dtype=float))          # (K, 3)
    world = transform_points(poses, endpoints)                    # (K, B, 2)
    cells = grid_map.world_to_grid(world)                         # (K, B, 2)
    inb = grid_map.in_bounds(cells)                               # (K, B)

    ## One distance transform per call: the most expensive step in the system.
    field = grid_map.likelihood_field(cfg.occ_thresh, cfg.max_dist)

    ## cells[..., 0] is the column, and the field is indexed [row, col].
    dist = np.full(inb.shape, float(cfg.max_dist))
    dist[inb] = field[cells[..., 1][inb], cells[..., 0][inb]]

    p = (cfg.z_hit * np.exp(-0.5 * (dist / cfg.sigma_hit) ** 2)
         + cfg.z_rand / cfg.range_max)
    return np.log(p).sum(axis=1)
