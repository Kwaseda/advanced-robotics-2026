"""Grid-FastSLAM 2.0: the filter itself.

The algorithm of hello-slam's `4_grid_based_slam.ipynb`. Each particle carries
a pose hypothesis and its own occupancy grid. Per particle and per step:
predict from odometry (3), scan-match against that particle's OWN map (5),
build a Gaussian proposal around the match (6-10), sample the new pose (11),
update the weight with the proposal's normalizer (12), and only then integrate
the scan (22). Then normalize (24), compute N_eff (25) and resample
selectively (26-27).

No rclpy in here. The ROS node feeds this class odometry increments and beam
endpoints and does its own publishing.

Occupancy mapping without a filter -- `use_measurement_update: false` -- does
NOT go through this class at all. The node runs `MappingOnlyFilter` for that,
so the very first task works before a single line of this file exists.
"""
from dataclasses import dataclass
from time import perf_counter
from typing import List

import numpy as np

from .evaluation import StepInfo
from .grid_map import GridMap
from .measurement_model import measurement_log_likelihood
from .motion_model import sample_motion_model_odometry
from .proposal import improved_proposal
from .resampling import effective_sample_size, systematic_resample
from .scan_matcher import match as scan_match
from .utils import logit, logsumexp, wrap_angle


def _fail(what: str, why: str):
    raise ValueError(f'params.yaml: {what}. {why}')


def check_config(cfg):
    """Refuse configurations that run but produce a blank map or identical
    particles. range_max is checked per step, since the node reads it from
    the first scan."""
    if not cfg.p_occ > 0.5:
        _fail(f'p_occ = {cfg.p_occ} must be above 0.5',
              'At or below 0.5, logit(p_occ) is not positive, so a hit '
              'pushes its cell toward free and nothing is ever occupied.')
    if not cfg.p_free < 0.5:
        _fail(f'p_free = {cfg.p_free} must be below 0.5',
              'At or above 0.5, a beam passing through a cell pushes it '
              'toward occupied.')
    l_thresh = logit(cfg.occ_thresh)
    if cfg.use_clamping and not cfg.log_odds_limit > l_thresh:
        _fail(f'log_odds_limit = {cfg.log_odds_limit} must exceed '
              f'logit(occ_thresh) = {l_thresh:.3f}',
              'Otherwise the clamp stops every cell below the occupancy '
              'threshold, the likelihood field is max_dist everywhere and '
              'every pose scores the same.')
    k = int(cfg.num_candidates)
    n = round(k ** (1.0 / 3.0))
    if k < 27 or n ** 3 != k:
        _fail(f'num_candidates = {k} must be a perfect cube, 27 or more',
              'K is floored to a cube, and a lattice with one point per axis '
              'is a point estimate, not a distribution.')
    if not (cfg.scan_match_window_xy > 0 and cfg.scan_match_window_theta > 0):
        _fail('scan_match_window_xy and scan_match_window_theta must be > 0',
              'The matcher divides by a step derived from the window.')
    if not all(float(s) > 0.0 for s in cfg.odometry_sigmas):
        _fail(f'odometry_sigmas = {list(cfg.odometry_sigmas)} must all be > 0',
              'Zeros make every particle identical and pin the scan '
              'matcher to the odometry through its derived prior.')
    if not cfg.max_dist >= 2.0 * cfg.sigma_hit:
        _fail(f'max_dist = {cfg.max_dist} must be at least 2 * sigma_hit = '
              f'{2.0 * cfg.sigma_hit}',
              'The likelihood field is clipped at max_dist, and clipping '
              'inside two sigma flattens the peak the proposal is fitted to.')


def check_range_max(cfg):
    """range_max > 0. The z_rand / range_max term divides by it."""
    if not cfg.range_max > 0.0:
        _fail(f'range_max = {cfg.range_max} must be > 0',
              'The node reads it from LaserScan.range_max on the first scan; '
              'code that drives the filter directly must set it by hand.')


@dataclass
class Particle:
    """A single particle in the Grid-FastSLAM 2.0 filter.
    Holds the robot's pose, the particle's log-weight, and its own occupancy grid.
    """
    pose: np.ndarray
    log_weight: float
    grid: GridMap


class GridFastSLAM:
    def __init__(self, cfg, rng):
        """Create N particles, all at the origin with equal log-weights.

        Each particle gets its OWN GridMap; they must never share one, or
        resampling would corrupt every hypothesis at once.
        """
        check_config(cfg)
        self.cfg = cfg
        self.rng = rng
        n = int(cfg.num_particles)
        ## Initialise the particle set with N particles at the origin, each with its own GridMap.
        self.particles: List[Particle] = [
            Particle(np.zeros(3), -np.log(n), GridMap(cfg)) for _ in range(n)
        ]

    def best_particle(self) -> Particle:
        """The heaviest particle -- the hypothesis published as the estimate."""
        return self.particles[int(np.argmax([p.log_weight
                                             for p in self.particles]))]

    def step(self, u, endpoints, ranges) -> StepInfo:
        """Run one filter update over the whole particle set.

        The map is integrated LAST on purpose. Every likelihood evaluated
        during a step must see m_{t-1}: scoring a scan against a map that
        already contains it is a different, far more optimistic estimator.

        `use_improved_proposal=False` samples from the motion model and
        weights by the measurement likelihood alone -- FastSLAM 1.0, the
        baseline the improved proposal is measured against.

        Args:
            u:         (3,) odometry increment [d_rot1, d_trans, d_rot2].
            endpoints: (B, 2) beam endpoints in the robot base frame.
            ranges:    (B,) beam ranges, used to tell a hit from a max-range
                    return.

        Returns a StepInfo carrying N_eff, whether resampling fired, the
        scan-match fallback count, the per-stage timings and the index of the
        best particle. The particle set is updated in place; read it back
        through `self.particles` and `best_particle()`.

        """
        cfg = self.cfg
        check_range_max(cfg)
        u = np.asarray(u, dtype=float)
        endpoints = np.asarray(endpoints, dtype=float)
        ranges = np.asarray(ranges, dtype=float)
        timings = {'scan_match': 0.0, 'likelihood': 0.0, 'map_integrate': 0.0}
        n_fallback = 0

        ## Max-range returns map free space but never score a pose.
        hits = ranges < cfg.range_max - 1e-6
        ep_hit = endpoints[hits]
        ep_match = ep_hit[::max(int(cfg.scan_match_stride), 1)]
        ep_w = ep_hit[::max(int(cfg.likelihood_stride), 1)]

        for p in self.particles:
            ## (3) Odometry prediction.
            x_bar = sample_motion_model_odometry(
                u, p.pose[None, :], cfg.odometry_sigmas, self.rng)[0]

            ## (5) Scan match against this particle's own map, m_{t-1}.
            if cfg.use_improved_proposal:
                t0 = perf_counter()
                x_star, score = scan_match(x_bar, p.grid, ep_match, cfg)
                timings['scan_match'] += perf_counter() - t0
            else:
                x_star, score = x_bar, -np.inf

            t0 = perf_counter()
            if score >= cfg.match_score_min:
                ## (6-10) Gaussian proposal around the match, and eta.
                log_eta, mu, sigma = improved_proposal(
                    x_star, p.pose, u, ep_w, p.grid, cfg)
                ## (11) Sample the new pose from it.
                x_new = self.rng.multivariate_normal(mu, sigma)
                x_new[2] = wrap_angle(x_new[2])
                ## (12) The weight update is the proposal's normalizer.
                p.log_weight += log_eta
            else:
                ## Untrusted match: plain FastSLAM 1.0 step for this particle.
                x_new = x_bar
                p.log_weight += float(measurement_log_likelihood(
                    x_bar, ep_w, p.grid, cfg)[0])
                if cfg.use_improved_proposal:
                    n_fallback += 1
            timings['likelihood'] += perf_counter() - t0

            p.pose = np.asarray(x_new, dtype=float)

            ## (22) Last, so every likelihood above saw m_{t-1}.
            t0 = perf_counter()
            p.grid.integrate_scan(p.pose, endpoints, ranges, cfg.range_max)
            timings['map_integrate'] += perf_counter() - t0

        ## (24-25) Normalize, then N_eff from the normalized weights.
        weights, n_eff = self._normalize()
        ## Before resampling; afterwards every weight is 1/N.
        best_index = int(np.argmax(weights))
        ## (26-27) Resample if the set has degenerated.
        resampled = self._maybe_resample(weights, n_eff)

        ## Pre-resample n_eff: the value that made the decision.
        return StepInfo(n_eff=float(n_eff), resampled=bool(resampled),
                        n_fallback=n_fallback, timings=timings,
                        best_index=best_index)

    def _normalize(self):
        """Normalize the particle weights and compute the effective sample size.

        Returns:
        
            weights : np.ndarray
                The normalized weights of the particles.
            n_eff : float
                The effective sample size.
        """
        log_w = np.array([p.log_weight for p in self.particles], dtype=float)
        log_w -= logsumexp(log_w)
        for p, lw in zip(self.particles, log_w):
            p.log_weight = float(lw)
        weights = np.exp(log_w)
        return weights, effective_sample_size(weights)

    def _maybe_resample(self, weights, n_eff) -> bool:
        """Resample the particle set if the diversity has collapsed.

        Args:
            weights: np.ndarray
                The normalized particle weights.
            n_eff: float
                The effective sample size before resampling.

        Returns:
            bool: True when a resample step was performed, otherwise False.
        """
        cfg = self.cfg
        n = len(self.particles)
        due = cfg.resample_every_step or n_eff < cfg.resample_threshold * n
        if not due:
            return False

        idx = systematic_resample(weights, self.rng)
        ## Every survivor gets a full grid copy, including the ones selected
        ## exactly once. This is the straightforward reading of the algorithm
        ## and it is what makes a large particle set expensive.
        self.particles = [
            Particle(self.particles[i].pose.copy(), -np.log(n),
                     self.particles[i].grid.copy())
            for i in idx
        ]
        return True
