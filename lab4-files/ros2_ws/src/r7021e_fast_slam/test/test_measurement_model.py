"""The likelihood field measurement model (Task 3)."""
import numpy as np
import pytest

from r7021e_fast_slam.grid_map import GridMap
from r7021e_fast_slam.measurement_model import measurement_log_likelihood

from conftest import mapped_grid, simulate_scan


def _one_cell_map(cfg, row, col):
    g = GridMap(cfg)
    g.log_odds[row, col] = cfg.log_odds_limit
    return g


def test_endpoint_on_the_occupied_cell_beats_ten_cells_away(cfg):
    g = _one_cell_map(cfg, 60, 60)
    ## Centre of cell (col 60, row 60) in world metres.
    cx, cy = g.origin + (np.array([60, 60]) + 0.5) * g.res
    ep = np.array([[1.0, 0.0]])                 # one beam, 1 m straight ahead
    on = np.array([cx - 1.0, cy, 0.0])
    off = np.array([cx - 1.0 + 10 * g.res, cy, 0.0])
    ll = measurement_log_likelihood(np.stack([on, off]), ep, g, cfg)
    assert ll[0] > ll[1]
    ## On the cell: distance 0, the best a beam can do.
    best = np.log(cfg.z_hit + cfg.z_rand / cfg.range_max)
    assert ll[0] == pytest.approx(best)
    ## Ten cells away is beyond max_dist: the worst a beam can do.
    worst = np.log(cfg.z_hit * np.exp(-0.5 * (cfg.max_dist / cfg.sigma_hit) ** 2)
                   + cfg.z_rand / cfg.range_max)
    assert ll[1] == pytest.approx(worst, rel=1e-6)


def test_score_falls_with_distance_inside_max_dist(cfg):
    g = _one_cell_map(cfg, 60, 60)
    cx, cy = g.origin + (np.array([60, 60]) + 0.5) * g.res
    ep = np.array([[1.0, 0.0]])
    poses = np.array([[cx - 1.0 + k * g.res, cy, 0.0] for k in range(4)])
    ll = measurement_log_likelihood(poses, ep, g, cfg)
    assert np.all(np.diff(ll) < 0)


def test_shapes_for_single_and_batched_poses(cfg):
    g = mapped_grid(cfg, [np.zeros(3)])
    ep, _ = simulate_scan(np.zeros(3))
    one = measurement_log_likelihood(np.zeros(3), ep, g, cfg)
    many = measurement_log_likelihood(np.zeros((5, 3)), ep, g, cfg)
    assert one.shape == (1,)
    assert many.shape == (5,)
    assert many[0] == pytest.approx(one[0])


def test_out_of_bounds_endpoints_score_worst_and_never_nan(cfg):
    g = mapped_grid(cfg, [np.zeros(3)])
    ep = np.array([[100.0, 0.0], [0.0, -250.0], [1e6, 1e6]])
    ll = measurement_log_likelihood(np.zeros((2, 3)), ep, g, cfg)
    assert np.all(np.isfinite(ll))
    worst = np.log(cfg.z_hit * np.exp(-0.5 * (cfg.max_dist / cfg.sigma_hit) ** 2)
                   + cfg.z_rand / cfg.range_max)
    assert ll == pytest.approx(np.full(2, 3 * worst))


def test_empty_map_scores_every_pose_the_same(cfg):
    g = GridMap(cfg)
    ep, _ = simulate_scan(np.zeros(3))
    ll = measurement_log_likelihood(np.random.default_rng(0).normal(size=(6, 3)),
                                    ep, g, cfg)
    ## Equal up to float32 rounding: the given likelihood_field stores
    ## max_dist as float32, off-map beams use the float64 value.
    assert np.ptp(ll) < 1e-4


def test_true_pose_scores_best_on_a_mapped_room(cfg):
    truth = np.array([0.2, 0.1, 0.4])
    g = mapped_grid(cfg, [truth])
    ep, r = simulate_scan(truth)
    ep = ep[r < cfg.range_max - 1e-6]
    offsets = np.array([[0, 0, 0], [0.1, 0, 0], [0, -0.1, 0], [0, 0, 0.1],
                        [0.05, 0.05, -0.05]])
    ll = measurement_log_likelihood(truth + offsets, ep, g, cfg)
    assert np.argmax(ll) == 0


def test_calls_likelihood_field_once_per_call(cfg, monkeypatch):
    g = mapped_grid(cfg, [np.zeros(3)])
    calls = []
    real = g.likelihood_field
    monkeypatch.setattr(g, 'likelihood_field',
                        lambda *a: calls.append(1) or real(*a))
    ep, _ = simulate_scan(np.zeros(3))
    measurement_log_likelihood(np.zeros((27, 3)), ep, g, cfg)
    assert len(calls) == 1
