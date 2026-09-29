"""The improved proposal (Task 5)."""
import numpy as np
import pytest

from r7021e_fast_slam import proposal
from r7021e_fast_slam.proposal import (SIGMA_REG, improved_proposal,
                                       proposal_candidates)
from r7021e_fast_slam.utils import logsumexp, wrap_angle

from conftest import mapped_grid, simulate_scan


@pytest.mark.parametrize('k, rows', [(27, 27), (26, 8), (3, 1), (1, 1),
                                     (8, 8), (63, 27), (64, 64), (125, 125),
                                     (216, 216), (1000, 1000)])
def test_candidate_count_floors_to_a_cube(cfg, k, rows):
    cfg.num_candidates = k
    assert proposal_candidates(np.zeros(3), cfg).shape == (rows, 3)


def test_lattice_spans_half_the_window_each_side(cfg):
    """proposal_window_* is the FULL width, unlike candidate_grid."""
    cfg.num_candidates = 27
    cfg.proposal_window_xy = 0.10
    cfg.proposal_window_theta = 0.05
    x_star = np.array([1.0, -0.5, 0.3])
    c = proposal_candidates(x_star, cfg)
    for axis, w in [(0, 0.10), (1, 0.10), (2, 0.05)]:
        vals = np.unique(np.round(c[:, axis] - x_star[axis], 12))
        np.testing.assert_allclose(vals, [-w / 2, 0.0, w / 2], atol=1e-12)


def test_single_candidate_is_x_star(cfg):
    cfg.num_candidates = 3
    x_star = np.array([0.4, 0.2, -1.0])
    np.testing.assert_allclose(proposal_candidates(x_star, cfg), [x_star])


def test_candidate_headings_are_wrapped(cfg):
    cfg.num_candidates = 27
    cfg.proposal_window_theta = 0.05     # wide enough to cross the seam
    c = proposal_candidates(np.array([0.0, 0.0, np.pi - 0.01]), cfg)
    assert np.all(c[:, 2] >= -np.pi) and np.all(c[:, 2] < np.pi)
    assert np.any(c[:, 2] < 0)          # at least one crossed the seam


def _realistic(cfg):
    truth = np.array([0.1, -0.2, 0.3])
    g = mapped_grid(cfg, [np.zeros(3), np.array([0.2, -0.3, 0.2])])
    ep, r = simulate_scan(truth)
    ep = ep[r < cfg.range_max - 1e-6]
    x_prev = np.array([0.0, -0.25, 0.3])
    from r7021e_fast_slam.utils import odometry_increment
    u = odometry_increment(x_prev, truth)
    return truth, g, ep, x_prev, u


def test_sigma_is_symmetric_positive_definite_on_a_real_scan(cfg):
    truth, g, ep, x_prev, u = _realistic(cfg)
    log_eta, mu, sigma = improved_proposal(truth + [0.02, -0.01, 0.01],
                                           x_prev, u, ep, g, cfg)
    assert np.isfinite(log_eta)
    assert mu.shape == (3,) and sigma.shape == (3, 3)
    np.testing.assert_allclose(sigma, sigma.T, atol=0)
    np.linalg.cholesky(sigma)
    assert np.all(np.linalg.eigvalsh(sigma) > 0)
    ## The mean lands near the truth, well inside the proposal window.
    assert np.hypot(*(mu[:2] - truth[:2])) < 0.06


def test_all_tau_on_one_candidate_still_factors(cfg, monkeypatch):
    cfg.num_candidates = 27
    x_star = np.array([0.5, 0.5, 1.0])
    cands = proposal_candidates(x_star, cfg)
    winner = 5
    fake = np.full(len(cands), -1e6)
    fake[winner] = 0.0
    monkeypatch.setattr(proposal, 'measurement_log_likelihood',
                        lambda *a, **k: fake.copy())
    monkeypatch.setattr(proposal, 'motion_model_log_pdf',
                        lambda *a, **k: np.zeros(len(cands)))
    log_eta, mu, sigma = improved_proposal(x_star, x_star, np.zeros(3),
                                           np.zeros((0, 2)), None, cfg)
    np.testing.assert_allclose(mu, cands[winner], atol=1e-12)
    np.linalg.cholesky(sigma)
    np.testing.assert_allclose(np.diag(sigma), SIGMA_REG, rtol=1e-6)


def test_circular_mean_straddling_pi_is_near_pi_not_zero(cfg, monkeypatch):
    cfg.num_candidates = 27
    cfg.proposal_window_theta = 0.05
    x_star = np.array([0.0, 0.0, np.pi - 0.01])
    n = len(proposal_candidates(x_star, cfg))
    ## Uniform tau: the mean must come back to x_star exactly.
    monkeypatch.setattr(proposal, 'measurement_log_likelihood',
                        lambda *a, **k: np.zeros(n))
    monkeypatch.setattr(proposal, 'motion_model_log_pdf',
                        lambda *a, **k: np.zeros(n))
    _, mu, sigma = improved_proposal(x_star, x_star, np.zeros(3),
                                     np.zeros((0, 2)), None, cfg)
    assert abs(wrap_angle(mu[2] - x_star[2])) < 1e-9
    assert abs(mu[2]) > 3.0
    ## Heading variance of three equally weighted offsets -w/2, 0, +w/2,
    ## plus the floor. Unwrapped residuals would give about pi^2 here.
    expected = 2.0 / 3.0 * 0.025 ** 2 + SIGMA_REG[2]
    assert sigma[2, 2] == pytest.approx(expected, rel=1e-6)


def test_log_eta_is_the_sum_over_candidates(cfg, monkeypatch):
    """The one to check by eye: logsumexp over ALL candidates of both terms."""
    cfg.num_candidates = 27
    rng = np.random.default_rng(4)
    meas = rng.normal(-300.0, 3.0, 27)
    mot = rng.normal(2.0, 1.0, 27)
    monkeypatch.setattr(proposal, 'measurement_log_likelihood',
                        lambda *a, **k: meas.copy())
    monkeypatch.setattr(proposal, 'motion_model_log_pdf',
                        lambda *a, **k: mot.copy())
    log_eta, _, _ = improved_proposal(np.zeros(3), np.zeros(3), np.zeros(3),
                                      np.zeros((0, 2)), None, cfg)
    assert log_eta == pytest.approx(logsumexp(meas + mot), abs=1e-9)
    ## Strictly more than the best single candidate, which is what a
    ## likelihood-at-one-pose weight would give.
    assert log_eta > np.max(meas + mot)


def test_all_minus_inf_returns_defined_values(cfg, monkeypatch):
    cfg.num_candidates = 27
    monkeypatch.setattr(proposal, 'measurement_log_likelihood',
                        lambda *a, **k: np.full(27, -np.inf))
    monkeypatch.setattr(proposal, 'motion_model_log_pdf',
                        lambda *a, **k: np.zeros(27))
    x_star = np.array([1.0, 2.0, 0.5])
    log_eta, mu, sigma = improved_proposal(x_star, x_star, np.zeros(3),
                                           np.zeros((0, 2)), None, cfg)
    assert log_eta == -np.inf
    np.testing.assert_array_equal(mu, x_star)
    np.testing.assert_array_equal(sigma, np.diag(SIGMA_REG))
    assert not np.any(np.isnan(sigma))


def test_improved_proposal_calls_the_distance_transform_once(cfg, monkeypatch):
    truth, g, ep, x_prev, u = _realistic(cfg)
    calls = []
    real = g.likelihood_field
    monkeypatch.setattr(g, 'likelihood_field',
                        lambda *a: calls.append(1) or real(*a))
    improved_proposal(truth, x_prev, u, ep, g, cfg)
    assert len(calls) == 1
