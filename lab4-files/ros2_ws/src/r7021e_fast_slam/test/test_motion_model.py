"""The odometry motion model in both forms (Task 2).

The central property: `motion_model_log_pdf` describes the same distribution
`sample_motion_model_odometry` draws from. If they disagree the improved
proposal is fitted to the wrong target and nothing visible says so.
"""
import numpy as np
import pytest

from r7021e_fast_slam.motion_model import (_SIGMA_FLOOR, motion_model_log_pdf,
                                           sample_motion_model_odometry)
from r7021e_fast_slam.utils import odometry_increment, wrap_angle


def _compose(pose, u):
    """Noise-free composition, written out independently of the module."""
    h = pose[2] + u[0]
    return np.array([pose[0] + u[1] * np.cos(h), pose[1] + u[1] * np.sin(h),
                     wrap_angle(h + u[2])])


def test_sampler_with_floored_sigmas_is_noise_free_composition(rng):
    poses = rng.uniform([-3, -3, -np.pi], [3, 3, np.pi], size=(40, 3))
    u = np.array([0.3, 0.4, -0.2])
    out = sample_motion_model_odometry(u, poses, [0.0, 0.0, 0.0], rng)
    expected = np.array([_compose(p, u) for p in poses])
    ## Sigma is floored at 1e-4, so expect agreement to a few 1e-4.
    np.testing.assert_allclose(out[:, :2], expected[:, :2], atol=1e-3)
    assert np.all(np.abs(wrap_angle(out[:, 2] - expected[:, 2])) < 1e-3)


def test_sampler_shape_and_heading_range(rng):
    poses = np.zeros((500, 3))
    poses[:, 2] = np.pi - 1e-3            # right at the seam
    out = sample_motion_model_odometry([0.1, 0.2, 0.1], poses,
                                       [0.1, 0.1, 0.1], rng)
    assert out.shape == (500, 3)
    assert np.all(out[:, 2] >= -np.pi) and np.all(out[:, 2] < np.pi)


def test_sampler_is_reproducible_with_the_same_seed():
    a = sample_motion_model_odometry([0.1, 0.2, 0.1], np.zeros((5, 3)),
                                     [0.02] * 3, np.random.default_rng(3))
    b = sample_motion_model_odometry([0.1, 0.2, 0.1], np.zeros((5, 3)),
                                     [0.02] * 3, np.random.default_rng(3))
    np.testing.assert_array_equal(a, b)


def test_log_pdf_peaks_at_the_noise_free_pose():
    x_prev = np.array([1.0, -2.0, 0.3])
    u = np.array([0.2, 0.5, -0.1])
    s = np.array([0.02, 0.02, 0.02])
    x0 = _compose(x_prev, u)
    rng = np.random.default_rng(0)
    others = x0 + rng.normal(0, 0.02, size=(200, 3))
    lp0 = motion_model_log_pdf(x0[None, :], x_prev, u, s)[0]
    assert lp0 == pytest.approx(-np.sum(np.log(s * np.sqrt(2 * np.pi))))
    assert np.all(motion_model_log_pdf(others, x_prev, u, s) < lp0)


def test_log_pdf_inverts_the_sampler_exactly():
    """The increment the density recovers is the one the sampler used.

    The sampler's own noisy increments are regenerated from the same seed,
    and every sampled pose must map back to them through the density's
    residuals. Checked through utils.odometry_increment, the provided
    decomposition, so the test does not reuse the code under test.
    """
    x_prev = np.array([0.5, 0.5, 2.0])
    u = np.array([0.3, 0.4, -0.2])
    s = np.array([0.03, 0.02, 0.04])
    n = 2000
    out = sample_motion_model_odometry(u, np.tile(x_prev, (n, 1)), s,
                                       np.random.default_rng(9))
    g = np.random.default_rng(9)
    noise = np.stack([g.normal(0, s[0], n), g.normal(0, s[1], n),
                      g.normal(0, s[2], n)], axis=1)
    rec = np.array([odometry_increment(x_prev, p) for p in out])
    np.testing.assert_allclose(wrap_angle(rec[:, 0] - u[0]), noise[:, 0], atol=1e-9)
    np.testing.assert_allclose(rec[:, 1] - u[1], noise[:, 1], atol=1e-9)
    np.testing.assert_allclose(wrap_angle(rec[:, 2] - u[2]), noise[:, 2], atol=1e-9)
    ## And the density of each sample is the Gaussian density of that noise.
    lp = motion_model_log_pdf(out, x_prev, u, s)
    ref = (-0.5 * np.sum((noise / s) ** 2, axis=1)
           - np.sum(np.log(s * np.sqrt(2 * np.pi))))
    np.testing.assert_allclose(lp, ref, atol=1e-8)


def test_log_pdf_tracks_the_empirical_density_of_the_sampler():
    """Histogram a large sample in (x, y, theta) and compare log densities.

    The pose-space density of the sampler is the increment density divided
    by trans, the Jacobian of (rot1, trans, rot2) -> (x, y, theta). The
    lab's log pdf is the increment density, so the comparison adds
    -log(trans) back. With trans = 0.5 m and sd 0.02 m that factor varies
    by only a few percent, which is why leaving it out of the proposal is
    harmless at normal speeds.
    """
    x_prev = np.array([1.0, -2.0, 0.3])
    u = np.array([0.2, 0.5, -0.1])
    s = np.array([0.02, 0.02, 0.02])
    n = 2_000_000
    rng = np.random.default_rng(1)
    pts = sample_motion_model_odometry(u, np.tile(x_prev, (n, 1)), s, rng)
    x0 = _compose(x_prev, u)
    rel = pts - x0
    rel[:, 2] = wrap_angle(rel[:, 2])
    ## Bins of about 0.4 sigma in every direction.
    edges = [np.linspace(-0.04, 0.04, 17), np.linspace(-0.04, 0.04, 17),
             np.linspace(-0.08, 0.08, 17)]
    hist, e = np.histogramdd(rel, bins=edges)
    vol = np.prod([np.diff(ei)[0] for ei in e])
    centres = np.stack(np.meshgrid(*[0.5 * (ei[1:] + ei[:-1]) for ei in e],
                                   indexing='ij'), axis=-1).reshape(-1, 3)
    counts = hist.ravel()
    keep = counts >= 2000
    assert keep.sum() > 30, 'too few populated bins to compare'
    emp = np.log(counts[keep] / (n * vol))
    c = centres[keep] + x0
    trans = np.hypot(c[:, 0] - x_prev[0], c[:, 1] - x_prev[1])
    ana = motion_model_log_pdf(c, x_prev, u, s) - np.log(trans)
    diff = emp - ana
    assert np.median(np.abs(diff)) < 0.05
    assert np.max(np.abs(diff)) < 0.15


def test_log_pdf_wraps_headings_across_the_seam():
    """+179 deg against an expected -179 deg is 2 deg off, not 358."""
    x_prev = np.zeros(3)
    u = np.array([0.0, 0.0, np.deg2rad(-179.0)])
    s = np.array([0.02, 0.02, 0.05])
    across = np.array([[0.0, 0.0, np.deg2rad(179.0)]])    # 2 deg the short way
    same_side = np.array([[0.0, 0.0, np.deg2rad(-177.0)]])  # 2 deg on this side
    a = motion_model_log_pdf(across, x_prev, u, s)[0]
    b = motion_model_log_pdf(same_side, x_prev, u, s)[0]
    assert a == pytest.approx(b, abs=1e-9)
    ## And it is a sensible density, not one 358 degrees out.
    expected = (-0.5 * (np.deg2rad(2.0) / s[2]) ** 2
                - np.sum(np.log(s * np.sqrt(2 * np.pi))))
    assert a == pytest.approx(expected, abs=1e-9)


def test_log_pdf_trans_eps_branch_matches_odometry_increment():
    """A candidate that barely moved puts all its turning into rot2."""
    x_prev = np.array([1.0, 1.0, 0.5])
    cand = np.array([[1.0, 1.0 + 1e-8, 1.2]])
    u = odometry_increment(x_prev, cand[0])
    assert u[0] == 0.0
    s = np.array([0.02, 0.02, 0.02])
    lp = motion_model_log_pdf(cand, x_prev, u, s)[0]
    assert lp == pytest.approx(-np.sum(np.log(s * np.sqrt(2 * np.pi))), abs=1e-6)


def test_log_pdf_floors_sigmas_like_the_sampler():
    lp = motion_model_log_pdf(np.zeros((1, 3)), np.zeros(3), np.zeros(3),
                              [0.0, 0.0, 0.0])
    assert np.isfinite(lp[0])
    assert lp[0] == pytest.approx(-3 * np.log(_SIGMA_FLOOR * np.sqrt(2 * np.pi)))
