"""GridFastSLAM.step (Task 6), including the three ordering traps.

Each trap is a way to get a convincing map from a broken filter. Each test
below fails if the ordering is wrong, whatever the map looks like.
"""
import numpy as np
import pytest

from r7021e_fast_slam import proposal, rbpf
from r7021e_fast_slam.grid_map import GridMap
from r7021e_fast_slam.rbpf import GridFastSLAM
from r7021e_fast_slam.utils import odometry_increment

from conftest import mapped_grid, simulate_scan

PATH = [np.array([0.0, 0.0, 0.0]), np.array([0.08, 0.0, 0.05]),
        np.array([0.16, 0.01, 0.10])]


def _filter_on_a_known_map(cfg, n, seed=0):
    """N particles at the origin, each holding its own copy of a real map."""
    cfg.num_particles = n
    f = GridFastSLAM(cfg, np.random.default_rng(seed))
    base = mapped_grid(cfg, [PATH[0]])
    for p in f.particles:
        p.grid = base.copy()
    return f


def _step_input(i=1):
    u = odometry_increment(PATH[i - 1], PATH[i])
    ep, r = simulate_scan(PATH[i])
    return u, ep, r


## --- TRAP 1: the map is integrated last ---------------------------------

@pytest.mark.parametrize('match_score_min', [0.0, 1.1],
                         ids=['improved-branch', 'fallback-branch'])
def test_trap1_every_likelihood_sees_the_previous_map(cfg, monkeypatch,
                                                      match_score_min):
    cfg.match_score_min = match_score_min
    f = _filter_on_a_known_map(cfg, 3)
    before = {id(p.grid): p.grid.log_odds.copy() for p in f.particles}
    events = []

    def spy(name, fn):
        def wrapped(*args, **kwargs):
            grid = next(a for a in args if isinstance(a, GridMap))
            ## The strong form of the trap: the map this call sees must be
            ## bit-for-bit the map from before the step.
            assert np.array_equal(grid.log_odds, before[id(grid)]), \
                f'{name} saw a map that already contains this scan'
            events.append((name, id(grid)))
            return fn(*args, **kwargs)
        return wrapped

    monkeypatch.setattr(rbpf, 'scan_match', spy('match', rbpf.scan_match))
    monkeypatch.setattr(rbpf, 'measurement_log_likelihood',
                        spy('lik', rbpf.measurement_log_likelihood))
    monkeypatch.setattr(proposal, 'measurement_log_likelihood',
                        spy('lik', proposal.measurement_log_likelihood))
    real_integrate = GridMap.integrate_scan

    def integrate(self, *a, **k):
        events.append(('integrate', id(self)))
        return real_integrate(self, *a, **k)
    monkeypatch.setattr(GridMap, 'integrate_scan', integrate)

    f.step(*_step_input())

    for gid in before:
        mine = [(i, name) for i, (name, g) in enumerate(events) if g == gid]
        integ = [i for i, name in mine if name == 'integrate']
        reads = [i for i, name in mine if name != 'integrate']
        assert len(integ) == 1, 'each particle integrates exactly once'
        assert reads, 'the likelihood was never evaluated'
        assert max(reads) < integ[0]


## --- TRAP 2: resampling deep-copies the grid ----------------------------

def test_trap2_resampled_particles_own_their_grids(cfg):
    f = _filter_on_a_known_map(cfg, 5)
    ancestor_grid = f.particles[2].grid
    weights = np.zeros(5)
    weights[2] = 1.0                  # every survivor is a copy of particle 2
    assert f._maybe_resample(weights, n_eff=1.0)

    grids = [p.grid for p in f.particles]
    assert len({id(g) for g in grids}) == 5
    assert all(g is not ancestor_grid for g in grids)
    assert len({id(g.log_odds) for g in grids}) == 5

    snapshot = [g.log_odds.copy() for g in grids]
    grids[0].log_odds[10:20, 10:20] += 3.0
    for g, s in zip(grids[1:], snapshot[1:]):
        np.testing.assert_array_equal(g.log_odds, s)
    ## Poses are copies too.
    f.particles[0].pose[0] += 1.0
    assert f.particles[1].pose[0] != f.particles[0].pose[0]


## --- TRAP 3: the reported N_eff is the pre-resample value ---------------

def test_trap3_reported_neff_is_pre_resample(cfg):
    n = 4
    f = _filter_on_a_known_map(cfg, n)
    ## Make particle 2 overwhelmingly heavy before the step, so N_eff after
    ## the update is about 1 and resampling must fire.
    for i, p in enumerate(f.particles):
        p.log_weight = 0.0 if i == 2 else -500.0
    info = f.step(*_step_input())

    assert info.resampled
    assert info.n_eff < cfg.resample_threshold * n
    assert info.n_eff != pytest.approx(n)
    assert info.n_eff == pytest.approx(1.0, abs=1e-6)
    ## After resampling every weight is 1/N, so N_eff is now N. The report
    ## must not be that number.
    w = np.exp([p.log_weight for p in f.particles])
    assert 1.0 / np.sum(w ** 2) == pytest.approx(n)
    ## best_index was read before resampling, so it names particle 2, not 0.
    assert info.best_index == 2


## --- the rest of step() -------------------------------------------------

def test_no_resample_while_neff_is_above_the_threshold(cfg):
    """The policy, not the physics: N_eff is at least 1, so a threshold of
    0.2 * 4 = 0.8 can never be crossed and resampling must not fire.

    An earlier version of this test assumed identical particles on an
    identical map would stay near equal weight. They do not: with
    sigma_hit 0.05 the likelihood peak is about a cell wide, the 27-point
    lattice is spaced a cell apart, and eta depends on where each
    particle's lattice happens to fall against that peak. One step took
    N_eff from 4 to 1.5: a property of the model, not of the code.
    """
    cfg.resample_threshold = 0.2
    f = _filter_on_a_known_map(cfg, 4)
    info = f.step(*_step_input())
    assert info.n_eff >= 1.0 - 1e-9
    assert not info.resampled
    assert len({round(p.log_weight, 12) for p in f.particles}) > 1


def test_resample_every_step_forces_it(cfg):
    cfg.resample_every_step = True
    cfg.resample_threshold = 0.0
    f = _filter_on_a_known_map(cfg, 4)
    assert f.step(*_step_input()).resampled


def test_fallback_counted_only_for_the_improved_proposal(cfg):
    cfg.match_score_min = 1.1                 # nothing is ever trusted
    f = _filter_on_a_known_map(cfg, 3)
    assert f.step(*_step_input()).n_fallback == 3

    cfg.use_improved_proposal = False         # FastSLAM 1.0 baseline
    f = _filter_on_a_known_map(cfg, 3)
    info = f.step(*_step_input())
    assert info.n_fallback == 0
    assert info.timings['scan_match'] == 0.0


def test_improved_branch_taken_on_a_known_map(cfg):
    cfg.match_score_min = 0.55
    f = _filter_on_a_known_map(cfg, 3)
    info = f.step(*_step_input())
    assert info.n_fallback == 0
    assert all(info.timings[k] > 0.0 for k in
               ('scan_match', 'likelihood', 'map_integrate'))


def test_empty_map_falls_back(cfg):
    """An unexplored map scores 0.5 everywhere, below match_score_min."""
    cfg.num_particles = 3
    f = GridFastSLAM(cfg, np.random.default_rng(0))
    info = f.step(*_step_input())
    assert info.n_fallback == 3


def test_max_range_beams_never_score_a_pose(cfg, monkeypatch):
    seen = []
    real = rbpf.measurement_log_likelihood

    def spy(poses, endpoints, grid, c):
        seen.append(np.asarray(endpoints).copy())
        return real(poses, endpoints, grid, c)
    monkeypatch.setattr(rbpf, 'measurement_log_likelihood', spy)
    cfg.match_score_min = 1.1
    f = _filter_on_a_known_map(cfg, 1)
    u, ep, r = _step_input()
    r = r.copy()
    r[:40] = cfg.range_max                    # forty no-returns
    ep[:40] = ep[:40] / np.linalg.norm(ep[:40], axis=1)[:, None] * cfg.range_max
    f.step(u, ep, r)
    assert seen[0].shape[0] == int(np.sum(r < cfg.range_max - 1e-6))


def test_step_rejects_a_missing_range_max(cfg):
    f = _filter_on_a_known_map(cfg, 2)
    cfg.range_max = 0.0
    with pytest.raises(ValueError, match='range_max'):
        f.step(*_step_input())


def test_filter_tracks_a_drive_better_than_its_odometry(cfg):
    """End to end on the simulated room: 40 steps, drifting odometry.

    Not a proof of anything statistical, one seed. It exists to catch a
    filter that runs cleanly and does nothing, which every unit test above
    could miss.
    """
    cfg.num_particles = 10
    rng = np.random.default_rng(7)
    f = GridFastSLAM(cfg, np.random.default_rng(8))

    ## An ellipse that starts at the filter's origin, heading along +x, and
    ## stays clear of both obstacles.
    t = np.linspace(0.0, 2.0 * np.pi, 41)
    truth = np.stack([0.6 * np.sin(t), 0.4 * (1.0 - np.cos(t)),
                      np.zeros_like(t)], axis=1)
    truth[:, 2] = np.arctan2(0.4 * np.sin(t), 0.6 * np.cos(t))

    odom = np.zeros(3)
    est_err, odom_err = [], []
    for i in range(1, len(truth)):
        u_true = odometry_increment(truth[i - 1], truth[i])
        u_noisy = u_true + rng.normal(0.0, [0.02, 0.01, 0.02])
        h = odom[2] + u_noisy[0]
        odom = np.array([odom[0] + u_noisy[1] * np.cos(h),
                         odom[1] + u_noisy[1] * np.sin(h), h + u_noisy[2]])
        ep, r = simulate_scan(truth[i])
        f.step(u_noisy, ep, r)
        est_err.append(np.hypot(*(f.best_particle().pose[:2] - truth[i, :2])))
        odom_err.append(np.hypot(*(odom[:2] - truth[i, :2])))
    rmse = np.sqrt(np.mean(np.square(est_err)))
    assert rmse < 0.5 * np.sqrt(np.mean(np.square(odom_err)))
    ## Two cells. The error is not aligned, and whatever the pose error is
    ## before the map exists freezes into the map frame, so a few cm of
    ## offset is expected even from a correct filter.
    assert rmse < 0.10
