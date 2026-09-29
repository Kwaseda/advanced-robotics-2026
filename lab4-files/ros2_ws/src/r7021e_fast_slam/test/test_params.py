"""params.yaml against the agreed values, and the startup checks.

The first test reads the file statically, so it also covers mapping-only
runs (use_measurement_update: false), which never construct GridFastSLAM
and so never reach its startup checks.
"""
import numpy as np
import pytest

from r7021e_fast_slam.config import Config
from r7021e_fast_slam.rbpf import GridFastSLAM, check_config

from conftest import config_from_yaml, load_params

## The tuned values. Change a value here and in params.yaml together.
TUNED = {
    'num_particles': 10,
    'log_odds_limit': 5.0, 'p_free': 0.4, 'p_occ': 0.7,
    'map_resolution': 0.05, 'map_size_m': 20.0, 'hit_window_cells': 2.0,
    'use_clamping': True,
    'odometry_sigmas': [0.02, 0.02, 0.02],
    'odom_noise_sigma': [0.003, 0.003, 0.003],
    'sigma_hit': 0.05, 'max_dist': 0.15, 'z_hit': 0.8, 'z_rand': 0.2,
    'occ_thresh': 0.6,
    'resample_threshold': 0.5, 'resample_every_step': False,
    'num_candidates': 27, 'proposal_window_xy': 0.10,
    'proposal_window_theta': 0.05, 'scan_match_window_xy': 0.20,
    'scan_match_window_theta': 0.10, 'match_score_min': 0.55,
    'update_max_rate': 10.0, 'update_min_dist': 0.05,
    'update_min_angle': 0.0175, 'scan_queue_depth': 1,
}


def test_params_yaml_matches_the_tuned_values():
    params = load_params()
    for key, value in TUNED.items():
        assert params[key] == pytest.approx(value), key


def test_every_params_key_is_a_config_field():
    """A typo in params.yaml would otherwise be silently ignored."""
    names = set(Config.__dataclass_fields__)
    assert set(load_params()) <= names


def test_shipped_params_pass_the_startup_checks():
    check_config(config_from_yaml())


def test_derived_values_have_the_right_sign():
    cfg = config_from_yaml()
    assert cfg.l_occ > 0 > cfg.l_free
    assert cfg.log_odds_limit > np.log(cfg.occ_thresh / (1 - cfg.occ_thresh))
    assert cfg.max_dist >= 2 * cfg.sigma_hit


@pytest.mark.parametrize('override, fragment', [
    ({'p_occ': 0.5}, 'p_occ'),
    ({'p_free': 0.5}, 'p_free'),
    ({'log_odds_limit': 0.4}, 'log_odds_limit'),
    ({'log_odds_limit': 0.0}, 'log_odds_limit'),
    ({'num_candidates': 3}, 'num_candidates'),
    ({'num_candidates': 26}, 'num_candidates'),
    ({'num_candidates': 8}, 'num_candidates'),
    ({'scan_match_window_xy': 0.0}, 'scan_match_window'),
    ({'scan_match_window_theta': 0.0}, 'scan_match_window'),
    ({'odometry_sigmas': (0.02, 0.0, 0.02)}, 'odometry_sigmas'),
    ({'max_dist': 0.09}, 'max_dist'),
    ({'sigma_hit': 0.10, 'max_dist': 0.10}, 'max_dist'),   # the shipped pair
])
def test_bad_values_fail_at_startup(override, fragment):
    cfg = config_from_yaml(**override)
    with pytest.raises(ValueError, match=fragment):
        GridFastSLAM(cfg, np.random.default_rng(0))


@pytest.mark.parametrize('k', [27, 64, 125, 1000])
def test_cubes_pass(k):
    check_config(config_from_yaml(num_candidates=k))


def test_clamp_check_only_applies_with_clamping_on():
    check_config(config_from_yaml(use_clamping=False, log_odds_limit=0.0))


def test_hardware_params_differ_only_where_intended():
    import yaml
    from conftest import PKG_ROOT
    with open(PKG_ROOT / 'config' / 'params_hardware.yaml') as f:
        hw = yaml.safe_load(f)['grid_slam_node']['ros__parameters']
    sim = load_params()
    assert set(hw) == set(sim)
    differ = {k for k in sim if hw[k] != sim[k]}
    assert differ == {'odom_noise_sigma', 'odometry_sigmas', 'run_name'}
    assert hw['odom_noise_sigma'] == [0.0, 0.0, 0.0]
    cfg = config_from_yaml()
    cfg.odometry_sigmas = tuple(hw['odometry_sigmas'])
    check_config(cfg)
