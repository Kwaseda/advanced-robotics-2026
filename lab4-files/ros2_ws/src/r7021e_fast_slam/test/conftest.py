"""Shared fixtures for the Lab 4 tests. numpy and PyYAML only, no ROS.

Every test starts from the values in config/params.yaml, so a test run checks
the configuration the node will actually use, not a hand-made one. Tests
that need a small map or a different knob override it on their copy.
"""
import sys
from dataclasses import fields
from pathlib import Path

import numpy as np
import pytest
import yaml

PKG_ROOT = Path(__file__).resolve().parents[1]
if str(PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(PKG_ROOT))

from r7021e_fast_slam.config import Config  # noqa: E402

PARAMS_YAML = PKG_ROOT / 'config' / 'params.yaml'

## LDS-01 / LDS-02 range_max. The node reads it from the first LaserScan;
## anything that drives the filter directly has to set it by hand.
RANGE_MAX = 3.5


def load_params() -> dict:
    with open(PARAMS_YAML) as f:
        return yaml.safe_load(f)['grid_slam_node']['ros__parameters']


def config_from_yaml(**overrides) -> Config:
    """A Config filled from params.yaml the way config_from_node fills it."""
    cfg = Config()
    params = load_params()
    for f in fields(Config):
        if f.name in params:
            value = params[f.name]
            setattr(cfg, f.name,
                    tuple(value) if isinstance(value, list) else value)
    cfg.range_max = RANGE_MAX
    for k, v in overrides.items():
        setattr(cfg, k, v)
    return cfg


@pytest.fixture
def cfg():
    """params.yaml values on a 6 m map, which keeps the tests fast."""
    return config_from_yaml(map_size_m=6.0)


@pytest.fixture
def rng():
    return np.random.default_rng(12345)


## --- a tiny simulated world, for the tests that need a real scan ---

def _box(x0, y0, x1, y1):
    return [((x0, y0), (x1, y0)), ((x1, y0), (x1, y1)),
            ((x1, y1), (x0, y1)), ((x0, y1), (x0, y0))]


## A 4 m square room with two obstacles, so that no two poses see the same
## scan. Without the obstacles the room is symmetric under 90 degree turns.
WORLD = np.array(_box(-2.0, -2.0, 2.0, 2.0)
                 + _box(0.7, -1.2, 1.2, -0.6)
                 + _box(-1.4, 0.5, -1.1, 1.5), dtype=float)


def simulate_scan(pose, n_beams=360, range_max=RANGE_MAX, segments=WORLD):
    """Exact ray casting against line segments. Returns (endpoints, ranges).

    A beam that hits nothing within range_max comes back at range_max, the
    same convention scan_utils.scan_to_endpoints uses for a no-return.
    """
    x, y, th = pose
    ang = th + np.linspace(-np.pi, np.pi, n_beams, endpoint=False)
    d = np.stack([np.cos(ang), np.sin(ang)], axis=1)            # (B, 2)
    p0 = segments[:, 0, :]                                      # (S, 2)
    e = segments[:, 1, :] - p0                                  # (S, 2)
    o = np.array([x, y])
    ## Solve o + t d = p0 + s e for every beam and segment.
    denom = d[:, None, 0] * e[None, :, 1] - d[:, None, 1] * e[None, :, 0]
    w = p0[None, :, :] - o[None, None, :]
    with np.errstate(divide='ignore', invalid='ignore'):
        t = (w[..., 0] * e[None, :, 1] - w[..., 1] * e[None, :, 0]) / denom
        s = (w[..., 0] * d[:, None, 1] - w[..., 1] * d[:, None, 0]) / denom
    valid = (np.abs(denom) > 1e-12) & (t > 1e-9) & (s >= 0) & (s <= 1)
    t = np.where(valid, t, np.inf).min(axis=1)
    ranges = np.minimum(t, range_max)
    rel = ang - th
    endpoints = np.stack([ranges * np.cos(rel), ranges * np.sin(rel)], axis=1)
    return endpoints, ranges


def mapped_grid(cfg, poses):
    """A GridMap built with known poses from simulated scans."""
    from r7021e_fast_slam.grid_map import GridMap
    g = GridMap(cfg)
    for p in poses:
        ep, r = simulate_scan(p)
        g.integrate_scan(np.asarray(p, dtype=float), ep, r, cfg.range_max)
    return g
