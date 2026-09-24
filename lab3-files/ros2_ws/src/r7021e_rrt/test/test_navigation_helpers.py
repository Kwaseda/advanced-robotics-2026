"""Desk tests for the path shaping the navigation node publishes through.

No ROS import anywhere, which is the point: `densify_path` lives in `rrt_star.py`
rather than in `navigation_node.py` precisely so it can be tested here. An
earlier version of this file imported the node itself, which pulled in rclpy and
quietly made the suite depend on a sourced ROS installation.
"""

from __future__ import annotations

import math

import pytest

from r7021e_rrt.rrt_star import densify_path


def test_densify_bounds_the_gap_between_waypoints() -> None:
    """The follower drops a waypoint inside its 0.20 m look-ahead and steers at
    the next one, so a gap wider than the look-ahead is a corner it can cut."""
    dense = densify_path([(0.0, 0.0), (0.3, 0.0), (0.3, 0.3)], 0.10)
    for a, b in zip(dense, dense[1:]):
        assert math.dist(a, b) <= 0.10 + 1e-9


def test_densify_keeps_the_polyline_it_was_given() -> None:
    """Resampling must not move the path the planner checked."""
    coarse = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)]
    dense = densify_path(coarse, 0.1)
    assert dense[0] == coarse[0]
    assert dense[-1] == pytest.approx(coarse[-1])
    for point in coarse:
        assert any(math.dist(point, d) < 1e-9 for d in dense)


def test_densify_is_a_no_op_on_a_single_waypoint() -> None:
    """The park path is one waypoint at the robot's own pose."""
    assert densify_path([(1.0, 2.0)], 0.1) == [(1.0, 2.0)]
    assert densify_path([], 0.1) == []


def test_densify_with_no_spacing_leaves_the_path_alone() -> None:
    coarse = [(0.0, 0.0), (1.0, 0.0)]
    assert densify_path(coarse, 0.0) == coarse
