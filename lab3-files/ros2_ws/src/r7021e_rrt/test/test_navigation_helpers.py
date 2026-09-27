"""Desk tests for the path shaping the navigation node publishes through.

No ROS import anywhere, which is the point: `densify_path` lives in `rrt_star.py`
rather than in `navigation_node.py` precisely so it can be tested here. An
earlier version of this file imported the node itself, which pulled in rclpy and
quietly made the suite depend on a sourced ROS installation.
"""

from __future__ import annotations

import math

import pytest

from r7021e_rrt.grid import OccupancyMap, PlanningGrid
from r7021e_rrt.rrt_star import centre_path, densify_path, look_around_target


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


class TestLookAroundTarget:
    """The carrot that turns the robot on the spot rather than moving it."""

    def test_bearing_is_the_step_off_the_nose(self):
        # The whole mechanism is that the follower sees a constant heading error
        # equal to the step, which is what keeps its linear velocity clamped.
        for yaw in (0.0, 1.0, -2.5, 3.0):
            x, y = look_around_target((0.0, 0.0), yaw, 0.05, 2.0)
            assert math.isclose(math.atan2(y, x), _wrap(yaw + 2.0), abs_tol=1e-9)

    def test_radius_is_respected_and_small(self):
        for yaw in (0.0, 0.7, -1.9):
            target = look_around_target((1.0, -2.0), yaw, 0.05, 2.0)
            assert math.isclose(math.dist(target, (1.0, -2.0)), 0.05, abs_tol=1e-12)

    def test_offset_from_the_robot_not_the_origin(self):
        target = look_around_target((3.0, 4.0), 0.0, 0.1, 0.0)
        assert math.isclose(target[0], 3.1, abs_tol=1e-12)
        assert math.isclose(target[1], 4.0, abs_tol=1e-12)

    def test_successive_calls_keep_turning_the_same_way(self):
        # Each cycle recomputes against the new heading, so the commanded error
        # stays at the step instead of shrinking. That is what makes the turn
        # continue rather than settle after one correction.
        yaw = 0.0
        bearings = []
        for _ in range(4):
            x, y = look_around_target((0.0, 0.0), yaw, 0.05, 2.0)
            bearings.append(_wrap(math.atan2(y, x) - yaw))
            yaw += 1.0          # the robot turned part of the way
        assert all(math.isclose(b, 2.0, abs_tol=1e-9) for b in bearings)


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class TestCentrePath:
    """The potential field half of Task 3's collision avoidance."""

    @staticmethod
    def _corridor(width_cells=18, height_cells=14, gap=(5, 9)):
        """A horizontal corridor: free between the gap rows, occupied outside."""
        rows = []
        for r in range(height_cells):
            free = gap[0] <= r <= gap[1]
            rows.append([0 if free else 100] * width_cells)
        occ = OccupancyMap.from_rows(rows, resolution=0.05,
                                     origin_x=0.0, origin_y=0.0)
        return PlanningGrid(occ, 0.0)

    def test_a_path_hugging_one_wall_moves_toward_the_middle(self):
        grid = self._corridor()
        centre_y = 0.05 * (5 + 9 + 1) / 2
        hugging = [(0.1, 0.28), (0.3, 0.28), (0.5, 0.28), (0.7, 0.28)]
        moved = centre_path(hugging, grid, max_shift_m=0.12)
        for before, after in zip(hugging[1:-1], moved[1:-1]):
            assert abs(after[1] - centre_y) < abs(before[1] - centre_y)

    def test_endpoints_are_never_moved(self):
        grid = self._corridor()
        path = [(0.1, 0.28), (0.3, 0.28), (0.5, 0.28), (0.7, 0.28)]
        moved = centre_path(path, grid, max_shift_m=0.12)
        assert moved[0] == path[0]
        assert moved[-1] == path[-1]

    def test_no_waypoint_moves_further_than_the_bound(self):
        grid = self._corridor()
        path = [(0.1, 0.28), (0.3, 0.28), (0.5, 0.28), (0.7, 0.28)]
        bound = 0.06
        moved = centre_path(path, grid, max_shift_m=bound)
        for before, after in zip(path, moved):
            assert math.dist(before, after) <= bound + 1e-9

    def test_short_paths_and_zero_shift_are_returned_unchanged(self):
        grid = self._corridor()
        two = [(0.1, 0.28), (0.7, 0.28)]
        assert centre_path(two, grid, max_shift_m=0.12) == two
        four = [(0.1, 0.28), (0.3, 0.28), (0.5, 0.28), (0.7, 0.28)]
        assert centre_path(four, grid, max_shift_m=0.0) == four

    def test_it_never_returns_a_blocked_waypoint(self):
        grid = self._corridor()
        path = [(0.1, 0.28), (0.3, 0.28), (0.5, 0.28), (0.7, 0.28)]
        for point in centre_path(path, grid, max_shift_m=0.30):
            assert not grid.is_blocked(point)


def test_look_around_waypoint_is_a_rotation_not_a_stop():
    """The shipped look-around numbers have to clear the follower's thresholds.

    The recovery commands a turn by publishing a waypoint off the robot's nose,
    which only turns the robot if the follower neither declares arrival at it nor
    decides it is close enough to the heading to drive. Experiment 11 ran with a
    0.05 m radius against a 0.05 m arrival tolerance and spent eight recovery
    cycles publishing a stop. This reads both numbers out of the shipped
    parameter file and checks them against the follower's own defaults, so the
    two cannot drift apart again without a red test.
    """
    import math
    import pathlib

    import yaml

    from r7021e_rrt.path_follower import FollowerLimits, follow
    from r7021e_rrt.rrt_star import look_around_target

    # The same package ships in two layouts: config/ at the repository root here,
    # and inside the bringup package when it is handed over as a self-contained
    # lab folder. Search rather than assume, or this test passes in one and
    # errors in the other.
    here = pathlib.Path(__file__).resolve()
    candidates = []
    for parent in here.parents[:6]:
        candidates += sorted(parent.glob('config/lab3.yaml'))
        candidates += sorted(parent.glob('*/config/lab3.yaml'))
        candidates += sorted(parent.glob('*/*/config/lab3.yaml'))
        if candidates:
            break
    assert candidates, f'no config/lab3.yaml found above {here}'
    config = yaml.safe_load(candidates[0].read_text())
    params = config['/navigation_node']['ros__parameters']
    radius = params['look_around_radius']
    step = params['look_around_step']
    limits = FollowerLimits()

    assert radius > limits.goal_tolerance
    # The worst case is the moment just before the navigation node republishes,
    # by which time the robot has already turned max_w for one tick.
    assert step - limits.max_w * 1.0 > limits.turn_in_place

    position, yaw = (1.0, -0.5), 0.4
    target = look_around_target(position, yaw, radius, step)
    command = follow([position, target], position, yaw, limits)
    assert command.v == 0.0
    assert abs(command.w) == limits.max_w
    assert not command.arrived

    # And still a rotation one tick later, after the robot has turned as far as
    # it can before the waypoint is refreshed.
    later = follow([position, target], position, yaw + math.copysign(1.0, step),
                   limits)
    assert later.v == 0.0
    assert not later.arrived


# ------------------------------------------- forgetting exhausted goals

class _FakeNode:
    """The three fields _try_forget_exhausted touches, and a logger that records.

    Testing this against a real Node would need rclpy up; the method only reads
    its own counters and the decision, so it is bound to a stand-in instead.
    """

    def __init__(self, exhausted, resets=0, max_resets=3):
        self._exhausted = list(exhausted)
        self._exhaust_resets = resets
        self.max_exhaust_resets = max_resets
        self._empty_cycles = 7
        self._cycle = 42
        self.warnings = []

    def get_logger(self):
        node = self

        class _Logger:
            def warn(self, message):
                node.warnings.append(message)
        return _Logger()


def _forget(node, decision):
    from r7021e_rrt.navigation_node import NavigationNode
    return NavigationNode._try_forget_exhausted(node, decision)


def _decision(clusters_found, candidates=()):
    from r7021e_rrt.exploration_gain import ExplorationDecision
    return ExplorationDecision(None, list(candidates), clusters_found)


def test_forgets_when_clusters_remain_but_nothing_was_scored():
    node = _FakeNode([(1.0, 2.0), (3.0, 4.0)])
    assert _forget(node, _decision(3)) is True
    assert node._exhausted == []
    assert node._exhaust_resets == 1
    # The countdown restarts, or forgetting buys nothing.
    assert node._empty_cycles == 0
    assert '3 clusters' in node.warnings[0]


def test_does_not_forget_when_the_map_really_is_clear():
    node = _FakeNode([(1.0, 2.0)])
    assert _forget(node, _decision(0)) is False
    assert node._exhausted == [(1.0, 2.0)]


def test_does_not_forget_when_candidates_were_scored_and_lost():
    # Something was planned to and failed. That is unreachability, not filtering,
    # and the look-around and unstick recoveries are the ones that address it.
    node = _FakeNode([(1.0, 2.0)])
    assert _forget(node, _decision(3, candidates=[object()])) is False
    assert node._exhausted == [(1.0, 2.0)]


def test_does_not_forget_when_there_is_nothing_to_forget():
    node = _FakeNode([])
    assert _forget(node, _decision(3)) is False


def test_forgetting_is_bounded_so_the_run_can_still_end():
    node = _FakeNode([(1.0, 2.0)], resets=3, max_resets=3)
    assert _forget(node, _decision(3)) is False
    assert node._exhausted == [(1.0, 2.0)]


def test_each_reset_is_counted_once_and_then_stops():
    node = _FakeNode([(1.0, 2.0)], max_resets=2)
    assert _forget(node, _decision(2)) is True
    node._exhausted = [(5.0, 6.0)]
    assert _forget(node, _decision(2)) is True
    node._exhausted = [(7.0, 8.0)]
    assert _forget(node, _decision(2)) is False
    assert node._exhaust_resets == 2
