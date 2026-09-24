"""Desk tests for the Lab 3 RRT* planner. No ROS, no robot, no simulator.

Same principle as Labs 1 and 2: the part that can be silently wrong is the
geometry and the collision wiring, and a unit test catches it in seconds where a
robot takes a lab session. What is checked here is not that the tree looks
pretty but that the properties the report claims actually hold: that a step
cannot pass through a wall, that a branch cannot run arbitrarily far into
unmapped space, that rewiring leaves every node's cost consistent with its
parent, and that RRT* returns a shorter path than the first branch it found.

Every test seeds its own `random.Random`, so a failure here is reproducible
rather than a thing that happened once on a Tuesday.
"""

from __future__ import annotations

import math
import random

import numpy as np
import pytest

from r7021e_rrt.grid import OccupancyMap, PlanningGrid
from r7021e_rrt.rrt_star import (
    RRTStarNode,
    RRTStarPlanner,
    RRTTree,
    _path_length,
)

RES = 0.05


def build_grid(cells: np.ndarray, inflation: float = 0.0) -> PlanningGrid:
    return PlanningGrid(
        OccupancyMap.from_rows(cells, resolution=RES), inflation_radius_m=inflation)


def open_room(width: int = 60, height: int = 60) -> np.ndarray:
    """A walled empty room. 3.0 by 3.0 m at 0.05 m per cell by default."""
    cells = np.zeros((height, width), dtype=np.int16)
    cells[0, :] = cells[-1, :] = cells[:, 0] = cells[:, -1] = 100
    return cells


def planner(grid: PlanningGrid, seed: int = 0, **kwargs: object) -> RRTStarPlanner:
    kwargs.setdefault('max_plan_time_s', 10.0)  # no wall-clock flakiness in CI
    return RRTStarPlanner(grid, rng=random.Random(seed), **kwargs)  # type: ignore[arg-type]


# ------------------------------------------------------------------- primitives


def test_steer_takes_exactly_one_step() -> None:
    p = planner(build_grid(open_room()))
    result = p.steer((0.0, 0.0), (10.0, 0.0), 0.3)
    assert result == pytest.approx((0.3, 0.0))


def test_steer_does_not_overshoot_a_near_target() -> None:
    """Returning to_point when it is closer, rather than overshooting past it."""
    p = planner(build_grid(open_room()))
    assert p.steer((0.0, 0.0), (0.1, 0.0), 0.3) == pytest.approx((0.1, 0.0))


def test_steer_on_a_zero_length_segment_does_not_divide_by_zero() -> None:
    p = planner(build_grid(open_room()))
    assert p.steer((1.0, 1.0), (1.0, 1.0), 0.3) == (1.0, 1.0)


def test_nearest_node_scans_the_whole_tree() -> None:
    p = planner(build_grid(open_room()))
    tree = RRTTree(8)
    for point in [(0.0, 0.0), (1.0, 1.0), (0.2, 0.0), (2.0, 2.0)]:
        tree.add(RRTStarNode(point))
    assert p.nearest_node(tree, (0.25, 0.0)).point == (0.2, 0.0)
    assert p.nearest_node(tree, (1.9, 1.9)).point == (2.0, 2.0)


def test_goal_sampling_rate_of_one_always_returns_the_goal() -> None:
    p = planner(build_grid(open_room()), goal_sample_rate=1.0)
    p._goal = (1.23, 4.56)
    assert p.sample_random_point() == (1.23, 4.56)


def test_samples_land_inside_the_map() -> None:
    grid = build_grid(open_room())
    p = planner(grid, goal_sample_rate=0.0)
    for _ in range(200):
        x, y = p.sample_random_point()
        assert grid.info.min_x <= x <= grid.info.max_x
        assert grid.info.min_y <= y <= grid.info.max_y


# -------------------------------------------------------------- collision check


def test_collision_check_catches_a_wall_between_the_endpoints() -> None:
    """The whole point of checking along the line. Both endpoints are free."""
    cells = open_room()
    cells[:, 30] = 100  # one-cell wall, 0.05 m thick
    grid = build_grid(cells)
    p = planner(grid)
    assert grid.is_known_free((1.0, 1.0))
    assert grid.is_known_free((2.0, 1.0))
    assert not p.is_collision_free((1.0, 1.0), (2.0, 1.0))


def test_collision_check_allows_a_clear_segment() -> None:
    p = planner(build_grid(open_room()))
    assert p.is_collision_free((1.0, 1.0), (2.0, 1.0))


def test_inflation_closes_a_gap_narrower_than_the_robot() -> None:
    """Task 3's answer, stated as a test rather than as a claim in the report.

    A 0.15 m gap is wider than the Burger is not: at a 0.105 m inflation radius
    the collars from both sides meet, and the planner refuses to thread it.
    """
    cells = open_room()
    cells[:, 30] = 100
    cells[28:31, 30] = 0  # a 0.15 m gap in the wall
    wide_open = build_grid(cells, inflation=0.0)
    inflated = build_grid(cells, inflation=0.105)
    assert planner(wide_open).is_collision_free((1.4, 1.475), (1.6, 1.475))
    assert not planner(inflated).is_collision_free((1.4, 1.475), (1.6, 1.475))


def test_segment_off_the_map_is_rejected() -> None:
    p = planner(build_grid(open_room()))
    assert not p.is_collision_free((1.0, 1.0), (99.0, 1.0))


def test_unknown_space_is_traversable_within_the_lookahead() -> None:
    cells = open_room()
    cells[:, 30:] = -1
    p = planner(build_grid(cells), unknown_lookahead_m=0.5)
    # Starts on known-free ground, 0.3 m into unknown. Allowed.
    assert p.is_collision_free((1.45, 1.0), (1.75, 1.0))


def test_unknown_space_beyond_the_lookahead_is_rejected() -> None:
    cells = open_room()
    cells[:, 30:] = -1
    p = planner(build_grid(cells), unknown_lookahead_m=0.5)
    assert not p.is_collision_free((1.45, 1.0), (2.45, 1.0))


def test_unknown_run_accumulates_across_consecutive_edges() -> None:
    """The reason unknown_run lives on the node and not on the edge.

    Three 0.3 m steps into unknown are each individually under a 0.5 m cap. If
    the cap were applied per edge, the tree would walk 0.9 m into unmapped space
    one legal step at a time.
    """
    cells = open_room()
    cells[:, 30:] = -1
    p = planner(build_grid(cells), unknown_lookahead_m=0.5)

    # Unknown begins at x = 1.50, so the first hop ends 0.25 m into it, not
    # 0.30: the run is measured from the last known-free sample, not from the
    # start of the edge.
    ok, run = p._check_segment((1.45, 1.0), (1.75, 1.0), 0.0)
    assert ok
    assert run == pytest.approx(0.25, abs=0.03)

    # The second hop is entirely unknown, so it adds its whole length to the
    # inherited run and lands just under the cap.
    ok, run = p._check_segment((1.75, 1.0), (1.95, 1.0), run)
    assert ok
    assert run == pytest.approx(0.45, abs=0.03)

    # The third takes the same branch past 0.5 m and must be refused, even
    # though its own 0.2 m length is well inside the cap on its own.
    assert not p._check_segment((1.95, 1.0), (2.15, 1.0), run)[0]


def test_unknown_run_resets_on_known_free_ground() -> None:
    """A branch that comes back out of unknown space gets its budget back."""
    cells = open_room()
    cells[:, 20:28] = -1  # a 0.40 m band, inside a 0.50 m cap
    p = planner(build_grid(cells), unknown_lookahead_m=0.5)
    ok, run = p._check_segment((0.9, 1.0), (1.6, 1.0), 0.0)
    assert ok
    assert run == pytest.approx(0.0, abs=1e-9)


# ------------------------------------------------------------------- planning


def test_plans_a_straight_path_across_an_empty_room() -> None:
    grid = build_grid(open_room())
    result = planner(grid).plan((0.5, 1.5), (2.5, 1.5))
    assert result.succeeded
    assert result.path is not None
    assert result.path[0] == (0.5, 1.5)
    assert math.dist(result.path[-1], (2.5, 1.5)) <= 1e-9
    # RRT* on an open room should come close to the straight line. 30 percent of
    # slack is generous and still fails loudly on a planner that wanders.
    assert result.length < 2.0 * 1.3


def test_every_returned_edge_is_collision_free() -> None:
    """The property the robot actually depends on, checked on the output itself."""
    cells = open_room()
    cells[:, 30] = 100
    cells[10:20, 30] = 0
    grid = build_grid(cells, inflation=0.05)
    p = planner(grid, max_iterations=4000, extra_iterations_after_solution=400)
    result = p.plan((0.5, 0.75), (2.5, 0.75))
    assert result.succeeded
    assert result.path is not None
    for a, b in zip(result.path, result.path[1:]):
        assert p.is_collision_free(a, b), f'segment {a} -> {b} passes through a wall'


def test_returns_none_when_the_goal_is_walled_off() -> None:
    """None is a real answer; exploration_gain drops those candidates."""
    cells = open_room()
    cells[:, 30] = 100  # no gap
    result = planner(build_grid(cells), max_iterations=600).plan((0.5, 1.5), (2.5, 1.5))
    assert not result.succeeded
    assert result.path is None


def test_a_start_inside_an_inflated_wall_still_plans() -> None:
    """Not a hypothetical: in a 0.8 m maze corridor the robot is usually closer
    to a wall than its own footprint radius, and refusing to plan would stop the
    run permanently."""
    grid = build_grid(open_room(), inflation=0.105)
    start = (0.07, 1.5)  # inside the left wall's collar
    assert grid.is_blocked(start)
    assert planner(grid).plan(start, (2.5, 1.5)).succeeded


def test_rewiring_leaves_every_cost_consistent_with_its_parent() -> None:
    """The subtree cost push, checked over the whole tree rather than one node.

    A rewire that re-parents a node without correcting its descendants leaves
    the tree looking fine from outside while later comparisons mix current and
    stale costs. This is the only way to see that from a test.
    """
    grid = build_grid(open_room())
    p = planner(grid, max_iterations=600)
    p._goal = (2.5, 1.5)
    p.plan((0.5, 1.5), (2.5, 1.5))

    # Rebuild a tree by hand so the assertions can reach it after planning.
    tree = RRTTree(600)
    root = tree.add(RRTStarNode((0.5, 1.5)))
    rng = random.Random(1)
    for _ in range(300):
        sample = (rng.uniform(0.1, 2.9), rng.uniform(0.1, 2.9))
        nearest = p.nearest_node(tree, sample)
        new_point = p.steer(nearest.point, sample, p.step_size_m)
        free, run = p._check_segment(nearest.point, new_point, nearest.unknown_run)
        if not free:
            continue
        node = p._insert_with_best_parent(tree, nearest, new_point, run)
        p.rewire(tree, node)

    assert len(tree) > 50
    for node in tree:
        if node.parent is None:
            assert node is root
            assert node.cost == 0.0
            continue
        expected = node.parent.cost + math.dist(node.parent.point, node.point)
        assert node.cost == pytest.approx(expected, abs=1e-9)


def test_no_cycles_in_the_tree_after_rewiring() -> None:
    """A rewire that re-parents a node to one of its own descendants makes a
    cycle, and extract_path then never terminates."""
    grid = build_grid(open_room())
    p = planner(grid, max_iterations=800)
    result = p.plan((0.5, 1.5), (2.5, 1.5))
    assert result.succeeded
    assert result.path is not None
    assert len(result.path) == len(set(result.path))


def test_rrt_star_beats_the_first_branch_it_found() -> None:
    """Slide 35's whole claim: keep growing, take the shortest branch.

    Compared against the same planner given no post-solution iterations, which
    is the plain-RRT behaviour of returning whatever first reached the goal.
    """
    grid = build_grid(open_room(80, 80))
    start, goal = (0.5, 0.5), (3.5, 3.5)
    greedy = planner(grid, seed=7, extra_iterations_after_solution=0).plan(start, goal)
    refined = planner(grid, seed=7, extra_iterations_after_solution=600).plan(start, goal)
    assert greedy.succeeded and refined.succeeded
    assert refined.length <= greedy.length


def test_path_length_matches_the_reported_length() -> None:
    """sum(d(p)) in the report's H equation is this number, not an estimate."""
    result = planner(build_grid(open_room())).plan((0.5, 1.5), (2.5, 1.5))
    assert result.path is not None
    assert result.length == pytest.approx(_path_length(result.path))


def test_plan_reports_the_edges_it_built() -> None:
    """The RViz tree marker is drawn from these; an empty list draws nothing."""
    result = planner(build_grid(open_room())).plan((0.5, 1.5), (2.5, 1.5))
    assert len(result.edges) == result.iterations or len(result.edges) > 10


def test_time_budget_is_honoured() -> None:
    """Compute time is not graded, but the replan timer still has a period."""
    grid = build_grid(open_room(120, 120))
    p = planner(grid, max_iterations=100000)
    p.max_plan_time_s = 0.05
    result = p.plan((0.5, 0.5), (5.5, 5.5))
    assert result.elapsed_s < 1.0
    assert result.iterations < 100000


def test_seeded_runs_are_reproducible() -> None:
    grid = build_grid(open_room())
    a = planner(grid, seed=42).plan((0.5, 1.5), (2.5, 1.5))
    b = planner(grid, seed=42).plan((0.5, 1.5), (2.5, 1.5))
    assert a.path == b.path


def test_the_escape_budget_cannot_carry_an_edge_through_a_wall() -> None:
    """The safety property behind the blocked-start exemption.

    The thinnest blocked region containing a real wall is the wall plus a full
    collar either side, so it is at least 2 * stamped_radius + one cell thick,
    and the escape budget is exactly 2 * stamped_radius. The robot may leave the
    collar it is in; it may not tunnel through the wall that made it.
    """
    cells = open_room()
    cells[:, 30] = 100
    grid = build_grid(cells, inflation=0.105)
    inside_the_collar = (1.40, 1.5)
    assert grid.is_blocked(inside_the_collar)

    p = planner(grid)
    p._escape_point = inside_the_collar
    # Out of the collar on the near side: allowed.
    assert p._check_segment(inside_the_collar, (1.20, 1.5), 0.0)[0]
    # Through the wall to the far side: refused, despite starting blocked.
    assert not p._check_segment(inside_the_collar, (1.80, 1.5), 0.0)[0]


def test_the_escape_exemption_does_not_apply_to_other_nodes() -> None:
    """Only the point the robot is actually standing on gets the exemption."""
    cells = open_room()
    cells[:, 30] = 100
    grid = build_grid(cells, inflation=0.105)
    p = planner(grid)
    p._escape_point = (0.5, 0.5)
    assert not p._check_segment((1.40, 1.5), (1.20, 1.5), 0.0)[0]
