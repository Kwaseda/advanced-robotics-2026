"""Desk tests for frontier clustering and goal selection. No ROS, no simulator.

This is the graded centre of the lab, so these tests are written against the
claims the report makes rather than against the implementation: that I(p) is
the reduced-range frontier count and not the cluster's own size, that H is
minimised and not maximised, that the real RRT* path length is what enters H
rather than a straight-line estimate, that an unreachable candidate is dropped
instead of crashing the cycle, and that the commitment stops the robot
oscillating between two frontiers of similar score.

Grids here are built at 0.05 m per cell, the SLAM map's real resolution, so the
cluster size floors and the gain radius mean the same thing they will mean on
the robot.
"""

from __future__ import annotations

import math
import random

import numpy as np
import pytest

from r7021e_rrt.exploration_gain import (
    FRONTIER_VALUE,
    MODE_CLUSTER_SIZE,
    MODE_REDUCED_RANGE,
    Candidate,
    FrontierCluster,
    FrontierField,
    GainConfig,
    cluster_goal_point,
    find_frontier_clusters,
    information_gain,
    score_path,
    select_best_frontier,
)
from r7021e_rrt.grid import GridInfo, OccupancyMap, PlanningGrid
from r7021e_rrt.rrt_star import PlanResult, RRTStarPlanner

RES = 0.05
WIDTH, HEIGHT = 80, 80  # 4.0 by 4.0 m


def empty_frontier() -> np.ndarray:
    return np.zeros((HEIGHT, WIDTH), dtype=np.int16)


def as_map(cells: np.ndarray) -> OccupancyMap:
    return OccupancyMap.from_rows(cells, resolution=RES)


def open_map() -> np.ndarray:
    cells = np.zeros((HEIGHT, WIDTH), dtype=np.int16)
    cells[0, :] = cells[-1, :] = cells[:, 0] = cells[:, -1] = 100
    return cells


# ------------------------------------------------------------------ clustering


def test_two_separated_runs_become_two_clusters() -> None:
    cells = empty_frontier()
    cells[10:20, 10] = FRONTIER_VALUE
    cells[10:20, 60] = FRONTIER_VALUE
    clusters = find_frontier_clusters(as_map(cells))
    assert len(clusters) == 2
    assert sorted(c.size() for c in clusters) == [10, 10]


def test_a_diagonal_frontier_is_one_cluster() -> None:
    """8-neighbourhood, and this test is the reason it is 8 and not 4.

    The boundary of what a rotating LiDAR has seen is a curve, and a curve on a
    grid is a staircase. Under 4-connectivity a staircase is N clusters of one
    cell, all below the noise floor, all discarded, and the robot concludes it
    has explored everything while looking straight at a frontier. That is what
    ended the first full Gazebo run with half the maze unmapped.
    """
    cells = empty_frontier()
    for i in range(6):
        cells[10 + i, 10 + i] = FRONTIER_VALUE
    clusters = find_frontier_clusters(as_map(cells), min_size=5)
    assert len(clusters) == 1
    assert clusters[0].size() == 6


def test_frontiers_that_only_touch_at_a_corner_do_merge() -> None:
    """The price of 8-connectivity, stated rather than hidden.

    Two arms meeting at one corner become a single cluster whose centroid sits
    between them. The centroid fallback walk handles the case where that lands
    somewhere unusable, and the size cap splits anything long enough for the
    merge to matter, so the price is worth paying against silently discarding
    every diagonal boundary.
    """
    cells = empty_frontier()
    cells[20, 20:26] = FRONTIER_VALUE
    cells[21, 26:32] = FRONTIER_VALUE
    clusters = find_frontier_clusters(as_map(cells), min_size=5)
    assert len(clusters) == 1


def test_clusters_below_the_floor_are_dropped_as_noise() -> None:
    cells = empty_frontier()
    cells[10, 10] = FRONTIER_VALUE           # a single stray pixel
    cells[20:30, 40] = FRONTIER_VALUE        # a real frontier
    clusters = find_frontier_clusters(as_map(cells), min_size=5)
    assert len(clusters) == 1
    assert clusters[0].size() == 10


def test_the_size_cap_splits_rather_than_truncates() -> None:
    """A 60-cell frontier at a cap of 20 becomes three candidates, not one.

    Truncating would silently discard two thirds of a long corridor frontier.
    Splitting spreads candidates along it, which is what you want: the centre of
    a long corridor frontier is rarely where the robot should drive.
    """
    cells = empty_frontier()
    cells[10:70, 40] = FRONTIER_VALUE
    clusters = find_frontier_clusters(as_map(cells), min_size=1, max_size=20)
    assert sum(c.size() for c in clusters) == 60
    assert all(c.size() <= 20 for c in clusters)
    assert len(clusters) == 3


def test_the_cap_does_not_lose_the_head_of_an_oversized_frontier() -> None:
    """Regression. The cap releases cells it queued but never popped, and the
    first implementation then never looked at them again, because the outer loop
    walked a list of frontier cells taken once at the start and the released
    cells sat earlier in that order. A long corridor frontier quietly lost its
    first chunk, which is invisible in a total cell count and shows up much later
    as a robot that will not go back for a gap it already walked past.

    Built as a plus shape so that the BFS from the first cell in row-major order
    fans out in several directions at once and hits the cap with a non-empty
    queue, which is the only situation in which the release path runs at all.
    """
    cells = empty_frontier()
    cells[40, 20:60] = FRONTIER_VALUE   # a 40-cell horizontal arm
    cells[20:60, 40] = FRONTIER_VALUE   # a 40-cell vertical arm through it
    total = int(np.count_nonzero(cells == FRONTIER_VALUE))

    clusters = find_frontier_clusters(as_map(cells), min_size=1, max_size=12)
    assert sum(c.size() for c in clusters) == total
    assert all(c.size() <= 12 for c in clusters)
    # And no cell is claimed twice.
    claimed = [cell for c in clusters for cell in c.cells]
    assert len(claimed) == len(set(claimed)) == total


def test_no_frontier_cells_means_no_clusters() -> None:
    assert find_frontier_clusters(as_map(empty_frontier())) == []


def test_cluster_cells_are_col_row_not_row_col() -> None:
    """(col, row), to read in the same order as world (x, y)."""
    cells = empty_frontier()
    cells[3, 50:56] = FRONTIER_VALUE  # row 3, columns 50 to 55
    cluster = find_frontier_clusters(as_map(cells), min_size=1)[0]
    assert all(cell[1] == 3 for cell in cluster.cells)
    assert sorted(cell[0] for cell in cluster.cells) == list(range(50, 56))


# ---------------------------------------------------------------- goal points


def planning(cells: np.ndarray, inflation: float = 0.0) -> PlanningGrid:
    return PlanningGrid(as_map(cells), inflation_radius_m=inflation)


def test_centroid_is_used_when_it_is_free() -> None:
    cluster = FrontierCluster([(20, 30), (20, 31), (20, 32)])
    goal = cluster_goal_point(cluster, planning(open_map()), (0.5, 0.5))
    assert goal == pytest.approx((1.025, 1.575))


def test_concave_cluster_falls_back_toward_the_robot() -> None:
    """The fallback the skeleton insisted on. A frontier wrapped around a corner
    has its centroid inside the corner."""
    cells = open_map()
    cells[28:33, 28:33] = 100  # a block where the centroid would land
    cluster = FrontierCluster([(30, 25), (30, 35), (25, 30), (35, 30)])
    grid = planning(cells)
    centroid = (1.525, 1.525)
    assert grid.is_blocked(centroid)
    goal = cluster_goal_point(cluster, grid, (0.3, 0.3))
    assert goal is not None
    assert not grid.is_blocked(goal)
    # One of the cluster's own cells, which is where the frontier is seen from.
    assert grid.info.world_to_cell(goal) in cluster.cells


def test_a_blocked_centroid_gives_the_same_goal_wherever_the_robot_stands() -> None:
    """A goal that moves with the robot can be placed on one cycle and dropped on
    the next, and the robot then turns back and forth between two frontiers."""
    cells = open_map()
    cells[28:33, 28:33] = 100
    cluster = FrontierCluster([(30, 22), (30, 38), (22, 30), (38, 30)])
    grid = planning(cells)
    goals = {cluster_goal_point(cluster, grid, robot)
             for robot in ((0.3, 0.3), (3.7, 3.7), (0.3, 3.7), (3.7, 0.3))}
    assert len(goals) == 1
    (goal,) = goals
    assert goal is not None
    assert grid.info.world_to_cell(goal) in cluster.cells


def test_a_cluster_straddling_unknown_prefers_the_robots_side() -> None:
    """A roomier cell beyond a strip of unknown is one the planner may not reach."""
    cells = open_map()
    cells[:, 38:43] = -1          # an unknown strip splitting the room
    cells[:, 33] = 100            # a wall close behind the near side
    grid = planning(cells)
    assert grid.clearance(grid.info.cell_to_world((43, 31))) > \
        grid.clearance(grid.info.cell_to_world((37, 31)))
    near = [(37, r) for r in range(30, 34)]
    far = [(43, r) for r in range(30, 34)]
    cluster = FrontierCluster(near + far)
    goal = cluster_goal_point(cluster, grid, (1.8, 1.5))
    assert goal is not None
    assert grid.info.world_to_cell(goal) in near


def test_goal_uses_the_inflated_grid_not_the_raw_map() -> None:
    """A goal free on the raw map but inside a collar is a goal the planner is
    forbidden to reach, so accepting it would fail every cycle, forever."""
    cells = open_map()
    cells[30, 32] = cells[31, 32] = 100   # two cells, so despeckling keeps them
    cluster = FrontierCluster([(30, 30), (30, 30), (30, 30)])
    raw = planning(cells, inflation=0.0)
    inflated = planning(cells, inflation=0.105)
    centroid = (1.525, 1.525)
    assert raw.is_known_free(centroid)
    assert inflated.is_blocked(centroid)
    assert cluster_goal_point(cluster, inflated, (0.3, 0.3)) != centroid


def test_unreachable_cluster_returns_none() -> None:
    """Behind a wall with nothing free on the way back: dropped, not crashed."""
    cells = np.full((HEIGHT, WIDTH), 100, dtype=np.int16)
    cluster = FrontierCluster([(40, 40)])
    assert cluster_goal_point(cluster, planning(cells), (0.5, 0.5)) is None


def test_goal_at_the_robot_position_does_not_divide_by_zero() -> None:
    cells = np.full((HEIGHT, WIDTH), 100, dtype=np.int16)
    cluster = FrontierCluster([(20, 20)])
    robot = (1.025, 1.025)  # exactly the centroid
    assert cluster_goal_point(cluster, planning(cells), robot) is None


# ------------------------------------------------------------ information gain


def test_reduced_range_counts_cells_from_every_cluster_not_just_this_one() -> None:
    """The difference between the two I(p) forms, in one assertion.

    Two clusters 0.25 m apart. Cluster size sees 6 cells. The reduced-range form
    at 0.75 m sees all 12, because one visit would clear both, and that is the
    behaviour the lab instructions' tip is asking for.
    """
    cells = empty_frontier()
    cells[30:36, 40] = FRONTIER_VALUE
    cells[30:36, 45] = FRONTIER_VALUE
    field_ = FrontierField(as_map(cells))
    clusters = find_frontier_clusters(as_map(cells), min_size=1)
    cluster = clusters[0]
    goal = (2.025, 1.625)

    reduced = information_gain(
        cluster, goal, field_, GainConfig(MODE_REDUCED_RANGE, 0.75, 0.10))
    plain = information_gain(
        cluster, goal, field_, GainConfig(MODE_CLUSTER_SIZE, 0.75, 0.10))
    assert plain == 6.0
    assert reduced == 12.0


def test_reduced_range_radius_actually_reduces() -> None:
    """A radius far below the Burger's 3.5 m LiDAR is the whole point of the tip."""
    cells = empty_frontier()
    cells[10:70, 40] = FRONTIER_VALUE  # a 3.0 m long frontier
    field_ = FrontierField(as_map(cells))
    cluster = find_frontier_clusters(as_map(cells), min_size=1, max_size=1000)[0]
    goal = (2.025, 2.025)
    near = information_gain(cluster, goal, field_, GainConfig(radius_m=0.25))
    far = information_gain(cluster, goal, field_, GainConfig(radius_m=3.5))
    assert near < far
    assert near == pytest.approx(10.0, abs=2.0)


def test_gain_config_rejects_an_unknown_mode() -> None:
    with pytest.raises(ValueError):
        GainConfig(mode='next_best_view')
    with pytest.raises(ValueError):
        GainConfig(radius_m=0.0)


def test_empty_frontier_field_reports_zero_gain() -> None:
    field_ = FrontierField(as_map(empty_frontier()))
    assert field_.count_within((1.0, 1.0), 1.0) == 0


# ----------------------------------------------------------------- the score


def test_h_is_distance_minus_weighted_gain() -> None:
    """H(p) = sum(d(p)) - w * I(p), the equation the report prints."""
    path = [(0.0, 0.0), (1.0, 0.0), (1.0, 2.0)]  # 3.0 m
    config = GainConfig(weight_m_per_cell=0.1)
    assert score_path(path, 10.0, config) == pytest.approx(3.0 - 1.0)


def test_a_zero_weight_makes_the_choice_purely_nearest_frontier() -> None:
    """The greedy end of the greedy-versus-complete knob the competition asks about."""
    path = [(0.0, 0.0), (2.0, 0.0)]
    greedy = GainConfig(weight_m_per_cell=0.0)
    assert score_path(path, 500.0, greedy) == pytest.approx(2.0)


def test_a_large_weight_buys_a_longer_drive_for_a_bigger_frontier() -> None:
    near_small = score_path([(0.0, 0.0), (1.0, 0.0)], 5.0, GainConfig(weight_m_per_cell=0.5))
    far_large = score_path([(0.0, 0.0), (4.0, 0.0)], 40.0, GainConfig(weight_m_per_cell=0.5))
    assert far_large < near_small  # lower is better


# ------------------------------------------------------------------ selection


def scenario() -> tuple[OccupancyMap, PlanningGrid, RRTStarPlanner]:
    """A room whose right third is unknown, frontier down the seam at x = 2.0."""
    occupancy = open_map()
    occupancy[:, 41:] = -1
    grid = PlanningGrid(as_map(occupancy), inflation_radius_m=0.05)
    frontier = empty_frontier()
    frontier[20:60, 40] = FRONTIER_VALUE
    planner = RRTStarPlanner(grid, rng=random.Random(3), max_plan_time_s=10.0)
    return as_map(frontier), grid, planner


def test_selects_a_reachable_goal_and_reports_a_real_path() -> None:
    frontier, grid, planner = scenario()
    decision = select_best_frontier(frontier, grid, (0.5, 1.5), planner)
    assert decision.chosen is not None
    assert decision.chosen.path is not None
    assert decision.chosen.path[0] == (0.5, 1.5)
    # The scored distance is the planned path, not a straight line to the goal.
    assert decision.chosen.path_length >= math.dist(
        (0.5, 1.5), decision.chosen.goal) - 1e-9


def test_no_frontier_cells_is_the_explored_enough_signal() -> None:
    _, grid, planner = scenario()
    decision = select_best_frontier(
        as_map(empty_frontier()), grid, (0.5, 1.5), planner)
    assert decision.chosen is None
    assert decision.clusters_found == 0


def test_a_frontier_walled_off_from_the_robot_yields_no_choice() -> None:
    """Every candidate unreachable is not a crash and not a silent success."""
    occupancy = open_map()
    occupancy[:, 40] = 100  # a solid wall, no gap
    occupancy[:, 41:] = -1
    grid = PlanningGrid(as_map(occupancy), inflation_radius_m=0.05)
    frontier = empty_frontier()
    frontier[20:60, 39] = FRONTIER_VALUE
    planner = RRTStarPlanner(
        grid, rng=random.Random(1), max_iterations=400, max_plan_time_s=10.0)
    # The goal sits in the wall's collar, so either the fallback walk or the
    # planner rejects it. Both outcomes mean the same thing to the caller.
    decision = select_best_frontier(
        as_map(frontier), grid, (3.5, 1.5), planner, min_cluster_size=5)
    assert decision.chosen is None


def test_mismatched_frontier_and_map_geometry_is_refused() -> None:
    """One cycle after a SLAM resize the two topics disagree, and a cell index
    would silently mean a different place in each."""
    _, grid, planner = scenario()
    smaller = OccupancyMap.from_rows(
        np.zeros((40, 40), dtype=np.int16), resolution=RES)
    with pytest.raises(ValueError):
        select_best_frontier(smaller, grid, (0.5, 1.5), planner)


def test_candidate_count_is_capped() -> None:
    """Slide 59's 'limit the number of candidate solutions'. One full RRT* runs
    per candidate, so this cap is what keeps a cycle inside its period."""
    occupancy = open_map()
    occupancy[:, 60:] = -1
    grid = PlanningGrid(as_map(occupancy), inflation_radius_m=0.05)
    frontier = empty_frontier()
    for row in range(10, 70, 6):
        frontier[row:row + 5, 59] = FRONTIER_VALUE
    planner = RRTStarPlanner(
        grid, rng=random.Random(2), max_iterations=300, max_plan_time_s=10.0)
    decision = select_best_frontier(
        as_map(frontier), grid, (0.5, 1.5), planner, max_candidates=3)
    assert decision.clusters_found == 10
    assert len(decision.candidates) == 3


def test_the_minimum_score_wins() -> None:
    """Lower H is better. Getting this backwards sends the robot to the worst
    frontier every time and still looks like a working system."""
    candidates = [
        Candidate(FrontierCluster([(0, 0)]), (1.0, 0.0), 5.0,
                  path=[(0.0, 0.0), (1.0, 0.0)], path_length=1.0, score=0.5),
        Candidate(FrontierCluster([(0, 0)]), (3.0, 0.0), 40.0,
                  path=[(0.0, 0.0), (3.0, 0.0)], path_length=3.0, score=-1.0),
    ]
    assert min(candidates, key=lambda c: c.score).goal == (3.0, 0.0)


# ----------------------------------------------------------------- commitment


def committed_scenario() -> tuple[OccupancyMap, PlanningGrid, RRTStarPlanner]:
    """Two frontiers of deliberately similar score, one on each side."""
    occupancy = open_map()
    occupancy[:, 60:] = -1
    occupancy[:, 1:18] = -1
    grid = PlanningGrid(as_map(occupancy), inflation_radius_m=0.05)
    frontier = empty_frontier()
    frontier[30:50, 59] = FRONTIER_VALUE
    frontier[30:50, 18] = FRONTIER_VALUE
    planner = RRTStarPlanner(
        grid, rng=random.Random(5), max_iterations=800, max_plan_time_s=10.0)
    return as_map(frontier), grid, planner


def test_a_committed_goal_is_held_against_a_marginally_better_rival() -> None:
    """Anticipated challenge number one in the plan draft: H is memoryless, so
    without this the robot bounces between two similar clusters forever."""
    frontier, grid, planner = committed_scenario()
    robot = (2.0, 2.0)
    first = select_best_frontier(frontier, grid, robot, planner)
    assert first.chosen is not None
    loser = min(
        (c for c in first.candidates if c.reachable and c is not first.chosen),
        key=lambda c: c.score)

    held = select_best_frontier(
        frontier, grid, robot, planner,
        committed_goal=loser.goal, switch_margin=100.0)
    assert held.kept_committed
    assert held.chosen is not None
    assert math.dist(held.chosen.goal, loser.goal) < 0.3


def test_a_decisively_better_rival_does_take_the_goal() -> None:
    """Hysteresis must not become paralysis."""
    frontier, grid, planner = committed_scenario()
    robot = (2.0, 2.0)
    first = select_best_frontier(frontier, grid, robot, planner)
    assert first.chosen is not None
    loser = min(
        (c for c in first.candidates if c.reachable and c is not first.chosen),
        key=lambda c: c.score)

    switched = select_best_frontier(
        frontier, grid, robot, planner,
        committed_goal=loser.goal, switch_margin=0.0)
    assert switched.chosen is not None
    assert math.dist(switched.chosen.goal, loser.goal) > 0.3


def test_a_committed_goal_that_has_vanished_is_simply_replaced() -> None:
    """The frontier the robot was driving to gets mapped and disappears. That is
    the normal, successful case, not an error."""
    frontier, grid, planner = committed_scenario()
    decision = select_best_frontier(
        frontier, grid, (2.0, 2.0), planner,
        committed_goal=(3.9, 3.9), switch_margin=100.0)
    assert decision.chosen is not None
    assert not decision.kept_committed


# ------------------------------------------------------------ retired goals


def test_an_exhausted_goal_is_not_offered_again() -> None:
    """The loop breaker. A frontier that survives the robot standing next to it
    keeps winning on distance forever, because H contains nothing that knows
    where the robot has already been."""
    frontier, grid, planner = scenario()
    first = select_best_frontier(frontier, grid, (0.5, 1.5), planner)
    assert first.chosen is not None

    again = select_best_frontier(
        frontier, grid, (0.5, 1.5), planner,
        exhausted_goals=[first.chosen.goal], exhaust_radius_m=0.30)
    if again.chosen is not None:
        assert math.dist(again.chosen.goal, first.chosen.goal) > 0.30


def test_exhausting_every_goal_is_the_explored_enough_signal() -> None:
    """Retiring everything must terminate the run, not crash it."""
    frontier, grid, planner = scenario()
    first = select_best_frontier(frontier, grid, (0.5, 1.5), planner)
    assert first.chosen is not None
    everywhere = [(x * 0.1, y * 0.1) for x in range(-5, 45) for y in range(0, 45)]
    decision = select_best_frontier(
        frontier, grid, (0.5, 1.5), planner,
        exhausted_goals=everywhere, exhaust_radius_m=0.30)
    assert decision.chosen is None
    assert decision.clusters_found > 0  # clusters existed, all of them retired


def test_a_distant_goal_survives_an_unrelated_retirement() -> None:
    """Retirement must be local. Blacklisting a radius must not quietly stop
    the robot going anywhere else."""
    frontier, grid, planner = scenario()
    baseline = select_best_frontier(frontier, grid, (0.5, 1.5), planner)
    assert baseline.chosen is not None
    decision = select_best_frontier(
        frontier, grid, (0.5, 1.5), planner,
        exhausted_goals=[(-99.0, -99.0)], exhaust_radius_m=0.30)
    assert decision.chosen is not None
    assert math.dist(decision.chosen.goal, baseline.chosen.goal) < 1e-9


# --------------------------------------------------- the turn term in H(p)

def test_turn_cost_is_off_by_default():
    config = GainConfig()
    assert config.turn_cost_m_per_rad == 0.0
    path = [(0.0, 0.0), (-1.0, 0.0)]
    assert score_path(path, 0.0, config, robot_yaw=0.0) == score_path(
        path, 0.0, config)


def test_turn_cost_charges_the_swing_onto_the_path():
    config = GainConfig(weight_m_per_cell=0.0, turn_cost_m_per_rad=0.15)
    ahead = [(0.0, 0.0), (1.0, 0.0)]
    behind = [(0.0, 0.0), (-1.0, 0.0)]
    # Same length, opposite directions, robot facing east.
    assert score_path(ahead, 0.0, config, robot_yaw=0.0) == 1.0
    assert math.isclose(score_path(behind, 0.0, config, robot_yaw=0.0),
                        1.0 + 0.15 * math.pi)


def test_turn_cost_takes_the_short_way_round():
    config = GainConfig(weight_m_per_cell=0.0, turn_cost_m_per_rad=1.0)
    path = [(0.0, 0.0), (1.0, 0.0)]
    # Robot facing just past due west either way: the charge must be the same.
    left = score_path(path, 0.0, config, robot_yaw=math.pi - 0.1)
    right = score_path(path, 0.0, config, robot_yaw=-math.pi + 0.1)
    assert math.isclose(left, right)
    assert math.isclose(left, 1.0 + (math.pi - 0.1))


def test_turn_cost_needs_a_yaw_and_two_waypoints():
    config = GainConfig(weight_m_per_cell=0.0, turn_cost_m_per_rad=1.0)
    assert score_path([(0.0, 0.0)], 0.0, config, robot_yaw=3.0) == 0.0
    assert score_path([(0.0, 0.0), (-1.0, 0.0)], 0.0, config) == 1.0


def test_turn_cost_cannot_outweigh_a_much_shorter_path():
    # A sanity bound on the units: the charge for the worst possible turn is
    # 0.47 m, so it can reorder candidates that are close in length and cannot
    # make a 5 m path beat a 1 m one.
    config = GainConfig(weight_m_per_cell=0.0, turn_cost_m_per_rad=0.15)
    near_behind = score_path([(0.0, 0.0), (-1.0, 0.0)], 0.0, config, 0.0)
    far_ahead = score_path([(0.0, 0.0), (5.0, 0.0)], 0.0, config, 0.0)
    assert near_behind < far_ahead


# ------------------------------------- the goal must be a viewpoint of its frontier

def test_walkback_is_capped_so_a_goal_cannot_detach_from_its_cluster():
    """A blocked centroid far from the robot yields no goal, not a goal at the robot.

    Uncapped, the fallback walked the whole way to the robot and returned a point
    at its feet, which sees no frontier, scores H as pure distance, and wins.
    """
    from r7021e_rrt.exploration_gain import GOAL_WALKBACK_MAX_M

    # A wall band the cluster sits inside, with the robot 3 m away in free space.
    info = GridInfo(resolution=0.05, origin_x=0.0, origin_y=0.0, width=100, height=40)
    data = np.zeros((40, 100), dtype=np.int16)
    data[:, 0:20] = 100           # everything under the cluster is occupied
    grid = PlanningGrid(OccupancyMap(data, info), inflation_radius_m=0.0)

    cluster = FrontierCluster(cells=[(10, 20)])
    robot = (4.0, 1.0)
    goal = cluster_goal_point(cluster, grid, robot)

    centroid = (info.origin_x + 10.5 * info.resolution,
                info.origin_y + 20.5 * info.resolution)
    if goal is not None:
        assert math.dist(goal, centroid) <= GOAL_WALKBACK_MAX_M + info.resolution
        assert math.dist(goal, robot) > GOAL_WALKBACK_MAX_M


def test_a_reachable_centroid_is_still_returned_unchanged():
    info = GridInfo(resolution=0.05, origin_x=0.0, origin_y=0.0, width=40, height=40)
    data = np.zeros((40, 40), dtype=np.int16)
    grid = PlanningGrid(OccupancyMap(data, info), inflation_radius_m=0.0)
    cluster = FrontierCluster(cells=[(20, 20)])
    goal = cluster_goal_point(cluster, grid, (0.1, 0.1))
    assert goal == (info.origin_x + 20.5 * info.resolution,
                    info.origin_y + 20.5 * info.resolution)


class _IterationBoundPlanner:
    """Solves only when given more iterations than the ordinary cap."""

    def __init__(self) -> None:
        self.max_iterations = 1500
        self.max_plan_time_s = 0.15
        self.budgets: list[tuple[int, float]] = []

    def plan(self, start, goal) -> PlanResult:
        self.budgets.append((self.max_iterations, self.max_plan_time_s))
        if self.max_iterations <= 1500:
            return PlanResult(None)
        return PlanResult([start, goal], math.dist(start, goal))


def test_the_retry_raises_the_iteration_cap_not_only_the_clock() -> None:
    """Long corridor routes need more iterations, not only more time."""
    frontier, grid, _ = scenario()
    planner = _IterationBoundPlanner()
    decision = select_best_frontier(frontier, grid, (0.5, 1.5), planner,
                                    retry_plan_time_scale=10.0)
    assert decision.chosen is not None
    assert (1500, 0.15) in planner.budgets
    assert (15000, 1.5) in planner.budgets
    assert (planner.max_iterations, planner.max_plan_time_s) == (1500, 0.15)
