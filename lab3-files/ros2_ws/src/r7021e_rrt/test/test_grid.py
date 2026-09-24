"""Desk tests for the shared grid representation. No ROS, no robot, no simulator.

Everything here is a coordinate convention or a set membership question, which is
exactly the category of bug that survives a passing integration test and then
presents on hardware as "the planner goes the wrong way". A transposed grid looks
perfectly correct for as long as every test grid is square, so several of these
use deliberately non-square grids.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from r7021e_rrt.grid import GridInfo, OccupancyMap, PlanningGrid, _inflate

RES = 0.05


def test_grid_info_rejects_degenerate_geometry() -> None:
    with pytest.raises(ValueError):
        GridInfo(resolution=0.0, origin_x=0.0, origin_y=0.0, width=10, height=10)
    with pytest.raises(ValueError):
        GridInfo(resolution=RES, origin_x=0.0, origin_y=0.0, width=0, height=10)


def test_cell_to_world_returns_cell_centres() -> None:
    """The half-cell offset. Without it every planned waypoint sits on a corner."""
    info = GridInfo(resolution=RES, origin_x=-1.0, origin_y=-2.0, width=40, height=80)
    assert info.cell_to_world((0, 0)) == pytest.approx((-0.975, -1.975))
    assert info.cell_to_world((10, 20)) == pytest.approx((-0.475, -0.975))


def test_world_and_cell_round_trip() -> None:
    info = GridInfo(resolution=RES, origin_x=-1.0, origin_y=-2.0, width=40, height=80)
    for cell in [(0, 0), (7, 3), (39, 79)]:
        assert info.world_to_cell(info.cell_to_world(cell)) == cell


def test_non_square_grid_is_not_transposed() -> None:
    """width is x, height is y. Only a non-square grid can catch this."""
    rows = np.zeros((4, 9), dtype=np.int16)  # 4 rows of 9 columns
    rows[0, 8] = 100
    grid = OccupancyMap.from_rows(rows, resolution=1.0)
    assert (grid.info.width, grid.info.height) == (9, 4)
    assert grid.value_at_point((8.5, 0.5)) == 100
    assert grid.value_at_point((0.5, 3.5)) == 0


def test_value_off_map_reads_unknown_not_an_error() -> None:
    """A pose just outside a growing SLAM map is a transient, not a failure."""
    grid = OccupancyMap.from_rows(np.zeros((4, 4), dtype=np.int16), resolution=1.0)
    assert grid.value_at_point((99.0, 99.0)) == -1


def test_from_message_matches_ros_row_major_order() -> None:
    """data[row * width + col], the ROS convention, bottom row first."""
    grid = OccupancyMap.from_message(
        data=[0, 1, 2, 3, 4, 5], width=3, height=2,
        resolution=1.0, origin_x=0.0, origin_y=0.0)
    assert grid.value_at_point((0.5, 0.5)) == 0   # row 0, col 0
    assert grid.value_at_point((2.5, 0.5)) == 2   # row 0, col 2
    assert grid.value_at_point((0.5, 1.5)) == 3   # row 1, col 0


def test_from_message_rejects_mismatched_shape() -> None:
    with pytest.raises(ValueError):
        OccupancyMap.from_message([0, 1, 2], 2, 2, 1.0, 0.0, 0.0)


def test_same_frame_as_detects_a_resized_map() -> None:
    """The guard for /map and /frontiers arriving from different map updates."""
    a = OccupancyMap.from_rows(np.zeros((4, 4), dtype=np.int16))
    b = OccupancyMap.from_rows(np.zeros((4, 5), dtype=np.int16))
    assert a.same_frame_as(OccupancyMap.from_rows(np.zeros((4, 4), dtype=np.int16)))
    assert not a.same_frame_as(b)


# --------------------------------------------------------------------- inflation


def test_inflation_radius_rounds_up_not_down() -> None:
    """0.105 m at 0.05 m per cell is 2.1 cells and must stamp 3, not 2.

    Rounding down would give 0.10 m of enforced margin against a YAML file that
    says 0.105, so the config would be quietly lying about what the robot does.
    """
    occupied = np.zeros((11, 11), dtype=bool)
    occupied[5, 5] = True
    inflated = _inflate(occupied, 0.105, RES)
    assert inflated[5, 8]        # three cells away, inside the disc
    assert not inflated[5, 9]    # four cells away


def test_inflation_is_a_disc_not_a_square() -> None:
    occupied = np.zeros((11, 11), dtype=bool)
    occupied[5, 5] = True
    inflated = _inflate(occupied, 0.105, RES)
    assert not inflated[8, 8]    # the corner of the 3-cell box is outside the disc


def test_inflation_does_not_wrap_around_the_map_edge() -> None:
    """np.roll would wrap; an obstacle on the right edge must not appear left."""
    occupied = np.zeros((5, 5), dtype=bool)
    occupied[2, 4] = True
    inflated = _inflate(occupied, 0.10, RES)
    assert not inflated[2, 0]
    assert inflated[2, 2]


def test_zero_radius_inflation_is_the_identity() -> None:
    occupied = np.zeros((4, 4), dtype=bool)
    occupied[1, 1] = True
    assert np.array_equal(_inflate(occupied, 0.0, RES), occupied)


# ----------------------------------------------------------------- PlanningGrid


def planning_grid(values: list[list[int]], inflation: float = 0.0) -> PlanningGrid:
    return PlanningGrid(
        OccupancyMap.from_rows(np.asarray(values, dtype=np.int16), resolution=1.0),
        inflation_radius_m=inflation,
    )


def test_unknown_is_traversable_and_known_free_is_distinct() -> None:
    """The decision the whole of frontier exploration rests on.

    A planner that refuses unknown cells cannot reach any frontier goal, because
    a frontier sits on the boundary of unknown space by definition.

    Two adjacent occupied cells, not one: a lone occupied cell is despeckled away
    as a spurious return before the inflation ever sees it.
    """
    grid = planning_grid([[0, -1, 100, 100]])
    assert grid.classify_point((0.5, 0.5)) == 1     # known free
    assert grid.classify_point((1.5, 0.5)) == 0     # unknown, traversable
    assert grid.classify_point((2.5, 0.5)) == -1    # occupied
    assert not grid.is_blocked((1.5, 0.5))
    assert not grid.is_known_free((1.5, 0.5))


def test_unknown_cells_are_not_inflated() -> None:
    """Inflating unknown would put a blocked collar around every frontier."""
    grid = PlanningGrid(
        OccupancyMap.from_rows(
            np.asarray([[0, 0, -1, 0, 0]], dtype=np.int16), resolution=1.0),
        inflation_radius_m=1.0,
    )
    assert not grid.is_blocked((1.5, 0.5))
    assert not grid.is_blocked((3.5, 0.5))


def test_occupied_inflation_blocks_neighbours_and_beats_unknown() -> None:
    grid = PlanningGrid(
        OccupancyMap.from_rows(
            np.asarray([[0, -1, 100, 100, -1, 0]], dtype=np.int16), resolution=1.0),
        inflation_radius_m=1.0,
    )
    assert grid.is_blocked((2.5, 0.5))
    # The unknown cells either side are inside the wall's collar. Blocked wins:
    # a cell inside an inflated wall is not "unknown with a caveat".
    assert grid.is_blocked((1.5, 0.5))
    assert grid.is_blocked((4.5, 0.5))
    assert not grid.is_blocked((0.5, 0.5))


def test_occupied_threshold_is_respected() -> None:
    """slam_toolbox writes probabilities, not a binary. 50 is not a wall."""
    grid = planning_grid([[0, 50, 64, 65, 100]])
    assert not grid.is_blocked((1.5, 0.5))
    assert not grid.is_blocked((2.5, 0.5))
    assert grid.is_blocked((3.5, 0.5))
    assert grid.is_blocked((4.5, 0.5))


def test_off_map_is_blocked_for_planning() -> None:
    """Deliberately different from OccupancyMap.value_at_point, which reads unknown.

    Reading off the edge of a growing map is benign. Planning off it is not:
    there is no evidence the space exists and the unknown-travel cap cannot
    bound something unbounded.
    """
    grid = planning_grid([[0, 0], [0, 0]])
    assert grid.is_blocked((-1.0, 0.5))
    assert grid.is_blocked((0.5, 99.0))


def test_negative_inflation_is_rejected() -> None:
    with pytest.raises(ValueError):
        planning_grid([[0]], inflation=-0.1)


# -------------------------------------------------------------------- despeckle


def test_an_isolated_occupied_cell_is_not_an_obstacle() -> None:
    """A lone occupied cell in open space is a spurious return, not a wall.

    Left in, a 3-cell inflation disc turns each one into a 7 by 7 blocked blob,
    and a corridor with a scatter of them becomes impassable. A full maze run
    ended with the robot's own cell as the only reachable cell on the whole map
    for exactly this reason.
    """
    cells = np.zeros((11, 11), dtype=np.int16)
    cells[5, 5] = 100
    grid = PlanningGrid(OccupancyMap.from_rows(cells, resolution=RES),
                        inflation_radius_m=0.105)
    assert not grid.blocked.any()


def test_a_two_cell_obstacle_survives_despeckling() -> None:
    """Conservative by design: only cells with no occupied neighbour at all go."""
    cells = np.zeros((11, 11), dtype=np.int16)
    cells[5, 5] = cells[5, 6] = 100
    grid = PlanningGrid(OccupancyMap.from_rows(cells, resolution=RES),
                        inflation_radius_m=0.105)
    assert grid.blocked[5, 5] and grid.blocked[5, 6]


def test_a_wall_is_untouched_by_despeckling() -> None:
    """Walls are continuous, so every wall cell has a neighbour. Nothing is lost."""
    cells = np.zeros((11, 11), dtype=np.int16)
    cells[:, 5] = 100
    grid = PlanningGrid(OccupancyMap.from_rows(cells, resolution=RES),
                        inflation_radius_m=0.0)
    assert grid.blocked[:, 5].all()


def test_despeckling_removes_only_diagonal_free_singletons() -> None:
    """8-neighbourhood, so two cells touching at a corner keep each other."""
    cells = np.zeros((11, 11), dtype=np.int16)
    cells[4, 4] = cells[5, 5] = 100
    grid = PlanningGrid(OccupancyMap.from_rows(cells, resolution=RES),
                        inflation_radius_m=0.0)
    assert grid.blocked[4, 4] and grid.blocked[5, 5]


def test_despeckling_can_be_turned_off() -> None:
    """min_obstacle_neighbours 0 is the old behaviour, kept for comparison runs."""
    cells = np.zeros((11, 11), dtype=np.int16)
    cells[5, 5] = 100
    grid = PlanningGrid(OccupancyMap.from_rows(cells, resolution=RES),
                        inflation_radius_m=0.0, min_obstacle_neighbours=0)
    assert grid.blocked[5, 5]


def test_nearest_and_farthest_free_points_differ() -> None:
    """The two serve different jobs: rooting a plan wants the nearest, and
    recovering from being sealed in wants the furthest, because that move exists
    to see the surroundings from somewhere else."""
    cells = np.zeros((41, 41), dtype=np.int16)
    cells[20, 20] = cells[20, 21] = 100          # a two-cell obstacle at the centre
    grid = PlanningGrid(OccupancyMap.from_rows(cells, resolution=RES),
                        inflation_radius_m=0.105)
    here = grid.info.cell_to_world((20, 20))
    assert grid.is_blocked(here)

    near = grid.nearest_unblocked(here, 0.60)
    far = grid.farthest_unblocked(here, 0.60)
    assert near is not None and far is not None
    assert math.dist(here, near) < math.dist(here, far)
    assert math.dist(here, far) <= 0.60 + 1e-9
    assert not grid.is_blocked(near) and not grid.is_blocked(far)


def test_no_free_point_within_the_radius_returns_none() -> None:
    cells = np.full((21, 21), 100, dtype=np.int16)
    grid = PlanningGrid(OccupancyMap.from_rows(cells, resolution=RES),
                        inflation_radius_m=0.105)
    here = grid.info.cell_to_world((10, 10))
    assert grid.nearest_unblocked(here, 0.30) is None
    assert grid.farthest_unblocked(here, 0.30) is None


def test_a_free_point_behind_a_wall_is_not_reachable() -> None:
    """Crossing a collar is the point; crossing a wall would drive through it."""
    cells = np.zeros((21, 41), dtype=np.int16)
    cells[:, 20] = 100                      # a full-height wall
    grid = PlanningGrid(OccupancyMap.from_rows(cells, resolution=RES),
                        inflation_radius_m=0.105)
    here = grid.info.cell_to_world((20, 10))
    for _, point in grid.reachable_free_points(here, 0.60):
        assert not grid._crosses_occupied(here, point)
