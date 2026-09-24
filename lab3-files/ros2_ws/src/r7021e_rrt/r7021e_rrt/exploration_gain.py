"""
exploration_gain.py

Plain-Python frontier clustering + goal selection. No ROS imports here
either, for the same reason as rrt_star.py: needs to be testable on a
made-up grid before it touches a robot.

Course reference: R7021E-Lecture_5_to_7_Navigation.pdf, slides 56-60
(Robot Exploration / Frontier Clustering).

Decisions already locked in for this lab (from the plan conversation):
- Frontier-based, not next-best-view. The frontier extractor is given and does
  the hard part; next-best-view would couple the gain scoring into the RRT's
  internals. Yamauchi 1997 is the citation.
- Heuristic is H(p) = sum(d(p)) - I(p). No R(p) term in here. Slide 58
  explicitly crosses out the R(p) term for the exploration-specific
  version of the formula. Obstacle risk lives in the map inflation inside
  grid.PlanningGrid instead, not in this score, so it is not added here too.
- Cluster size: a floor of 5 cells (filters single noisy pixels) and a
  ceiling of 100 cells (bounds how much work each cluster costs to
  evaluate, per slide 59's "limit/define the number of candidate
  solutions").
- Goal point per cluster = centroid, WITH a fallback: if the centroid
  lands on an unknown or occupied cell (this happens, frontier clusters
  are often concave), walk from the centroid toward the robot in small
  steps until a free cell is found, and use that instead.
- Plan a REAL RRT* path to every surviving cluster and score by the actual
  path length, not a straight-line estimate. Slide 60's option (b):
  "Plan path to all g_c,i ... travel along minimum p*." Affordable because
  the lab instructions say compute time is not graded.

The information gain
--------------------
I(p) is the number of frontier cells within `radius_m` of the candidate goal,
counted across the WHOLE frontier grid rather than within the candidate's own
cluster, with `radius_m` far below the Burger's 3.5 m LiDAR range. That is the
lab instructions' own tip, verbatim: "greatly reduce the LiDAR range for info
gain computation to drive exploratory behavior". Counting only the candidate's
own cluster is available as `mode="cluster_size"`, so the two can be measured against
each other on the same maze rather than argued about.

The reduced-range form also fixes something plain cell count gets wrong.
Two clusters twenty centimetres apart each score their own size and neither
knows the other exists, so the robot commits to a spot that one visit would
have cleared anyway. Counting every frontier cell within the radius, whoever
it belongs to, prices that correctly.

H mixes metres with cell counts, so I(p) needs a conversion factor to be
subtractable from a path length at all. `weight_m_per_cell` is that factor and
it is the greedy-versus-complete knob the optional competition asks about:
at zero the robot goes to the nearest frontier always, and as it rises the
robot will cross the maze for a large unexplored region.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field
from typing import Sequence

import numpy as np
import numpy.typing as npt

from .grid import OccupancyMap, PlanningGrid
from .rrt_star import RRTStarPlanner, _path_length

Point = tuple[float, float]
Cell = tuple[int, int]

FRONTIER_VALUE = 100  # what frontier_detector_node.py writes into its output

# I(p) forms. Strings rather than an enum because they arrive from a YAML file
# and are echoed back into the report, and a string that matches the report's
# wording is easier to check than an enum member that has to be translated.
MODE_REDUCED_RANGE = "reduced_range"
MODE_CLUSTER_SIZE = "cluster_size"


class FrontierCluster:
    """One cluster of connected frontier cells (grid coordinates, not world)."""

    __slots__ = ("cells",)

    def __init__(self, cells: list[Cell]) -> None:
        self.cells = cells

    def size(self) -> int:
        return len(self.cells)

    def centroid_cell(self) -> tuple[float, float]:
        """Mean (col, row). Fractional on purpose: rounding here would move the
        centroid by up to half a cell before the fallback walk even starts."""
        cols = sum(c[0] for c in self.cells) / len(self.cells)
        rows = sum(c[1] for c in self.cells) / len(self.cells)
        return cols, rows


@dataclass(frozen=True)
class GainConfig:
    """How I(p) is computed and what one frontier cell is worth in metres."""

    mode: str = MODE_REDUCED_RANGE
    radius_m: float = 0.75
    weight_m_per_cell: float = 0.10

    def __post_init__(self) -> None:
        if self.mode not in (MODE_REDUCED_RANGE, MODE_CLUSTER_SIZE):
            raise ValueError(f"unknown information gain mode: {self.mode!r}")
        if self.radius_m <= 0.0:
            raise ValueError("information gain radius must be positive")


@dataclass
class Candidate:
    """One surviving cluster, its goal, and what the planner made of it."""

    cluster: FrontierCluster
    goal: Point
    info_gain: float
    path: list[Point] | None = None
    path_length: float = math.inf
    score: float = math.inf
    edges: list[tuple[Point, Point]] = field(default_factory=list)

    @property
    def reachable(self) -> bool:
        return self.path is not None


@dataclass
class ExplorationDecision:
    """The outcome of one selection cycle.

    `chosen` is None when nothing survived, which is the "explored enough"
    signal. `candidates` carries every cluster that was scored, reachable or
    not, because that is what the RViz markers draw and what makes a run
    explicable afterwards rather than a robot that moved for no stated reason.
    """

    chosen: Candidate | None
    candidates: list[Candidate] = field(default_factory=list)
    clusters_found: int = 0
    kept_committed: bool = False

    @property
    def path(self) -> list[Point] | None:
        return self.chosen.path if self.chosen is not None else None


class FrontierField:
    """The frontier grid, plus its cells as arrays, for radius queries.

    Built once per replan cycle and reused by every candidate. I(p) under the
    reduced-range form is a radius query per candidate, and rebuilding the
    coordinate array inside each one would make the cost quadratic in the number
    of candidates for no reason.
    """

    __slots__ = ("map", "cells", "points", "_mask")

    def __init__(self, frontier_map: OccupancyMap) -> None:
        self.map = frontier_map
        self._mask = frontier_map.data == FRONTIER_VALUE
        rows, cols = np.nonzero(self._mask)
        self.cells: npt.NDArray[np.int64] = np.stack([cols, rows], axis=1)
        info = frontier_map.info
        self.points: npt.NDArray[np.float64] = np.stack(
            [
                info.origin_x + (cols + 0.5) * info.resolution,
                info.origin_y + (rows + 0.5) * info.resolution,
            ],
            axis=1,
        )

    @property
    def mask(self) -> npt.NDArray[np.bool_]:
        return self._mask

    def count_within(self, point: Point, radius_m: float) -> int:
        """Frontier cells within `radius_m` of `point`, from any cluster.

        The shrunken stand-in for a LiDAR sweep. A real sensor model would trace
        rays and stop at walls, so this over-counts frontier cells hidden around
        a corner. That is a deliberate simplification and the report says so:
        ray tracing per candidate per cycle costs far more than it buys when the
        radius is already only a fifth of the sensor's real range.
        """
        if self.points.shape[0] == 0:
            return 0
        deltas = self.points - np.asarray(point)
        return int(np.count_nonzero(np.einsum("ij,ij->i", deltas, deltas) <= radius_m ** 2))


def find_frontier_clusters(
    frontier_map: OccupancyMap, min_size: int = 5, max_size: int = 100
) -> list[FrontierCluster]:
    """
    Connected-component grouping on the frontier grid (the /frontiers
    OccupancyGrid from the given frontier_detector_node.py: value 100 =
    frontier cell, everything else 0).

    8-neighbourhood BFS, and the difference from 4 is not cosmetic.

    frontier_detector_node.py uses a 4-neighbourhood for a different question:
    whether one free cell is adjacent to unknown space. Grouping frontier cells
    into boundaries is a separate question, and a boundary that runs diagonally
    is still one boundary.

    Under 4-connectivity a diagonal run of frontier cells is not a cluster at
    all; it is N clusters of one cell, every one below `min_size` and discarded
    as noise. The robot then reports zero clusters and declares the maze explored
    while looking straight at a frontier. Diagonal frontiers are the normal case,
    not an edge case: the boundary of what a rotating LiDAR has seen is a curve,
    and a curve on a grid is a staircase.

    `max_size` stops a cluster growing, it does not discard it. The remaining
    cells of an oversized frontier are picked up by the next BFS as separate
    clusters, which is the behaviour wanted: a single 400-cell frontier along a
    long corridor becomes four candidates spread along it rather than one
    candidate at its centre, and the centre of a long corridor frontier is
    rarely where you want to drive.

    `min_size` does discard. It is the noise filter and a separate job from the
    cap: a lone frontier pixel is usually one scan ray that found a gap, and
    chasing it costs a whole replan cycle.
    """
    remaining = frontier_map.data == FRONTIER_VALUE
    height, width = remaining.shape
    clusters: list[FrontierCluster] = []

    # `remaining` is the single source of truth for what is still unclustered, and
    # the outer loop re-reads it rather than walking a list taken once at the
    # start. That matters because of the cap: a cluster that stops at max_size
    # releases the cells it queued but never popped, and with a fixed scan order
    # any released cell earlier in that order would never be seen again. A long
    # corridor frontier would quietly lose its first chunk.
    while True:
        pending = np.argwhere(remaining)
        if pending.size == 0:
            break
        start_row, start_col = int(pending[0][0]), int(pending[0][1])

        cells: list[Cell] = []
        queue: deque[tuple[int, int]] = deque([(start_row, start_col)])
        remaining[start_row, start_col] = False
        while queue and len(cells) < max_size:
            r, c = queue.popleft()
            cells.append((c, r))  # (col, row), to read like (x, y)
            for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1),
                           (-1, -1), (-1, 1), (1, -1), (1, 1)):
                nr, nc = r + dr, c + dc
                if 0 <= nr < height and 0 <= nc < width and remaining[nr, nc]:
                    remaining[nr, nc] = False
                    queue.append((nr, nc))
        for r, c in queue:
            remaining[r, c] = True

        if len(cells) >= min_size:
            clusters.append(FrontierCluster(cells))
    return clusters


def cluster_goal_point(
    cluster: FrontierCluster, grid: PlanningGrid, robot_pos: Point
) -> Point | None:
    """
    Centroid of the cluster's cells, converted to world coordinates.

    Fallback: if the centroid is not known-free in `grid`, walk from it toward
    `robot_pos` one cell at a time until a known-free point is found, and return
    that instead. Concave clusters make plain centroids land on walls more often
    than you would expect: a frontier wrapping around a corner has its centroid
    inside the corner.

    Returns None when the whole walk back to the robot finds nothing free, which
    means the cluster is behind an inflated wall. The caller drops the candidate
    rather than handing the planner a goal it cannot reach.

    Note which grid this uses: the inflated `PlanningGrid`, not the raw map. A
    goal that is free on the raw map but inside a wall's inflation collar is a
    goal the planner is forbidden to reach, so accepting it here would produce a
    candidate that fails in `plan()` every cycle, forever.
    """
    info = grid.info
    col, row = cluster.centroid_cell()
    centroid = (
        info.origin_x + (col + 0.5) * info.resolution,
        info.origin_y + (row + 0.5) * info.resolution,
    )
    if grid.is_known_free(centroid):
        return centroid

    dx = robot_pos[0] - centroid[0]
    dy = robot_pos[1] - centroid[1]
    distance = math.hypot(dx, dy)
    if distance < 1e-9:
        return None
    step = info.resolution
    steps = int(distance / step)
    for i in range(1, steps + 1):
        t = (i * step) / distance
        probe = (centroid[0] + dx * t, centroid[1] + dy * t)
        if grid.is_known_free(probe):
            return probe
    return None


def information_gain(
    cluster: FrontierCluster,
    goal: Point,
    field_: FrontierField,
    config: GainConfig,
) -> float:
    """
    I(p) for this candidate, in frontier cells.

    `reduced_range`: every frontier cell within `config.radius_m` of the goal,
    from any cluster. This is the lab instructions' reduced-LiDAR-range tip and
    the form the report's equations describe.

    `cluster_size`: the candidate's own cell count, the simpler form the
    simpler alternative. Kept so the two can be compared on the same maze.
    """
    if config.mode == MODE_CLUSTER_SIZE:
        return float(cluster.size())
    return float(field_.count_within(goal, config.radius_m))


def score_path(path: list[Point], info_gain: float, config: GainConfig) -> float:
    """
    H(p) = sum(d(p)) - w * I(p). Lower is better; the caller takes the minimum.

    d(p) is the sum of straight-line distances between consecutive waypoints of
    the REAL path RRT* returned, not a straight line to the cluster. That is
    what makes a frontier on the far side of a wall score as far away rather
    than near, which a straight-line estimate gets exactly backwards.

    `w` is `config.weight_m_per_cell`, and it exists because slide 58's formula
    subtracts a cell count from a distance and those are not the same unit. A
    report that writes H = sum(d) - I without saying what converts them has an
    equation that cannot be evaluated.
    """
    return _path_length(path) - config.weight_m_per_cell * info_gain


def select_best_frontier(
    frontier_map: OccupancyMap,
    grid: PlanningGrid,
    robot_pose: Point,
    planner: RRTStarPlanner,
    config: GainConfig | None = None,
    min_cluster_size: int = 5,
    max_cluster_size: int = 100,
    max_candidates: int = 6,
    committed_goal: Point | None = None,
    switch_margin: float = 0.5,
    commit_match_radius_m: float = 0.30,
    exhausted_goals: Sequence[Point] = (),
    exhaust_radius_m: float = 0.30,
) -> ExplorationDecision:
    """
    Main entry point, called from navigation_node.py's planning tick.

    1. Cluster the frontier grid.
    2. If nothing survives, return a decision with `chosen` None. That is the
       "explored enough, stop" signal and the navigation node acts on it rather
       than swallowing it.
    3. Give every cluster a goal point, dropping any whose fallback walk fails.
    4. Drop any goal within `exhaust_radius_m` of a goal in `exhausted_goals`,
       then keep the `max_candidates` nearest by straight-line distance. This is
       slide 59's "limit the number of candidate solutions", and it is what
       keeps a full RRT* per candidate affordable inside one replan period.
       Straight-line distance is fine as a pre-filter precisely because the
       real path length is computed for everything that survives it.
    5. Plan a real RRT* path to each, drop the unreachable, and score with
       H = sum(d) - w * I.
    6. Apply the commitment: an incumbent goal is only displaced by a rival that
       beats it by more than `switch_margin`.
    7. Return the winner, and every candidate that was scored, for the markers.

    Every tuning number is an explicit argument rather than a default buried
    here, so that it lives in the YAML file and can be read back off a running
    node with `ros2 param get`.
    """
    config = config if config is not None else GainConfig()

    if frontier_map.info != grid.info:
        # The frontier detector copies msg.info straight off the map, so these
        # agree by construction, but only while both messages come from the same
        # map update. They arrive on separate topics with no synchronisation, so
        # after a SLAM loop closure resizes the map there is a window where they
        # do not. Cell indices would silently mean different places, which is the
        # kind of bug that looks like a tuning problem for an hour.
        raise ValueError("frontier grid and planning grid do not index the same cells")

    clusters = find_frontier_clusters(frontier_map, min_cluster_size, max_cluster_size)
    if not clusters:
        return ExplorationDecision(None, [], 0)

    field_ = FrontierField(frontier_map)

    goals: list[tuple[FrontierCluster, Point]] = []
    for cluster in clusters:
        goal = cluster_goal_point(cluster, grid, robot_pose)
        if goal is not None:
            goals.append((cluster, goal))
    if not goals:
        return ExplorationDecision(None, [], len(clusters))

    # Exhausted goals are places the robot has already been, or has already
    # failed to reach. Dropping them here rather than after scoring saves a full
    # RRT* per excluded candidate, and more importantly it is the only thing
    # that breaks the arrive-reselect-arrive loop: a frontier that survives the
    # robot standing next to it will keep winning on distance forever, because
    # nothing else in H knows the robot has been there.
    if exhausted_goals:
        goals = [
            (cluster, goal) for cluster, goal in goals
            if all(math.hypot(goal[0] - e[0], goal[1] - e[1]) > exhaust_radius_m
                   for e in exhausted_goals)
        ]
        if not goals:
            return ExplorationDecision(None, [], len(clusters))

    goals.sort(key=lambda pair: math.hypot(
        pair[1][0] - robot_pose[0], pair[1][1] - robot_pose[1]))
    goals = goals[:max_candidates]

    candidates: list[Candidate] = []
    for cluster, goal in goals:
        gain = information_gain(cluster, goal, field_, config)
        candidate = Candidate(cluster=cluster, goal=goal, info_gain=gain)
        result = planner.plan(robot_pose, goal)
        if result.path is not None:
            candidate.path = result.path
            candidate.path_length = result.length
            candidate.score = score_path(result.path, gain, config)
            candidate.edges = result.edges
        candidates.append(candidate)

    reachable = [c for c in candidates if c.reachable]
    if not reachable:
        return ExplorationDecision(None, candidates, len(clusters))

    best = min(reachable, key=lambda c: c.score)

    # Hysteresis. H is memoryless: as the robot drives toward cluster A, d
    # shrinks for A but the map grows behind it and I can flip the ranking to B
    # and back, which is the frontier oscillation the plan draft named as
    # anticipated challenge number one. An incumbent keeps the goal unless a
    # rival is better by a stated margin.
    incumbent = _incumbent(reachable, committed_goal, commit_match_radius_m)
    if incumbent is not None and best is not incumbent:
        if best.score > incumbent.score - switch_margin:
            return ExplorationDecision(incumbent, candidates, len(clusters), True)

    return ExplorationDecision(best, candidates, len(clusters), incumbent is best)


def _incumbent(
    candidates: list[Candidate], committed_goal: Point | None, radius_m: float
) -> Candidate | None:
    """The candidate representing the goal already committed to, if it survives.

    Matched by proximity rather than identity because the goal point moves a
    little every cycle: the cluster's cells change as the map fills in, so its
    centroid drifts even when it is unmistakably the same frontier.
    """
    if committed_goal is None:
        return None
    best: Candidate | None = None
    best_distance = radius_m
    for candidate in candidates:
        distance = math.hypot(
            candidate.goal[0] - committed_goal[0], candidate.goal[1] - committed_goal[1]
        )
        if distance <= best_distance:
            best, best_distance = candidate, distance
    return best


if __name__ == "__main__":
    # A room whose right half is unknown, so the frontier is the vertical seam
    # down the middle. Confirms clustering and scoring pick something sane
    # before any of this touches a robot.
    from .grid import OccupancyMap as _Map

    WIDTH, HEIGHT = 60, 40
    occupancy = np.zeros((HEIGHT, WIDTH), dtype=np.int16)
    occupancy[:, 0] = occupancy[0, :] = occupancy[-1, :] = 100
    occupancy[:, 30:] = -1
    grid_map = _Map.from_rows(occupancy, resolution=0.05)

    frontier = np.zeros((HEIGHT, WIDTH), dtype=np.int16)
    frontier[5:35, 29] = FRONTIER_VALUE
    frontier_map = _Map.from_rows(frontier, resolution=0.05)

    planning = PlanningGrid(grid_map, inflation_radius_m=0.105)
    decision = select_best_frontier(
        frontier_map, planning, (0.4, 1.0), RRTStarPlanner(planning, max_plan_time_s=2.0)
    )
    print(f"clusters {decision.clusters_found}, scored {len(decision.candidates)}")
    if decision.chosen is None:
        print("nothing chosen")
    else:
        c = decision.chosen
        print(f"goal {c.goal}, I={c.info_gain:.0f}, d={c.path_length:.2f} m, H={c.score:.2f}")
