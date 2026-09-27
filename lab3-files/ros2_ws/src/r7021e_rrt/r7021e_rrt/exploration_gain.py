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

Two things the skeleton left open, settled 2026-09-23
------------------------------------------------------
I(p) is the number of frontier cells within `radius_m` of the candidate goal,
counted across the WHOLE frontier grid rather than within the candidate's own
cluster, with `radius_m` far below the Burger's 3.5 m LiDAR range. That is the
lab instructions' own tip, verbatim: "greatly reduce the LiDAR range for info
gain computation to drive exploratory behavior". Counting only the candidate's
own cluster is available as `mode="cluster_size"` so the two can be measured
against each other rather than argued about, and the report carries the
measurement.

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

# How much further past the first free cell to keep looking for a roomier
# goal. Half a corridor width: far enough to leave a wall, near enough that
# the goal still belongs to the frontier that generated it.
GOAL_CLEARANCE_WALKBACK_M = 0.35

# How far the fallback may walk from a blocked centroid before giving up on the
# cluster entirely. A goal is meant to be a viewpoint of its own frontier, and a
# point further than this from the cluster is not one.
#
# Without the cap the walk ran the whole way to the robot. On a map whose walls
# are drawn about 2.5 times too thick, a frontier centroid's entire neighbourhood
# is often blocked, so the walk terminated at the robot's feet. Replaying four
# recorded runs through this function: 11 of 13 clusters in one cycle produced
# goal points inside a 0.6 m blob around the robot, 1.0 to 3.0 m from their own
# cluster, and between 27 and 45 percent of all cluster instances across the runs
# produced a goal seeing zero frontier cells. Under this cap, none do.
#
# Those detached goals then won, because select_best_frontier keeps the six
# NEAREST candidates before scoring and a goal at the robot's feet is the nearest
# thing there is. H collapses to its distance term, the robot drove four
# centimetres, declared arrival inside the 0.20 m tolerance, and retired a 0.30 m
# exhaustion disc around where it already stood. Once those discs covered the
# track every cluster was filtered out before planning, which is the code path
# that logs "N clusters found, 0 unreachable". Eight of sixteen runs ended there.
#
# 0.5 m is a little over the 0.35 m clearance walkback and well inside the 0.75 m
# gain radius, so a goal that survives it still sees its own frontier.
# Evidence tier: replay, 2026-09-25.
GOAL_WALKBACK_MAX_M = 0.5

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
    # Metres of equivalent path charged per radian the robot must turn before it
    # can start down a candidate path. Zero disables it.
    turn_cost_m_per_rad: float = 0.0

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

    frontier_detector_node.py uses a 4-neighbourhood, but for a different
    question: whether a free cell is *adjacent to unknown*, which is a statement
    about that one cell. Grouping frontier cells into boundaries is a second,
    separate question, and a boundary that runs diagonally is still one boundary.

    Under 4-connectivity a diagonal run of frontier cells is not a cluster at
    all; it is N clusters of one cell each, every one of them below `min_size`
    and therefore discarded as noise. That is not hypothetical. On
    lab3_maze_small the run terminated with roughly ten frontier cells still
    visible in RViz, stepping diagonally across the map's lower left, and the
    node correctly reported zero clusters and declared the maze explored with
    the entire right half of it unmapped. Diagonal frontiers are the normal case
    rather than an edge case, because the boundary of what a rotating LiDAR has
    seen is a curve, and a curve on a grid is a staircase.
    Evidence tier: sim, lab3_maze_small, 2026-09-23.

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
    # the outer loop re-reads it every time rather than walking a list of cells
    # taken once at the start. That matters only because of the cap: a cluster
    # that stops at max_size puts the cells it queued but never popped back into
    # `remaining`, and with a precomputed scan order any released cell earlier in
    # that order would never be looked at again. The result is a long corridor
    # frontier that quietly loses its first chunk, which is invisible in a total
    # cell count and shows up as a robot that will not go back for a gap it
    # already walked past.
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

    Fallbacks, in order, when the centroid is not known-free in `grid`: the
    roomiest known-free cell of the cluster itself, then a walk from the centroid
    toward `robot_pos` one cell at a time until a known-free point is found. Concave clusters make plain centroids land on walls more often
    than you would expect: a frontier wrapping around a corner has its centroid
    inside the corner.

    The walk is capped at `GOAL_WALKBACK_MAX_M`. Returns None when nothing free
    is found inside it, which means the cluster is behind an inflated wall. The
    caller drops the candidate rather than handing the planner a goal that is not
    a viewpoint of the frontier it came from. See the constant for what happened
    when the walk was uncapped.

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

    # The cluster's own cells do not depend on where the robot stands, so the goal
    # stays put between cycles. Cells reachable through mapped space rank first,
    # because a cluster can straddle a strip of unknown the planner may not cross.
    region = grid.known_free_region(robot_pos)
    own: Point | None = None
    own_rank = (False, -1.0)
    for cell in cluster.cells:
        probe = info.cell_to_world(cell)
        if grid.is_known_free(probe):
            rank = (region is not None and bool(region[cell[1], cell[0]]),
                    grid.clearance(probe))
            if rank > own_rank:
                own, own_rank = probe, rank
    if own is not None:
        return own

    dx = robot_pos[0] - centroid[0]
    dy = robot_pos[1] - centroid[1]
    distance = math.hypot(dx, dy)
    if distance < 1e-9:
        return None
    step = info.resolution
    # Bounded by the cap as well as by the robot, so a cluster whose surroundings
    # are all blocked is dropped rather than turned into a goal somewhere else.
    steps = int(min(distance, GOAL_WALKBACK_MAX_M) / step)
    # Walk back toward the robot and take the first known-free point, then keep
    # walking a little further and take the roomiest point found instead.
    #
    # The first free point is, by construction, the one closest to whatever
    # blocked the centroid, so it sits against a wall with the bare collar for
    # clearance. Measured in simulation, that is where the robot was sent and
    # where it drove into the real wall: every failed run made first contact
    # within a few centimetres of the same frontier goal. The collar is
    # computed from the map, and at a frontier the map is exactly where it is
    # least trustworthy, so a goal that merely clears the collar clears nothing
    # reliable. Preferring the roomiest point costs a fraction of a metre of
    # travel and moves the goal off the wall.
    best: Point | None = None
    best_clearance = -1.0
    extra = int(GOAL_CLEARANCE_WALKBACK_M / step)
    for i in range(1, steps + 1):
        t = (i * step) / distance
        probe = (centroid[0] + dx * t, centroid[1] + dy * t)
        if grid.is_known_free(probe):
            room = grid.clearance(probe)
            if room > best_clearance:
                best, best_clearance = probe, room
            if best is not None:
                extra -= 1
                if extra <= 0:
                    return best
    return best


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
    skeleton proposed. Kept so the two can be compared on the same maze rather
    than argued about on a slide.
    """
    if config.mode == MODE_CLUSTER_SIZE:
        return float(cluster.size())
    return float(field_.count_within(goal, config.radius_m))


def score_path(path: list[Point], info_gain: float, config: GainConfig,
               robot_yaw: float | None = None) -> float:
    """
    H(p) = sum(d(p)) + t * |turn| - w * I(p). Lower is better.

    d(p) is the sum of straight-line distances between consecutive waypoints of
    the REAL path RRT* returned, not a straight line to the cluster. That is
    what makes a frontier on the far side of a wall score as far away rather
    than near, which a straight-line estimate gets exactly backwards.

    `w` is `config.weight_m_per_cell`, and it exists because slide 58's formula
    subtracts a cell count from a distance and those are not the same unit. A
    report that writes H = sum(d) - I without saying what converts them has an
    equation that cannot be evaluated.

    The turn term is the same argument applied again. Two candidates 2 m away,
    one straight ahead and one directly behind, cost the same in d and are not
    the same cost, because the robot has to stop and swing 180 degrees to start
    on the second. `turn_cost_m_per_rad` converts that rotation into the metres
    it displaces: at `max_v` 0.15 m/s and `max_w` 1.0 rad/s the robot covers
    0.15 m in the time it takes to turn one radian, so 0.15 is the value with a
    derivation rather than a preference behind it, and a half turn then costs
    0.47 m of equivalent path.

    Only the first segment is charged. Turns further along the path are real
    costs too, but they are costs the path already has regardless of which
    candidate wins, and charging them would penalise a long path twice.
    """
    cost = _path_length(path) - config.weight_m_per_cell * info_gain
    if (robot_yaw is not None and config.turn_cost_m_per_rad > 0.0
            and len(path) >= 2):
        heading = math.atan2(path[1][1] - path[0][1], path[1][0] - path[0][0])
        turn = abs(math.remainder(heading - robot_yaw, 2.0 * math.pi))
        cost += config.turn_cost_m_per_rad * turn
    return cost


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
    robot_yaw: float | None = None,
    retry_plan_time_scale: float = 0.0,
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

    The signature grew past the skeleton's four arguments because every one of
    the extra ones is a number the report has to state. Passing them explicitly
    keeps them in the YAML file, where they can be read back off a running node,
    rather than as defaults buried in a function nobody launches.
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
        # A goal that sees none of its own frontier is not an exploration goal.
        # It can only be an artefact of goal placement, and it beats every real
        # candidate because H collapses to its distance term. Dropping it here
        # rather than scoring it is the difference between the robot driving to a
        # frontier and the robot driving four centimetres to a point it is
        # already standing on.
        if gain <= 0.0:
            continue
        candidate = Candidate(cluster=cluster, goal=goal, info_gain=gain)
        result = planner.plan(robot_pose, goal)
        if result.path is not None:
            candidate.path = result.path
            candidate.path_length = result.length
            candidate.score = score_path(result.path, gain, config, robot_yaw)
            candidate.edges = result.edges
        candidates.append(candidate)

    reachable = [c for c in candidates if c.reachable]

    # Nothing reachable inside the ordinary budget is not the same statement as
    # nothing reachable. RRT* is probabilistically complete, so a failure at
    # 0.15 s of sampling is a failure to find a path in 0.15 s, and the cost of
    # believing it is the run: a cycle with no reachable candidate is what starts
    # the look-around, the unstick, and eventually the termination countdown.
    #
    # The replan period is 1.0 s and six candidates at 0.15 s use at most 0.9 s
    # of it, so on the cycles where it matters there is budget sitting unused.
    # This spends it, once, only when the alternative is giving up.
    # Both caps scale: at 1500 iterations the planner stops before its clock does,
    # so scaling only the clock changed nothing.
    if not reachable and retry_plan_time_scale > 1.0:
        budget = planner.max_plan_time_s
        iterations = planner.max_iterations
        planner.max_plan_time_s = budget * retry_plan_time_scale
        planner.max_iterations = int(iterations * retry_plan_time_scale)
        try:
            for candidate in candidates:
                result = planner.plan(robot_pose, candidate.goal)
                if result.path is not None:
                    candidate.path = result.path
                    candidate.path_length = result.length
                    candidate.score = score_path(result.path,
                                                 candidate.info_gain, config,
                                                 robot_yaw)
                    candidate.edges = result.edges
        finally:
            planner.max_plan_time_s = budget
            planner.max_iterations = iterations
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
