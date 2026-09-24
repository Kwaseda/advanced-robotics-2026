"""
rrt_star.py

Plain-Python RRT* planner. No ROS imports here on purpose: this needs to be
testable against a made-up/synthetic occupancy grid, without a robot or a
lab room. navigation_node.py is the only file that should import rclpy.

Course reference: R7021E-Lecture_5_to_7_Navigation.pdf, slide 33 (baseline
RRT, 8 steps), slide 35 (RRT*: same 8 steps, then "extract all branches to
goal, select the shortest").

Decisions already locked in for this lab (from the plan conversation):
- Obstacle safety = map inflation (0.105 m), which lives in grid.PlanningGrid
  and is consumed by the collision check below. No separate potential-fields
  system, no risk term anywhere in the scoring (that lives in
  exploration_gain.py, and it doesn't have one either, on purpose).
- Compute time is explicitly NOT graded (see Lab_Instructions_Exploration.pdf),
  so it's fine for this planner to get called multiple times per replan
  cycle, once per surviving frontier candidate, instead of just once.

Three implementation notes
--------------------------
1. The tree is an `RRTTree` object rather than a plain list, only so the nearest
   and near-neighbour searches can run in numpy. They are still exhaustive scans
   over every node with no spatial index; a Python loop over 1500 nodes took
   about 0.9 s per plan against 0.06 s vectorised, and six candidate clusters per
   second does not fit in the first.

2. Nodes cache their cost from the root and keep a list of children. Rewiring
   asks for the cost of every nearby node on every iteration, so recomputing it
   by walking parent pointers would be the inner loop. The children list is what
   lets a re-parent push the corrected cost down its whole subtree, which is the
   part of RRT* that is easy to omit and invisible from outside.

3. `plan()` does not return the first branch that reaches the goal. Slide 35 is
   explicit: "extract all branches to goal, select the shortest". The tree keeps
   growing after the first success and the best goal-connected node wins, bounded
   by `extra_iterations_after_solution` so the replan period still means
   something.
"""

from __future__ import annotations

import math
import random
import time
from dataclasses import dataclass, field
from typing import Iterator, Sequence

import numpy as np
import numpy.typing as npt

from .grid import PlanningGrid

Point = tuple[float, float]


class RRTStarNode:
    """One node in the tree.

    `cost` is the path length from the root along parent pointers, kept current
    rather than recomputed; see note 2 in the module docstring.

    `unknown_run` is the distance this node sits beyond the last known-free
    ground on its own branch, in metres. It is 0.0 for any node standing on a
    mapped free cell. It exists so that `unknown_lookahead_m` can be enforced
    per branch rather than per edge: without carrying it forward, a chain of
    short steps each individually under the cap would walk arbitrarily far into
    unmapped space.
    """

    __slots__ = ("point", "parent", "cost", "unknown_run", "children")

    def __init__(
        self,
        point: Point,
        parent: RRTStarNode | None = None,
        cost: float = 0.0,
        unknown_run: float = 0.0,
    ) -> None:
        self.point = point
        self.parent = parent
        self.cost = cost
        self.unknown_run = unknown_run
        self.children: list[RRTStarNode] = []


class RRTTree(Sequence[RRTStarNode]):
    """The tree, plus a preallocated array of its points for the scans.

    A `Sequence`, so it still indexes, iterates and reports `len()` exactly like
    a plain list would. The array is an implementation detail of making the
    exhaustive scans fast, not a change of algorithm.
    """

    __slots__ = ("_nodes", "_coords")

    def __init__(self, capacity: int) -> None:
        self._nodes: list[RRTStarNode] = []
        self._coords: npt.NDArray[np.float64] = np.empty((max(capacity, 1), 2))

    def __len__(self) -> int:
        return len(self._nodes)

    def __getitem__(self, index: int) -> RRTStarNode:  # type: ignore[override]
        return self._nodes[index]

    def __iter__(self) -> Iterator[RRTStarNode]:
        return iter(self._nodes)

    @property
    def coords(self) -> npt.NDArray[np.float64]:
        """View of the node points, shape (len(self), 2), in insertion order."""
        return self._coords[: len(self._nodes)]

    def add(self, node: RRTStarNode) -> RRTStarNode:
        index = len(self._nodes)
        if index >= self._coords.shape[0]:
            # Growing should not happen: capacity is sized from max_iterations.
            # Doubling rather than raising keeps a mis-sized capacity from being
            # a crash on the robot.
            self._coords = np.vstack([self._coords, np.empty_like(self._coords)])
        self._coords[index] = node.point
        self._nodes.append(node)
        if node.parent is not None:
            node.parent.children.append(node)
        return node


@dataclass
class PlanResult:
    """What one call to `plan()` produced, including the parts only RViz wants.

    `path` is the answer. `edges` and `iterations` exist so the navigation node
    can draw the tree that produced it and log how hard it worked, without the
    planner having to know that RViz exists.
    """

    path: list[Point] | None
    length: float = math.inf
    iterations: int = 0
    elapsed_s: float = 0.0
    edges: list[tuple[Point, Point]] = field(default_factory=list)

    @property
    def succeeded(self) -> bool:
        return self.path is not None


class RRTStarPlanner:
    """
    Plans a single path from `start` to `goal` on an inflated occupancy grid.

    Read slide 33 (baseline RRT) before reading plan(). RRT* (slide 35) is the
    same 8 steps, plus choosing the cheapest available parent for each new node
    and rewiring nearby nodes through it as the tree grows.
    """

    def __init__(
        self,
        grid: PlanningGrid,
        max_iterations: int = 1500,
        step_size_m: float = 0.30,
        goal_sample_rate: float = 0.10,
        goal_tolerance_m: float = 0.15,
        rewire_radius_m: float = 0.60,
        unknown_lookahead_m: float = 0.50,
        sample_bounds: tuple[float, float, float, float] | None = None,
        start_search_radius_m: float = 0.60,
        extra_iterations_after_solution: int = 200,
        max_plan_time_s: float = 0.15,
        rng: random.Random | None = None,
    ) -> None:
        """
        grid: a `PlanningGrid`, which is the shared representation decided in
            grid.py and used identically by exploration_gain.py. It has already
            had the inflation applied, once per replan cycle rather than once
            per candidate, because the navigation loop constructs one of these
            and hands it to every planner in that cycle.

        step_size_m: 0.30 m, six cells at the SLAM map's 0.05 m resolution.
            Short enough that a step cannot jump a 0.2 m maze wall even before
            the along-the-line check, long enough that crossing an 8 m maze does
            not need thousands of nodes.

        rewire_radius_m: 0.60 m, twice the step size. The textbook RRT* radius
            shrinks as log(n)/n, which is what gives the asymptotic optimality
            proof. A fixed radius is used here instead because the proof is
            about the limit and this planner is explicitly budgeted, and because
            one number in a YAML file with a reason next to it is easier to
            defend in an oral than a formula whose constant nobody tuned.

        unknown_lookahead_m: how far a branch may run through unmapped space
            before it must touch known-free ground again. See grid.PlanningGrid
            for why unknown is traversable at all.

        start_search_radius_m: how far to look for a traversable cell when the
            robot's own pose is not one. See `PlanningGrid.nearest_unblocked`.
            0.60 m is a little under one maze cell, so the search cannot reach
            past the corridor the robot is standing in.

        sample_bounds: (min_x, min_y, max_x, max_y) to sample inside, or None
            for the whole map, which is the default.

            Restricting samples to a box around start and goal is the usual
            refinement and is wrong here: a maze path is the opposite of direct,
            so a box drawn from the endpoints excludes exactly the detour that
            makes the path exist. A 4 by 3 m room whose only gap is at x = 3.0,
            with start and goal both at x = 0.5, finds no path at all under a
            1.5 m margin. The argument stays for callers that genuinely know the
            bounds, and for keeping test trees small.

        max_plan_time_s: a wall-clock budget per plan. Compute time is not
            graded, but the navigation loop still ticks at a fixed period and
            plans once per candidate cluster, so an unbounded plan turns into a
            loop that silently stops replanning. Exceeding it is not an error:
            the best path found so far is returned.
        """
        self.grid = grid
        self.max_iterations = max_iterations
        self.step_size_m = step_size_m
        self.goal_sample_rate = goal_sample_rate
        self.goal_tolerance_m = goal_tolerance_m
        self.rewire_radius_m = rewire_radius_m
        self.unknown_lookahead_m = unknown_lookahead_m
        self.sample_bounds = sample_bounds
        self.start_search_radius_m = start_search_radius_m
        self.extra_iterations_after_solution = extra_iterations_after_solution
        self.max_plan_time_s = max_plan_time_s
        self._rng = rng if rng is not None else random.Random()

        # Collision checks walk a segment at half-cell spacing. Half a cell, not
        # a whole one, because a line crossing a cell's corner can otherwise step
        # from one side of a one-cell-thick wall to the other without ever
        # landing inside it. See slides 46-49 on volumetric collision checks.
        self._check_step_m = 0.5 * grid.info.resolution

        self._sample_box: tuple[float, float, float, float] = sample_bounds or (
            grid.info.min_x, grid.info.min_y, grid.info.max_x, grid.info.max_y
        )
        self._goal: Point = (0.0, 0.0)
        # Points prepended to a returned path, when the tree had to be rooted
        # away from the robot's own pose. Set per call by plan().
        self._prefix: list[Point] = []
        # Set by plan() to the start point when the robot is standing inside an
        # inflated wall; see _check_segment for what it licenses.
        self._escape_point: Point | None = None

    # ---------------------------------------------------------------- planning

    def plan(self, start: Point, goal: Point) -> PlanResult:
        """
        Main entry point. Returns a `PlanResult` whose `path` is a list of
        waypoints from start to goal, or None if no path was found.

        Steps, mapped to slides 33 and 35:
        1. Initialize tree at start.
        2. Loop up to max_iterations:
           a. sample = sample_random_point()
           b. nearest = nearest_node(tree, sample)
           c. new_point = steer(nearest.point, sample, step_size_m)
           d. if is_collision_free(nearest.point, new_point): attach new_point,
              choosing the cheapest collision-free parent among nearby nodes
              rather than `nearest` automatically. That choice is half of what
              makes this RRT* and not RRT; rewire() is the other half.
           e. rewire(tree, new_node)
           f. if new_point is within goal_tolerance_m of goal, remember it as a
              goal candidate and keep going.
        3. Return the shortest goal-connected branch, or None.

        `None` is a real answer and the caller must handle it. A frontier goal
        behind a wall SLAM has not finished closing is a perfectly ordinary
        thing to ask for, and exploration_gain.py drops those candidates rather
        than treating the cycle as failed.
        """
        started = time.monotonic()
        self._goal = goal

        # A start pose that is not traversable is not a bug in the planner and not
        # rare: a 0.7 m maze corridor is narrower than twice the 0.15 m collar
        # plus the robot, so driving down the middle of one already puts the
        # robot inside the collar. Refusing to plan would stop the run.
        #
        # Two mechanisms, and both are needed for different depths of the same
        # problem. `_escape_point` licenses a leading blocked prefix on edges
        # leaving the root, bounded so it can leave a collar and cannot cross a
        # wall; that covers the ordinary corridor case without any search.
        #
        # It does not cover a start sealed into a blocked pocket deeper than that
        # bound, which happens where the map carries a diffuse scatter of
        # spurious occupied cells: each inflates to a 7 by 7 blob and a region
        # with them two cells apart has no traversable cell in it at all. There
        # the tree is rooted at the nearest genuinely reachable point instead,
        # and the robot's own pose is put back on the front of the path so the
        # follower still starts from where the robot is.
        root_point = start
        self._prefix: list[Point] = []
        self._escape_point = None
        if self.grid.is_blocked(start):
            self._escape_point = start
            free_ok, _ = self._check_segment(start, self.steer(
                start, goal, self.step_size_m), 0.0)
            if not free_ok:
                relocated = self.grid.nearest_unblocked(
                    start, self.start_search_radius_m)
                if relocated is not None:
                    root_point, self._prefix = relocated, [start]
                    self._escape_point = None
        capacity = self.max_iterations + 2
        tree = RRTTree(capacity)
        # The root's unknown run is 0.0 even when the robot is standing on an
        # unknown cell, which happens on the very first map. Charging the robot
        # for the ground it is already on would shrink the budget for the ground
        # ahead, and it has demonstrably survived where it is.
        root = RRTStarNode(root_point, parent=None, cost=0.0, unknown_run=0.0)
        tree.add(root)

        best_goal_node: RRTStarNode | None = None
        best_goal_cost = math.inf
        solved_at: int | None = None
        iterations = 0

        for iterations in range(1, self.max_iterations + 1):
            if iterations % 50 == 0 and time.monotonic() - started > self.max_plan_time_s:
                break
            if (
                solved_at is not None
                and iterations - solved_at > self.extra_iterations_after_solution
            ):
                break

            sample = self.sample_random_point()
            nearest = self.nearest_node(tree, sample)
            new_point = self.steer(nearest.point, sample, self.step_size_m)

            free, run = self._check_segment(nearest.point, new_point, nearest.unknown_run)
            if not free:
                continue

            new_node = self._insert_with_best_parent(tree, nearest, new_point, run)
            self.rewire(tree, new_node)

            if _distance(new_node.point, goal) <= self.goal_tolerance_m:
                if solved_at is None:
                    solved_at = iterations
                if new_node.cost < best_goal_cost:
                    best_goal_cost, best_goal_node = new_node.cost, new_node

        edges = [
            (node.parent.point, node.point) for node in tree if node.parent is not None
        ]
        elapsed = time.monotonic() - started

        if best_goal_node is None:
            return PlanResult(None, math.inf, iterations, elapsed, edges)

        # Rewiring may have shortened the winning branch after it was recorded,
        # so the cost is read off the node now rather than trusted from then.
        path = self._prefix + self.extract_path(best_goal_node)
        # The goal itself is appended when the branch ended short of it. The
        # final hop is within goal_tolerance_m and was never collision checked
        # as an edge, so it is only appended when it actually passes one.
        if _distance(path[-1], goal) > 1e-9:
            ok, _ = self._check_segment(path[-1], goal, best_goal_node.unknown_run)
            if ok:
                path.append(goal)
        return PlanResult(path, _path_length(path), iterations, elapsed, edges)

    # ------------------------------------------------------------- the 8 steps

    def sample_random_point(self) -> Point:
        """
        Sample a point in free space. With probability `goal_sample_rate`,
        return the goal itself instead, so the tree does not wander forever
        before happening to reach it.

        Rejection sampling against blocked cells, capped at a fixed number of
        tries. The cap matters: in a nearly-full map almost every sample is
        rejected, and an uncapped loop would hang the navigation node rather
        than fail a plan. Falling through the cap returns a blocked point, which
        the collision check then rejects, costing one wasted iteration instead
        of the whole session.
        """
        if self._rng.random() < self.goal_sample_rate:
            return self._goal

        min_x, min_y, max_x, max_y = self._sample_box
        point = (min_x, min_y)
        for _ in range(20):
            point = (
                self._rng.uniform(min_x, max_x),
                self._rng.uniform(min_y, max_y),
            )
            if not self.grid.is_blocked(point):
                return point
        return point

    def nearest_node(self, tree: RRTTree, point: Point) -> RRTStarNode:
        """
        Exhaustive scan over every node. No kd-tree, no spatial hash; the scan
        just runs in numpy rather than a Python loop. See note 1 in the module
        docstring for the measurement behind that.
        """
        coords = tree.coords
        deltas = coords - np.asarray(point)
        index = int(np.argmin(np.einsum("ij,ij->i", deltas, deltas)))
        return tree[index]

    def steer(self, from_point: Point, to_point: Point, step_size: float) -> Point:
        """
        Move `step_size` metres from from_point toward to_point. If to_point is
        already closer than step_size, return to_point itself rather than
        overshooting past it.
        """
        dx = to_point[0] - from_point[0]
        dy = to_point[1] - from_point[1]
        distance = math.hypot(dx, dy)
        if distance <= step_size or distance == 0.0:
            return to_point
        scale = step_size / distance
        return (from_point[0] + dx * scale, from_point[1] + dy * scale)

    def is_collision_free(self, from_point: Point, to_point: Point) -> bool:
        """
        True when the robot may traverse the straight segment between these two
        points. This is where the 0.105 m inflation lives, via `PlanningGrid`.

        Occupancy is checked ALONG the line at half-cell spacing, not just at the
        endpoints: a 0.3 m step at 0.05 m resolution spans six cells and could
        otherwise pass clean through a one-cell wall. See slides 46-49 (Map
        Inflation, Volumetric/Area Collision Checks).

        The public form assumes the branch arrives on known-free ground, i.e. an
        unknown run of zero. `plan()` calls `_check_segment` directly so it can
        carry each branch's accumulated unknown travel forward.
        """
        free, _ = self._check_segment(from_point, to_point, 0.0)
        return free

    def rewire(self, tree: RRTTree, new_node: RRTStarNode) -> None:
        """
        The thing that makes this RRT* rather than plain RRT. For every existing
        node within `rewire_radius_m` of new_node, check whether routing through
        new_node gives it a shorter path back to the root than its current
        parent does. If so, and the connecting edge is collision free, re-parent
        it and push the corrected cost down its subtree.

        The subtree push is the part that is easy to omit. Without it a
        re-parented node reports the new, shorter cost while all of its
        descendants still report costs computed through the old parent, so later
        rewiring decisions compare a mix of current and stale numbers and the
        tree stops converging for reasons that are invisible from the outside.
        """
        for candidate in self._near_nodes(tree, new_node.point):
            if candidate is new_node or candidate is new_node.parent:
                continue
            if candidate.parent is None:
                continue  # never re-parent the root
            edge = _distance(new_node.point, candidate.point)
            improved = new_node.cost + edge
            if improved >= candidate.cost:
                continue
            free, run = self._check_segment(
                new_node.point, candidate.point, new_node.unknown_run
            )
            if not free:
                continue
            candidate.parent.children.remove(candidate)
            candidate.parent = new_node
            new_node.children.append(candidate)
            _propagate(candidate, improved, run)

    def extract_path(self, node: RRTStarNode) -> list[Point]:
        """Walk parent pointers back to the root, then reverse the list."""
        points: list[Point] = []
        current: RRTStarNode | None = node
        while current is not None:
            points.append(current.point)
            current = current.parent
        points.reverse()
        return points

    # ----------------------------------------------------------------- helpers

    def _insert_with_best_parent(
        self, tree: RRTTree, nearest: RRTStarNode, new_point: Point, run: float
    ) -> RRTStarNode:
        """Attach new_point under whichever nearby node makes it cheapest.

        RRT would attach it to `nearest` and stop. RRT* considers every node
        within the rewire radius and takes the cheapest collision-free one,
        which is why an RRT* path comes out smooth where an RRT path comes out
        jagged, and why the plan draft chose RRT* given that compute time is not
        graded.
        """
        best_parent = nearest
        best_cost = nearest.cost + _distance(nearest.point, new_point)
        best_run = run

        for candidate in self._near_nodes(tree, new_point):
            if candidate is nearest:
                continue
            cost = candidate.cost + _distance(candidate.point, new_point)
            if cost >= best_cost:
                continue
            free, candidate_run = self._check_segment(
                candidate.point, new_point, candidate.unknown_run
            )
            if free:
                best_parent, best_cost, best_run = candidate, cost, candidate_run

        return tree.add(
            RRTStarNode(new_point, parent=best_parent, cost=best_cost, unknown_run=best_run)
        )

    def _near_nodes(self, tree: RRTTree, point: Point) -> list[RRTStarNode]:
        """Every node within `rewire_radius_m`. Exhaustive scan, done in numpy."""
        coords = tree.coords
        deltas = coords - np.asarray(point)
        within = np.einsum("ij,ij->i", deltas, deltas) <= self.rewire_radius_m ** 2
        return [tree[int(i)] for i in np.flatnonzero(within)]

    def _check_segment(
        self, from_point: Point, to_point: Point, entry_unknown_run: float
    ) -> tuple[bool, float]:
        """Collision check plus unknown-run bookkeeping for one edge.

        Returns (traversable, unknown_run_at_to_point). The second value is the
        distance `to_point` lies beyond the last known-free ground along this
        branch, which the caller stores on the new node so the next edge can
        continue the accounting.

        Two ways to fail: any sampled point is blocked, or the running distance
        through unknown space exceeds `unknown_lookahead_m` anywhere along the
        segment. The second is what keeps "unknown is traversable" from becoming
        a licence to plan across an entire unmapped maze on no evidence.
        """
        distance = _distance(from_point, to_point)
        samples = max(1, int(math.ceil(distance / self._check_step_m)))
        ts = np.linspace(0.0, 1.0, samples + 1)
        xs = from_point[0] + ts * (to_point[0] - from_point[0])
        ys = from_point[1] + ts * (to_point[1] - from_point[1])

        info = self.grid.info
        cols = np.floor((xs - info.origin_x) / info.resolution).astype(np.int64)
        rows = np.floor((ys - info.origin_y) / info.resolution).astype(np.int64)

        off_map = (
            (cols < 0) | (cols >= info.width) | (rows < 0) | (rows >= info.height)
        )
        if bool(off_map.any()):
            # Off the map is blocked, not unknown. There is no evidence the space
            # exists, so the unknown-travel cap has nothing to bound. See
            # PlanningGrid.classify_point for the matching decision.
            return False, math.inf

        blocked = self.grid.blocked[rows, cols]
        travelled = ts * distance
        first = 0
        if self._escape_point is not None and from_point == self._escape_point:
            # Leading blocked samples are the collar the robot is standing in.
            # Skip them, but only up to twice the stamped collar radius.
            #
            # That bound is not arbitrary. The thinnest blocked region that
            # contains a real wall is the wall itself plus a full collar on each
            # side, so it is at least 2 * stamped_radius + one cell thick. A
            # budget of exactly 2 * stamped_radius therefore cannot carry an
            # edge through a wall, while still allowing the robot to leave a
            # collar it is up to one radius deep in, from any angle.
            unblocked = np.flatnonzero(~blocked)
            if unblocked.size == 0:
                return False, math.inf
            first = int(unblocked[0])
            if travelled[first] > 2.0 * self.grid.stamped_radius_m:
                return False, math.inf

        if bool(blocked[first:].any()):
            return False, math.inf

        rows, cols = rows[first:], cols[first:]
        travelled = travelled[first:]
        ts = ts[first:]
        samples = rows.size - 1
        free_mask = ~self.grid.unknown[rows, cols]
        indices = np.arange(samples + 1)
        # Index of the most recent known-free sample at or before each position,
        # or -1 if this segment has not touched known-free ground yet.
        last_free = np.maximum.accumulate(np.where(free_mask, indices, -1))
        seen_free = last_free >= 0
        run = np.where(
            free_mask,
            0.0,
            np.where(
                seen_free,
                travelled - travelled[np.maximum(last_free, 0)],
                entry_unknown_run + travelled,
            ),
        )
        if float(run.max()) > self.unknown_lookahead_m:
            return False, math.inf
        return True, float(run[-1])

# --------------------------------------------------------------- free functions


def _distance(a: Point, b: Point) -> float:
    return math.hypot(b[0] - a[0], b[1] - a[1])


def _path_length(path: Sequence[Point]) -> float:
    """sum(d(p)), the distance term of the exploration heuristic in slide 58."""
    return sum(_distance(path[i], path[i + 1]) for i in range(len(path) - 1))


def densify_path(points: Sequence[Point], spacing: float) -> list[Point]:
    """Insert intermediate waypoints so no gap exceeds `spacing`.

    The planner returns waypoints one step size apart, 0.30 m, and the supplied
    follower consumes a waypoint as soon as it is within its 0.20 m look-ahead
    and then steers at the next one. Across a corner that means it aims through
    the corner rather than around it, and the arc it cuts is bounded by the
    waypoint spacing, not by anything the planner checked. The planner verified
    the straight segments; the robot drove the chord.

    In a 0.7 m maze corridor that chord is enough to put the robot inside the
    inflation margin and wedge it against a wall with its wheels turning. That is
    worse than a lost second: slipping wheels move odometry without moving the
    robot, and slam_toolbox's correlation search space is only 0.5 m wide, so once
    the odom error exceeds it the scan match fails and the map smears.

    Densifying to 0.10 m, half the look-ahead, is the whole fix and costs nothing
    but a longer message: the follower always has a waypoint inside its look-ahead
    that lies on the path the planner actually checked.
    """
    if spacing <= 0.0 or len(points) < 2:
        return list(points)
    dense: list[Point] = [points[0]]
    for start, end in zip(points, points[1:]):
        distance = _distance(start, end)
        steps = max(1, int(math.ceil(distance / spacing)))
        for i in range(1, steps + 1):
            t = i / steps
            dense.append((start[0] + (end[0] - start[0]) * t,
                          start[1] + (end[1] - start[1]) * t))
    return dense


def _propagate(node: RRTStarNode, cost: float, unknown_run: float) -> None:
    """Write a corrected cost onto `node` and every descendant of it.

    Iterative rather than recursive: a tree of 1500 nodes can be a chain of 1500
    nodes, and Python's default recursion limit is 1000.
    """
    node.cost = cost
    node.unknown_run = unknown_run
    stack = list(node.children)
    while stack:
        child = stack.pop()
        parent = child.parent
        assert parent is not None  # children always have a parent
        child.cost = parent.cost + _distance(parent.point, child.point)
        stack.extend(child.children)


if __name__ == "__main__":
    # A room with one wall and a gap in it, at the SLAM map's real resolution.
    # If the planner cannot solve a fake room it will not solve a real one.
    from .grid import OccupancyMap

    WIDTH, HEIGHT = 80, 60  # 4.0 m by 3.0 m at 0.05 m per cell
    cells = np.zeros((HEIGHT, WIDTH), dtype=np.int16)
    cells[:, 0] = cells[:, -1] = cells[0, :] = cells[-1, :] = 100
    cells[25:28, :] = 100          # a wall across the room
    cells[25:28, 55:65] = 0        # with a gap in it

    occupancy = OccupancyMap.from_rows(cells, resolution=0.05)
    planner = RRTStarPlanner(
        PlanningGrid(occupancy, inflation_radius_m=0.105),
        rng=random.Random(0),
        max_plan_time_s=5.0,
    )
    result = planner.plan((0.5, 0.5), (0.5, 2.5))
    print(f"iterations {result.iterations}, {result.elapsed_s * 1e3:.1f} ms")
    if result.path is None:
        print("no path found")
    else:
        print(f"{len(result.path)} waypoints, {result.length:.2f} m")
