"""The one grid representation, shared by the planner and the exploration gain.

One occupancy grid representation, decided once, in a file that both the planner and
the exploration gain import. Two grid formats in one package is two sets of
coordinate conventions to get wrong.

Why a separate module and not just "pass the OccupancyGrid around"
------------------------------------------------------------------
Three things have to be true at once and none of them is free:

1. No ROS import, so the algorithms can run at a desk against a grid typed out by
   hand. The node unpacks the message into plain numbers and hands those down.
   `from_message()` is the only function in the package that knows the field names of
   a ROS message, and it takes them as arguments rather than importing the type.

2. `/map` and `/frontiers` are guaranteed to be the same shape. Read
   `frontier_detector_node.py`: it copies `msg.info` straight from the map onto its
   own output. So width, height, resolution and origin are identical between the two
   grids by construction, and a cell index means the same cell in both. That is worth
   relying on deliberately rather than by accident, so `GridInfo` is shared and
   `OccupancyMap.same_frame_as()` asserts it rather than hoping.

3. Inflation happens once per replan cycle, not once per candidate. The navigation
   loop plans one RRT* per surviving frontier cluster, so anything done per planner
   construction gets multiplied by the number of candidates. `PlanningGrid` is built
   once from the map and handed to every planner that cycle.

Coordinate conventions, stated once because getting them wrong is silent
------------------------------------------------------------------------
ROS occupancy grids are row-major with the origin at the *bottom-left* corner of cell
(0, 0), and `data[row * width + col]`. World position of a cell *centre* is therefore

    x = origin_x + (col + 0.5) * resolution
    y = origin_y + (row + 0.5) * resolution

The half-cell is not decoration. Without it every world point a planner returns sits
on a cell corner, and a path that grazes a wall by half a cell is a path that clips it
on the robot.

Cell values follow the ROS convention: -1 unknown, 0 to 100 the occupancy probability
in percent.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence

import numpy as np
import numpy.typing as npt

Point = tuple[float, float]
Cell = tuple[int, int]  # (col, row), x-like first, matching world (x, y) order

UNKNOWN = -1


@dataclass(frozen=True)
class GridInfo:
    """Geometry of an occupancy grid, the `nav_msgs/MapMetaData` fields we use.

    Frozen because two grids in the same cycle must agree on it and a shared mutable
    copy is how they would stop agreeing.
    """

    resolution: float
    origin_x: float
    origin_y: float
    width: int
    height: int

    def __post_init__(self) -> None:
        if self.resolution <= 0.0:
            raise ValueError(f"resolution must be positive, got {self.resolution}")
        if self.width <= 0 or self.height <= 0:
            raise ValueError(f"grid must be non-empty, got {self.width}x{self.height}")

    @property
    def min_x(self) -> float:
        return self.origin_x

    @property
    def min_y(self) -> float:
        return self.origin_y

    @property
    def max_x(self) -> float:
        return self.origin_x + self.width * self.resolution

    @property
    def max_y(self) -> float:
        return self.origin_y + self.height * self.resolution

    def world_to_cell(self, point: Point) -> Cell:
        """Cell containing `point`. May be out of bounds; callers check."""
        col = int(math.floor((point[0] - self.origin_x) / self.resolution))
        row = int(math.floor((point[1] - self.origin_y) / self.resolution))
        return col, row

    def cell_to_world(self, cell: Cell) -> Point:
        """Centre of `cell` in world coordinates. See the half-cell note above."""
        col, row = cell
        return (
            self.origin_x + (col + 0.5) * self.resolution,
            self.origin_y + (row + 0.5) * self.resolution,
        )

    def contains_cell(self, cell: Cell) -> bool:
        col, row = cell
        return 0 <= col < self.width and 0 <= row < self.height

    def contains_point(self, point: Point) -> bool:
        return self.contains_cell(self.world_to_cell(point))


class OccupancyMap:
    """A `nav_msgs/OccupancyGrid`'s payload, as a 2D array plus its geometry.

    `data` is indexed `[row, col]`, which is numpy's natural order for a row-major
    buffer. Everything that takes or returns a cell uses `(col, row)` instead, so that
    cell tuples read in the same order as world `(x, y)` tuples. Mixing those two up
    produces a map that looks transposed only when it is not square, which is a bug
    that hides for as long as you test on square grids.
    """

    __slots__ = ("info", "data")

    def __init__(self, data: npt.NDArray[np.int16], info: GridInfo) -> None:
        if data.shape != (info.height, info.width):
            raise ValueError(
                f"data shape {data.shape} does not match info {info.height}x{info.width}"
            )
        self.info = info
        self.data = data

    @classmethod
    def from_message(
        cls,
        data: Sequence[int],
        width: int,
        height: int,
        resolution: float,
        origin_x: float,
        origin_y: float,
    ) -> OccupancyMap:
        """Build from the flat fields of an OccupancyGrid message.

        Takes the fields rather than the message so this module stays ROS-free. The
        navigation node is the only caller and the only place that touches `msg.info`.
        """
        info = GridInfo(
            resolution=resolution,
            origin_x=origin_x,
            origin_y=origin_y,
            width=width,
            height=height,
        )
        array = np.asarray(data, dtype=np.int16).reshape((height, width))
        return cls(array, info)

    @classmethod
    def from_rows(cls, rows: Sequence[Sequence[int]], resolution: float = 0.05,
                  origin_x: float = 0.0, origin_y: float = 0.0) -> OccupancyMap:
        """Build from a list of rows, bottom row first. For tests and the __main__ demos.

        Bottom row first because that is what the ROS convention means: row 0 is at
        `origin_y`. Writing test grids the other way up and then wondering why the
        planner goes the wrong way is a wasted evening.
        """
        array = np.asarray(rows, dtype=np.int16)
        info = GridInfo(
            resolution=resolution,
            origin_x=origin_x,
            origin_y=origin_y,
            width=int(array.shape[1]),
            height=int(array.shape[0]),
        )
        return cls(array, info)

    def same_frame_as(self, other: OccupancyMap) -> bool:
        """True when `other` indexes the same cells as this map.

        The frontier detector guarantees this for `/map` and `/frontiers`. The
        navigation node checks it anyway, because the guarantee holds only while both
        messages come from the same map update, and they arrive on separate topics
        with no synchronisation.
        """
        return self.info == other.info

    def value_at_point(self, point: Point) -> int:
        """Cell value at a world point, or UNKNOWN for points off the map.

        Off-map reads as unknown rather than raising: the SLAM map grows, so a pose
        just outside the current bounds is a normal transient, not an error.
        """
        cell = self.info.world_to_cell(point)
        if not self.info.contains_cell(cell):
            return UNKNOWN
        col, row = cell
        return int(self.data[row, col])


class PlanningGrid:
    """An `OccupancyMap` with obstacles inflated, ready for collision checks.

    Task 3 of the lab instructions is collision avoidance, and this class is our whole
    answer to it. The instructions allow reactive avoidance, a potential field or a
    risk heuristic; we inflate the map by the robot's radius instead, so the RRT* can
    keep treating the robot as a point exactly as the instructions permit, and the
    margin is enforced by geometry that cannot be tuned away at run time. That is a
    design decision rather than a skipped task, and the report says so.

    Three classifications, and the difference between the second and third is the
    thing that makes frontier exploration work at all:

        blocked     occupied at or above `occupied_threshold`, or within
                    `inflation_radius_m` of a cell that is. Never traversable.
        known free  0 <= value < occupied_threshold, and not blocked.
        unknown     value < 0, and not blocked.

    Unknown is traversable. It has to be: a frontier goal sits by definition on the
    boundary of unknown space, so a planner that refuses to enter unknown cells cannot
    reach a single goal the exploration gain ever picks. What stops that becoming a
    fantasy route across half an unmapped maze is `unknown_lookahead_m` in
    `rrt_star.py`, which bounds how far a branch may run through unknown space before
    it has to touch known-free ground again.

    Unknown cells are deliberately NOT inflated. Inflating them would put a blocked
    collar around every frontier, which is the same failure in a different coat.
    """

    __slots__ = ("info", "blocked", "known_free", "unknown", "occupied",
                 "inflation_radius_m", "stamped_radius_m", "occupied_threshold",
                 "min_obstacle_neighbours")

    def __init__(
        self,
        occupancy: OccupancyMap,
        inflation_radius_m: float,
        occupied_threshold: int = 65,
        min_obstacle_neighbours: int = 1,
    ) -> None:
        if inflation_radius_m < 0.0:
            raise ValueError("inflation radius cannot be negative")
        self.info = occupancy.info
        self.inflation_radius_m = inflation_radius_m
        self.occupied_threshold = occupied_threshold
        self.min_obstacle_neighbours = min_obstacle_neighbours

        # The radius actually stamped, after the ceil in _inflate. Callers that
        # reason about how thick a collar is must use this and not the requested
        # radius: at 0.05 m cells a requested 0.105 m becomes 0.15 m, and a
        # margin computed from the wrong one of those two is off by 50 percent.
        self.stamped_radius_m = (
            math.ceil(inflation_radius_m / occupancy.info.resolution)
            * occupancy.info.resolution
        )

        raw_occupied = occupancy.data >= occupied_threshold
        # Kept as well as `blocked`: the inflated map says where the robot may
        # plan, and this says where there is actually something. `nearest_free`
        # needs the difference, because the whole point of it is to cross a
        # collar without crossing a wall.
        self.occupied = _despeckle(raw_occupied, min_obstacle_neighbours)
        self.blocked = _inflate(self.occupied, inflation_radius_m,
                                occupancy.info.resolution)
        raw_unknown = occupancy.data < 0
        # Order matters: blocked wins over both. A cell inside a wall's inflation
        # collar is not "free with a caveat", it is somewhere the robot may not go.
        self.unknown = raw_unknown & ~self.blocked
        self.known_free = ~raw_unknown & ~self.blocked

    def classify_point(self, point: Point) -> int:
        """-1 blocked, 0 unknown, 1 known free. Off-map counts as blocked.

        Off-map is blocked here and unknown in `OccupancyMap.value_at_point`, and the
        difference is intentional. Reading a value off the edge of a growing map is
        benign. Planning a path off the edge of one is not: there is no evidence the
        space exists, and the cap on unknown travel cannot bound something unbounded.
        """
        cell = self.info.world_to_cell(point)
        if not self.info.contains_cell(cell):
            return -1
        col, row = cell
        if self.blocked[row, col]:
            return -1
        return 0 if self.unknown[row, col] else 1

    def is_blocked(self, point: Point) -> bool:
        return self.classify_point(point) < 0

    def is_known_free(self, point: Point) -> bool:
        return self.classify_point(point) > 0

    def nearest_unblocked(self, point: Point, radius_m: float) -> Point | None:
        """The closest traversable point to `point`, or None within `radius_m`.

        Used when the robot's own pose is not traversable, which is not an error
        and not rare. Two things put it there. A 0.7 m maze corridor is narrower
        than twice the 0.15 m collar plus the robot, so driving down the middle
        of one already means standing inside the collar. And a live SLAM map
        carries scattered spurious occupied cells; each inflates to a 7 by 7
        blob, and a region with a diffuse scatter of them two cells apart has no
        traversable cell left in it at all, even though its raw map is almost
        entirely free.

        Refusing to plan in either case stops the run permanently, which is the
        worse failure: it ends a full maze run with most of the maze unexplored
        and perfectly reachable frontier goals reported unreachable.

        The returned point is checked against the RAW occupancy along the way,
        not the inflated map. Crossing a collar is the entire purpose; crossing
        a wall to reach free space on its far side would put the robot through
        it, so any candidate whose straight line passes a genuinely occupied
        cell is rejected. Candidates are searched nearest first, so the answer
        is the closest one that survives that test.
        """
        reachable = self.reachable_free_points(point, radius_m)
        return reachable[0][1] if reachable else None

    def farthest_unblocked(self, point: Point, radius_m: float) -> Point | None:
        """The furthest traversable point within `radius_m`, or None.

        The counterpart of `nearest_unblocked`, for a different job. Rooting a
        plan wants the nearest such point, because the robot has to drive there
        before the real path starts. Recovering from being sealed in wants the
        furthest, because the entire purpose of that move is to see the
        surroundings from somewhere else and let new scans contradict the
        phantom walls.

        Taking the nearest for that job degenerates in a way worth recording: the
        robot moves to the nearest free point, the next cycle finds it still
        sealed in, and the nearest free point is now the one it is standing next
        to, so the recovery becomes 0.15 m, then 0.08, then 0.03, then 0.01, and
        four attempts are spent without the robot going anywhere.
        """
        reachable = self.reachable_free_points(point, radius_m)
        return reachable[-1][1] if reachable else None

    def reachable_free_points(
        self, point: Point, radius_m: float
    ) -> list[tuple[float, Point]]:
        """Traversable points within `radius_m`, nearest first, as (distance, point).

        A candidate is rejected when the straight line from `point` to it passes
        a genuinely occupied cell, so this is what the robot could actually drive
        to in one move rather than merely what is nearby.
        """
        origin = self.info.world_to_cell(point)
        max_cells = int(math.ceil(radius_m / self.info.resolution))
        found: list[tuple[float, Point]] = []

        for dy in range(-max_cells, max_cells + 1):
            for dx in range(-max_cells, max_cells + 1):
                cell = (origin[0] + dx, origin[1] + dy)
                if not self.info.contains_cell(cell):
                    continue
                if self.blocked[cell[1], cell[0]]:
                    continue
                candidate = self.info.cell_to_world(cell)
                distance = math.hypot(candidate[0] - point[0], candidate[1] - point[1])
                if distance > radius_m:
                    continue
                if self._crosses_occupied(point, candidate):
                    continue
                found.append((distance, candidate))
        found.sort(key=lambda pair: pair[0])
        return found

    def _crosses_occupied(self, a: Point, b: Point) -> bool:
        """True when the straight segment a..b passes a genuinely occupied cell.

        The sample at `a` itself is skipped. `a` is the robot's own pose, and on
        a noisy map the robot's own cell is sometimes marked occupied: a spurious
        return landed on the cell the robot is standing in. Counting that as an
        obstacle rejects every candidate and the search returns nothing, which is
        exactly what it did before this line existed. The robot has demonstrably
        survived being where it is, so the question is only what lies between
        there and the candidate.
        """
        distance = math.hypot(b[0] - a[0], b[1] - a[1])
        steps = max(1, int(math.ceil(distance / (0.5 * self.info.resolution))))
        for i in range(1, steps + 1):
            t = i / steps
            probe = (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)
            cell = self.info.world_to_cell(probe)
            if not self.info.contains_cell(cell):
                return True
            if self.occupied[cell[1], cell[0]]:
                return True
        return False


def _despeckle(
    occupied: npt.NDArray[np.bool_], min_neighbours: int
) -> npt.NDArray[np.bool_]:
    """Drop occupied cells with fewer than `min_neighbours` occupied neighbours.

    This runs before the inflation and it is not cosmetic. A live SLAM map carries
    isolated occupied cells in the middle of free space: single spurious returns,
    a moving reflection, a cell that crossed the occupancy threshold on one beam.
    One of them is harmless on its own. Inflated by a 3-cell disc, each becomes a
    7 by 7 blocked blob, and a corridor with a handful of them scattered along it
    is a corridor the planner cannot enter at all.

    That is not hypothetical and it is not a small effect. On a 7.2 m maze run the
    robot finished inside a region whose raw map was almost entirely free, with a
    scatter of isolated occupied cells through it, and whose inflated map was
    solid: a flood fill from the robot's own cell reached one cell, its own. Every
    plan failed, all six frontier candidates were reported unreachable, and the run
    stopped with most of the maze unexplored, while the planner was individually
    correct about every answer it gave.

    A threshold of 1 removes only cells with no occupied neighbour at all in the
    8-neighbourhood, which is as conservative as this can be while doing anything.
    Walls are continuous, so a genuine wall cell has at least one neighbour; the
    only real obstacle this can erase is a free-standing one-cell object, and at
    0.05 m resolution that is smaller than the map can describe reliably anyway.
    """
    if min_neighbours <= 0:
        return occupied

    counts = np.zeros(occupied.shape, dtype=np.int8)
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            if dy == 0 and dx == 0:
                continue
            # Same clipped-slice shift as _inflate uses, and for the same reason:
            # np.roll would wrap, giving cells on one edge neighbours on the other.
            height, width = occupied.shape
            src_y0, src_y1 = max(0, -dy), min(height, height - dy)
            src_x0, src_x1 = max(0, -dx), min(width, width - dx)
            if src_y0 >= src_y1 or src_x0 >= src_x1:
                continue
            counts[src_y0 + dy:src_y1 + dy, src_x0 + dx:src_x1 + dx] += (
                occupied[src_y0:src_y1, src_x0:src_x1]
            )
    return occupied & (counts >= min_neighbours)


def _inflate(
    occupied: npt.NDArray[np.bool_], radius_m: float, resolution: float
) -> npt.NDArray[np.bool_]:
    """Stamp a disc of `radius_m` over every occupied cell.

    numpy shifts rather than `scipy.ndimage.binary_dilation`: scipy is not a declared
    dependency of anything in this workspace, and discovering a missing one on a fresh
    USB image at a lab bench costs more than fifteen lines of shifts.

    The radius is converted with `ceil`, not `round`. At the SLAM map's 0.05 m
    resolution the 0.105 m footprint radius is 2.1 cells, and rounding down to 2 cells
    gives 0.10 m of real margin, 5 mm less than the number written in the YAML. The
    config file would then be quietly lying about what the robot enforces. Rounding up
    to 3 cells gives 0.15 m, which is more margin than asked for but in the direction
    that does not scrape a wall, and it is visible in the map rather than hidden in an
    integer conversion.
    """
    if radius_m <= 0.0:
        return occupied.copy()

    radius_cells = int(math.ceil(radius_m / resolution))
    offsets = [
        (dy, dx)
        for dy in range(-radius_cells, radius_cells + 1)
        for dx in range(-radius_cells, radius_cells + 1)
        if dy * dy + dx * dx <= radius_cells * radius_cells
    ]

    height, width = occupied.shape
    result = np.zeros_like(occupied)
    for dy, dx in offsets:
        # Source window and destination window, clipped at both edges. Writing this as
        # explicit slice bounds rather than np.roll matters: roll wraps, so an obstacle
        # on the right edge of the map would inflate onto the left edge.
        src_y0, src_y1 = max(0, -dy), min(height, height - dy)
        src_x0, src_x1 = max(0, -dx), min(width, width - dx)
        if src_y0 >= src_y1 or src_x0 >= src_x1:
            continue
        dst_y0, dst_y1 = src_y0 + dy, src_y1 + dy
        dst_x0, dst_x1 = src_x0 + dx, src_x1 + dx
        result[dst_y0:dst_y1, dst_x0:dst_x1] |= occupied[src_y0:src_y1, src_x0:src_x1]
    return result
