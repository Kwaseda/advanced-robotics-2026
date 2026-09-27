"""The path following control law, with no ROS in it.

Why this exists at all
----------------------
The course supplies `r7021e_exploration/path_follower_node.py` and the lab
instructions say "you are allowed to modify the nodes as you feel is needed".
Two measurements off recorded runs say the supplied law is worth replacing:

  - 45 to 62 percent of every command issued across a run is a pure rotation,
    zero linear velocity, robot turning on the spot and covering no ground.
  - The law flips between turning and driving about 14 times a minute, median
    turn burst 0.9 s, median drive burst under 2.5 s.

That is the supplied law working exactly as written. It zeroes linear velocity
whenever the heading error to its target exceeds 0.3 rad, so with waypoints
0.1 m apart on a path that bends, the robot stops, turns, creeps forward until
the next waypoint is off the nose by more than 0.3 rad, and stops again. The
competition is scored on time, so half the run spent not translating is half
the score. The starts and stops are also where the wheels slip, and slip is
what feeds the odometry error the scan matcher cannot recover from.

What this law changes, and what it keeps
----------------------------------------
Kept: pure pursuit against a look-ahead point, proportional heading control,
the same velocity limits, the same single `cmd_vel` topic and message type. The
launch file takes `follower:=course` to run the supplied node instead.

Changed, three things:

1. Arrival is detected. The supplied version pops waypoints only
   `while len(path) > 1`, so its list never empties, and at the end of a path it
   chases `atan2` of a vanishing vector and spins. This one stops when the last
   waypoint is inside `goal_tolerance` and stays stopped. That alone turns the
   navigation node's park manoeuvre from "rotate to face world east and settle"
   into "stop", with no change to the navigation node.

2. Speed tapers with heading error instead of switching. `cos(error)` scales the
   speed, so a robot 0.3 rad off its line keeps 95 percent of its speed and
   drives an arc, and a robot 1.0 rad off keeps 54 percent. Turning and driving
   at once is what a differential drive is for.

3. The turn-in-place threshold moves from 0.3 rad to a parameter defaulting to
   1.2 rad, which is where `cos` has fallen to 0.36 anyway. Below it the robot
   arcs, above it the target is far enough off the nose that translating would
   carry the body sideways into whatever the path was avoiding, so it turns
   first. The threshold stops being the normal case and becomes the exception.

Measured in Gazebo, world lab3_maze_wide: pure rotation commands fall from 45
to 8 percent of the run, and the body's real rotation stops exceeding what it
was commanded to do, 3.73 times down to 0.90.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import NamedTuple, Sequence

Point = tuple[float, float]


class Command(NamedTuple):
    """One velocity command, plus how far along the path it was produced."""

    v: float
    w: float
    index: int
    arrived: bool


@dataclass(frozen=True)
class FollowerLimits:
    """Everything the law is allowed to know about the robot."""

    max_v: float = 0.15
    max_w: float = 1.0
    kp_vel: float = 1.0
    kp_yaw: float = 2.0
    look_ahead: float = 0.2
    goal_tolerance: float = 0.05
    turn_in_place: float = 1.2
    # The reactive layer. `stop_distance` is where forward motion is refused
    # outright and `slow_distance` is where it starts tapering; between them the
    # speed cap falls linearly.
    stop_distance: float = 0.18
    slow_distance: float = 0.30
    # Half width of the strip checked ahead: 0.069 m of Burger plus 0.031 m.
    half_width: float = 0.10


def angle_difference(from_angle: float, to_angle: float) -> float:
    """Shortest signed rotation from one heading to another, in (-pi, pi]."""
    return math.remainder(to_angle - from_angle, 2.0 * math.pi)


def advance_index(path: Sequence[Point], position: Point, index: int,
                  look_ahead: float) -> int:
    """The first waypoint at or beyond the look-ahead distance, never going back.

    Carrying an index rather than popping the list keeps the last waypoint
    addressable after the ones before it are consumed, which is what makes
    arrival detectable. It also means a path that doubles back on itself cannot
    have its tail eaten by the robot passing near it early on.
    """
    last = len(path) - 1
    while index < last and _distance(path[index], position) <= look_ahead:
        index += 1
    return index


def follow(path: Sequence[Point], position: Point, yaw: float,
           limits: FollowerLimits, index: int = 0) -> Command:
    """One control cycle: where to aim, how fast, and whether we are done."""
    if not path:
        return Command(0.0, 0.0, 0, True)

    index = advance_index(path, position, min(index, len(path) - 1),
                          limits.look_ahead)
    target = path[index]
    distance = _distance(target, position)

    # Arrival is only meaningful at the end of the path. A waypoint in the
    # middle passing inside the tolerance means the robot is on course, not that
    # it has finished, and `advance_index` has already moved past it.
    if index == len(path) - 1 and distance <= limits.goal_tolerance:
        return Command(0.0, 0.0, index, True)

    error = angle_difference(yaw, math.atan2(target[1] - position[1],
                                             target[0] - position[0]))
    w = _clamp(limits.kp_yaw * error, limits.max_w)

    if abs(error) >= limits.turn_in_place:
        return Command(0.0, w, index, False)

    # Slow down for the last waypoint so the robot settles on it rather than
    # overshooting and turning round, and slow down for heading error so the
    # path that gets driven is the arc the robot can actually hold.
    speed = min(limits.max_v, limits.kp_vel * distance) \
        if index == len(path) - 1 else limits.max_v
    return Command(max(0.0, speed * math.cos(error)), w, index, False)


def _clamp(value: float, limit: float) -> float:
    return math.copysign(min(abs(value), limit), value)


def _distance(a: Point, b: Point) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def reactive_limit(command: Command, clearance: float,
                   limits: FollowerLimits) -> Command:
    """Cap forward speed on what the LiDAR sees straight ahead.

    Why this is here and not in the planner
    ---------------------------------------
    The navigation node already refuses to drive at something closer than
    `safety_stop_distance`, and it checks once per replan, which is 1 Hz. A robot
    at 0.15 m/s covers 0.15 m between two of those checks and the threshold is
    0.18 m, so the check gets one look at the gap and often gets it too late.
    Measured on a recorded run: forward clearance went 0.45, 0.40, 0.25, 0.10 m
    on consecutive seconds at a commanded 0.150 m/s the whole way, and the robot
    was against the wall before the planner ticked again.

    The follower runs at 10 Hz, which is 0.015 m of travel per cycle, and the
    scan arrives at 5 Hz. So this is the layer that can actually stop in time,
    and stopping in time is what Task 3 asks for: "As long as the robot is
    ensured not to drive into walls". Inflation alone cannot do it, because
    inflation is a statement about the map and the map is wrong exactly when it
    matters.

    Angular velocity is deliberately untouched. A robot that has stopped in front
    of a wall still needs to turn away from it, and zeroing the turn is how a stop
    becomes a wedge.

    The margin, written out: braking from 0.15 m/s at the drive plugin's
    1.0 m/s^2 takes 0.011 m, plus 0.015 m travelled before the next cycle sees
    anything. 0.026 m against a 0.12 m gap between `stop_distance` and the
    LiDAR's own 0.12 m `range_min` floor, below which there are no returns to
    read at all.
    """
    if command.v <= 0.0 or not math.isfinite(clearance):
        return command
    if clearance <= limits.stop_distance:
        return command._replace(v=0.0)
    if clearance >= limits.slow_distance:
        return command
    span = limits.slow_distance - limits.stop_distance
    if span <= 0.0:
        return command
    scale = (clearance - limits.stop_distance) / span
    return command._replace(v=command.v * scale)


def forward_clearance(ranges: Sequence[float], angle_min: float,
                      angle_increment: float, limits: FollowerLimits,
                      range_min: float = 0.0,
                      range_max: float = float('inf')) -> float:
    """Distance ahead to the nearest return inside the strip the body sweeps.

    A return at bearing b and range r counts when r cos b > 0 and
    |r sin b| <= `limits.half_width`, and the answer is r cos b. A cone of
    bearings misses a wall corner passing at 45 to 90 degrees, which is where a
    corner is when the robot pulls away from it. Wider than 0.10 m also catches a
    wall the robot is sliding past and stops it in narrow dead ends.

    Returns infinity when the strip holds nothing, which means nothing is in
    range rather than nothing is there. A reading sitting exactly on `range_min`
    is kept, not discarded: the simulator clamps a hit closer than the minimum to
    the minimum, so that value means "this close or closer" and is the single
    most important reading in the array.
    """
    best = float('inf')
    for i, value in enumerate(ranges):
        if not math.isfinite(value) or value < range_min or value > range_max:
            continue
        bearing = angle_min + i * angle_increment
        ahead = value * math.cos(bearing)
        if ahead > 0.0 and abs(value * math.sin(bearing)) <= limits.half_width:
            best = min(best, ahead)
    return best
