#!/usr/bin/env python3
"""The Lab 3 navigation loop: the only file in this package that imports rclpy.

Tasks 1 and 5 of the lab instructions both live here. Task 1 is "send a Path to
the follower and send a new one when the robot reaches the end"; task 5 is that
same loop with the goal chosen by the exploration gain instead of written down.
They are the same node, and building task 1 as a throwaway script would have
meant building this twice.

Everything algorithmic is somewhere else. This file owns subscriptions, timers,
message types, headers, tf and parameters, and nothing that could be tested at a
desk. That is the split Labs 1 and 2 used and the reason the planner and the gain
have unit tests at all.

The loop
--------
A timer at `replan_period` seconds, not the map callback. The course template
replans inside `_on_map`, which fires at map rate; with one full RRT* per
candidate cluster that is far too often, and it ties the planning cadence to a
SLAM parameter nobody set with planning in mind. One timer, three triggers:

    arrival    within `arrival_tolerance` of the last waypoint
    stall      less than `stall_distance` of progress in `stall_timeout` seconds
    staleness  the current path is older than `max_path_age` seconds

Stall matters more than it looks. The provided path follower has no recovery
behaviour of its own, so a robot wedged against a wall stays wedged until a
human intervenes, and a hardware session is three hours long.

Detecting arrival ourselves, and stopping at all
-------------------------------------------------
`path_follower_node.py` publishes on exactly one topic, `cmd_vel`. There is no
status, done or goal-reached topic anywhere in it, so completion is ours to
detect by comparing the tf pose against the last waypoint. Read from source
2026-09-23; evidence tier `untested`.

Stopping is the same problem one step further on. The follower pops waypoints
only `while len(self.path) > 1`, so its list never empties and it keeps
publishing forever; and it returns early on an empty path *without* publishing
zero, so there is no message that stops it. What does work is a path holding a
single waypoint at the robot's own position: the distance term goes to zero and
the robot stops translating. It then rotates to face world east, because
`atan2(0.0, 0.0)` is 0.0, and parks there. That is the park manoeuvre this node
performs on termination, and it is a real-time problem the report names rather
than a bug this node hides.

The alternative was publishing zero `TwistStamped` ourselves, which would put a
second publisher on `cmd_vel` against a follower that never stops publishing.
Two publishers on one `cmd_vel` do not error, they interleave, and the robot
does something that looks like a tuning problem and is not.

The frontier topic defect
-------------------------
`frontier_detector_node.py` publishes on `frontiers`. The course template's
navigation node declares `frontier_topic` defaulting to `frontier`, singular,
and `exploration.launch.py` carries no remapping, so as shipped that
subscription never receives anything. This node defaults to the plural, which is
the name the publisher actually uses. Confirmed by reading the package
2026-09-23; evidence tier `untested`.
"""

from __future__ import annotations

import math
from typing import Sequence

import rclpy
from builtin_interfaces.msg import Duration as DurationMsg
from geometry_msgs.msg import PoseStamped, Point as PointMsg, Quaternion, Vector3
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import LaserScan
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy, qos_profile_sensor_data)
from rclpy.time import Time
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros

from .exploration_gain import (
    ExplorationDecision,
    GainConfig,
    select_best_frontier,
)
from .grid import OccupancyMap, PlanningGrid
from .rrt_star import (RRTStarPlanner, centre_path, densify_path,
                       look_around_target)

Point = tuple[float, float]

# Two different profiles, and they are not interchangeable.
#
# slam_toolbox publishes /map TRANSIENT_LOCAL, so a late subscriber still gets
# the current map instead of waiting up to a full map_update_interval for the
# next one. The course's frontier_detector_node creates its publisher with a bare
# `QoSProfile(depth=1)`, which is VOLATILE, so /frontiers is volatile whatever we
# would prefer.
#
# Using the map's profile for both is what this node did first, and DDS does not
# fall back: an incompatible subscription silently receives nothing. It is not
# quite silent in this case, which is the only reason it took one run rather than
# an evening. Both sides log it, and the wording is worth recognising:
#
#   [frontier_detector] New subscription discovered on topic 'frontiers',
#       requesting incompatible QoS. No messages will be sent to it.
#       Last incompatible policy: DURABILITY
#   [navigation_node] New publisher discovered on topic 'frontiers', offering
#       incompatible QoS. No messages will be received from it.
#
# Evidence tier: sim, lab3_maze_small, 2026-09-23.
MAP_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

FRONTIER_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    durability=DurabilityPolicy.VOLATILE,
)


def yaw_to_quaternion(yaw: float) -> Quaternion:
    """Planar yaw to a Quaternion."""
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def quat_to_yaw(q: Quaternion) -> float:
    """Planar yaw from a Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class NavigationNode(Node):
    """Frontier exploration: choose a goal, plan to it, drive, repeat, stop."""

    def __init__(self) -> None:
        super().__init__('navigation_node')

        # ---- parameters. Every one of these is in config/lab3.yaml with the
        # reason for its value next to it, so `ros2 param get` on a running node
        # reads back the number the report quotes.
        self.declare_parameter('map_topic', 'map')
        self.declare_parameter('frontier_topic', 'frontiers')
        self.declare_parameter('path_topic', 'path')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('tf_timeout_sec', 0.5)

        self.declare_parameter('replan_period', 1.0)
        self.declare_parameter('arrival_tolerance', 0.20)
        self.declare_parameter('stall_distance', 0.05)
        self.declare_parameter('stall_timeout', 8.0)
        self.declare_parameter('max_path_age', 20.0)
        self.declare_parameter('max_empty_cycles', 10)
        self.declare_parameter('max_exhaust_resets', 3)
        self.declare_parameter('path_centring_shift', 0.12)
        self.declare_parameter('safety_stop_distance', 0.18)
        self.declare_parameter('safety_sector', 0.6)
        self.declare_parameter('safety_retreat', 0.25)
        self.declare_parameter('max_look_around_cycles', 8)
        self.declare_parameter('look_around_radius', 0.05)
        self.declare_parameter('look_around_step', 2.0)
        self.declare_parameter('max_unstick_attempts', 4)
        self.declare_parameter('unstick_radius', 0.60)
        self.declare_parameter('unstick_min_distance', 0.15)

        self.declare_parameter('inflation_radius', 0.105)
        self.declare_parameter('occupied_threshold', 65)
        self.declare_parameter('min_obstacle_neighbours', 1)

        self.declare_parameter('rrt.max_iterations', 1500)
        self.declare_parameter('rrt.step_size', 0.30)
        self.declare_parameter('rrt.goal_sample_rate', 0.10)
        self.declare_parameter('rrt.goal_tolerance', 0.15)
        self.declare_parameter('rrt.rewire_radius', 0.60)
        self.declare_parameter('rrt.unknown_lookahead', 0.50)
        self.declare_parameter('rrt.extra_iterations_after_solution', 200)
        self.declare_parameter('rrt.max_plan_time', 0.15)
        self.declare_parameter('rrt.retry_plan_time_scale', 1.0)

        self.declare_parameter('gain.mode', 'reduced_range')
        self.declare_parameter('gain.radius', 0.75)
        self.declare_parameter('gain.weight_per_cell', 0.10)
        self.declare_parameter('gain.turn_cost_per_rad', 0.0)

        self.declare_parameter('cluster.min_size', 5)
        self.declare_parameter('cluster.max_size', 100)
        self.declare_parameter('cluster.max_candidates', 6)

        self.declare_parameter('commit.switch_margin', 0.5)
        self.declare_parameter('commit.match_radius', 0.30)
        self.declare_parameter('goal.exhaust_radius', 0.30)
        self.declare_parameter('path_waypoint_spacing', 0.10)

        self.declare_parameter('publish_markers', True)

        self.global_frame = str(self.get_parameter('global_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.tf_timeout = Duration(
            seconds=float(self.get_parameter('tf_timeout_sec').value))

        self.arrival_tolerance = float(self.get_parameter('arrival_tolerance').value)
        self.stall_distance = float(self.get_parameter('stall_distance').value)
        self.stall_timeout = float(self.get_parameter('stall_timeout').value)
        self.max_path_age = float(self.get_parameter('max_path_age').value)
        self.max_empty_cycles = int(self.get_parameter('max_empty_cycles').value)
        self.max_exhaust_resets = int(
            self.get_parameter('max_exhaust_resets').value)
        self.path_centring_shift = float(
            self.get_parameter('path_centring_shift').value)
        self.safety_stop_distance = float(
            self.get_parameter('safety_stop_distance').value)
        self.safety_sector = float(self.get_parameter('safety_sector').value)
        self.safety_retreat = float(
            self.get_parameter('safety_retreat').value)
        self.max_look_around_cycles = int(
            self.get_parameter('max_look_around_cycles').value)
        self.look_around_radius = float(
            self.get_parameter('look_around_radius').value)
        self.look_around_step = float(
            self.get_parameter('look_around_step').value)
        self.max_unstick_attempts = int(
            self.get_parameter('max_unstick_attempts').value)
        self.unstick_radius = float(self.get_parameter('unstick_radius').value)
        self.unstick_min_distance = float(
            self.get_parameter('unstick_min_distance').value)
        self.inflation_radius = float(self.get_parameter('inflation_radius').value)
        self.occupied_threshold = int(self.get_parameter('occupied_threshold').value)
        self.min_obstacle_neighbours = int(
            self.get_parameter('min_obstacle_neighbours').value)
        self.publish_markers = bool(self.get_parameter('publish_markers').value)
        self.waypoint_spacing = float(
            self.get_parameter('path_waypoint_spacing').value)

        self.gain_config = GainConfig(
            mode=str(self.get_parameter('gain.mode').value),
            radius_m=float(self.get_parameter('gain.radius').value),
            weight_m_per_cell=float(self.get_parameter('gain.weight_per_cell').value),
            turn_cost_m_per_rad=float(
                self.get_parameter('gain.turn_cost_per_rad').value),
        )

        # ---- interfaces
        self.path_pub = self.create_publisher(
            Path, str(self.get_parameter('path_topic').value), 1)
        self.tree_pub = self.create_publisher(Marker, 'rrt_tree', 1)
        self.goals_pub = self.create_publisher(MarkerArray, 'frontier_goals', 1)
        self.status_pub = self.create_publisher(Marker, 'exploration_status', 1)

        self.create_subscription(
            OccupancyGrid, str(self.get_parameter('map_topic').value),
            self._on_map, MAP_QOS)
        self.create_subscription(
            OccupancyGrid, str(self.get_parameter('frontier_topic').value),
            self._on_frontier, FRONTIER_QOS)
        # The live scan, used for nothing but refusing to drive into something
        # the map has not got round to believing in yet. Sensor data QoS,
        # because a scan is worth nothing late.
        self.create_subscription(
            LaserScan, 'scan', self._on_scan, qos_profile_sensor_data)

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---- state
        self._map: OccupancyGrid | None = None
        self._frontier: OccupancyGrid | None = None
        self._path: list[Point] = []
        self._path_age = 0.0
        self._committed_goal: Point | None = None
        # Goals the robot has already stood on, or already failed to reach.
        # See _retire_goal for why this list has to exist at all.
        self._exhausted: list[Point] = []
        self._empty_cycles = 0
        # How many times the exhausted list has been cleared. Bounded, because an
        # unbounded version cannot terminate. See _try_forget_exhausted.
        self._exhaust_resets = 0
        self._unstick_attempts = 0
        self._look_around_cycles = 0
        self._scan: LaserScan | None = None
        self._grid = None
        self._safety_stops = 0
        self._look_around_cycles = 0
        self._cycle = 0
        self._finished = False
        self._finished_at: Point | None = None
        self._started_at = self.get_clock().now()
        # Whether this node has ever had something to explore. Termination is
        # gated on it; see _on_nothing_to_explore.
        self._started_exploring = False
        self._progress_pose: Point | None = None
        self._progress_age = 0.0

        self._period = float(self.get_parameter('replan_period').value)
        self.create_timer(self._period, self._tick)

        self.get_logger().info(
            f'navigation_node up. inflation {self.inflation_radius:.3f} m, '
            f'I(p) mode {self.gain_config.mode} at {self.gain_config.radius_m:.2f} m, '
            f'replanning every {self._period:.1f} s.')

    # ------------------------------------------------------------- callbacks

    def _on_scan(self, msg: LaserScan) -> None:
        self._scan = msg

    def _closest_return(self, half_angle: float) -> tuple[float, float]:
        """(range, bearing) of the nearest return within `half_angle` of ahead.

        Bearing is in the robot's own frame, so zero is straight ahead. Returns
        infinite range when there is no scan or nothing in the sector, so a
        missing sensor never manufactures a stop.
        """
        scan = self._scan
        if scan is None or not scan.ranges:
            return float('inf'), 0.0
        best, bearing = float('inf'), 0.0
        for index, value in enumerate(scan.ranges):
            if not math.isfinite(value) or value < scan.range_min:
                continue
            angle = scan.angle_min + index * scan.angle_increment
            angle = math.atan2(math.sin(angle), math.cos(angle))
            if abs(angle) <= half_angle and value < best:
                best, bearing = value, angle
        return best, bearing

    def _forward_clearance(self, half_angle: float) -> float:
        """Closest return within `half_angle` of straight ahead, in metres.

        The scan is in the robot's own frame, so this needs no pose, no map and
        no tf, and it is right even when all three are wrong. That is the whole
        point of it: every other check in this node reads the map, and the map
        is by construction out of date exactly where the robot is going, at the
        frontier.

        Returns infinity when there is no scan yet or nothing in the sector, so
        a missing sensor never manufactures a stop.
        """
        scan = self._scan
        if scan is None or not scan.ranges:
            return float('inf')
        best = float('inf')
        for index, value in enumerate(scan.ranges):
            if not math.isfinite(value) or value < scan.range_min:
                continue
            angle = scan.angle_min + index * scan.angle_increment
            angle = math.atan2(math.sin(angle), math.cos(angle))
            if abs(angle) <= half_angle and value < best:
                best = value
        return best

    def _on_map(self, msg: OccupancyGrid) -> None:
        """Store the map. Deliberately does no planning; see the module docstring."""
        self._map = msg

    def _on_frontier(self, msg: OccupancyGrid) -> None:
        self._frontier = msg

    # ------------------------------------------------------------- tf helper

    def get_robot_pose(self) -> tuple[float, float, float] | None:
        """(x, y, yaw) in the global frame, or None when tf is not ready.

        Every failure mode returns None rather than raising. The provided path
        follower does the opposite: it calls lookup_transform with no timeout
        and no try, inside a timer callback, so a lookup that fails propagates
        out of rclpy.spin() and kills that node. Nothing in this node publishes
        a path until this method has succeeded at least once, which keeps a
        path from arriving at the follower before tf is up.
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                self.global_frame, self.base_frame, Time(), timeout=self.tf_timeout)
        except tf2_ros.TransformException as exc:
            self.get_logger().warn(
                f'tf {self.global_frame} <- {self.base_frame} failed: {exc}',
                throttle_duration_sec=5.0)
            return None
        t = transform.transform.translation
        return t.x, t.y, quat_to_yaw(transform.transform.rotation)

    # ------------------------------------------------------------- the loop

    def _tick(self) -> None:
        if self._finished:
            # Keep saying so. The node used to return here, which meant that
            # after the last cycle nothing was published at all and the final
            # state of the run was a stale marker reading "idle 10/10", left
            # over from the cycle before the one that decided. A run that has
            # finished should be visibly finished, to RViz, to a bag opened
            # afterwards, and to anything that subscribes late, so the terminal
            # status is republished at the tick rate instead of latched once.
            self._publish_finished()
            return

        pose = self.get_robot_pose()
        if pose is None or self._map is None or self._frontier is None:
            return
        position = (pose[0], pose[1])

        # Before anything that consults the map: is the robot about to drive
        # into something? Measured cause of every failed run in this world is a
        # collision with a wall the map either had not seen or had put
        # elsewhere, after which the wheels lose traction, dead reckoning loses
        # thousands of degrees and the map becomes fiction. The map cannot
        # prevent that because the map is what is wrong. The scan can.
        near, _ = self._closest_return(self.safety_sector)
        if self._path and near < self.safety_stop_distance:
            self._safety_stops += 1
            # Stop, retire the goal, and let the next cycle plan afresh. An
            # earlier version backed away from the obstacle instead, which
            # sounds strictly better and measured far worse: the retreat point
            # sits behind the robot, the follower turns on the spot to face it
            # because the heading error exceeds its own 0.3 rad threshold, and a
            # 180 degree turn taken while already close to a wall is exactly the
            # manoeuvre that scrapes. Tipped samples went from 32 to 10104.
            # Stopping puts the robot nowhere new, which turns out to be the
            # point.
            self.get_logger().warn(
                f'cycle {self._cycle}: obstacle {near:.2f} m ahead, under the '
                f'{self.safety_stop_distance:.2f} m limit. Stopping and '
                f'retiring this goal (stop {self._safety_stops}).')
            if self._committed_goal is not None:
                self._exhausted.append(self._committed_goal)
            self._committed_goal = None
            self._path = []
            self._park(position)
            self._reset_progress(position)
            return

        self._track_progress(position)
        reason = self._replan_reason(position)
        if reason is None:
            return

        self._retire_goal(reason)

        try:
            decision = self._decide(position, pose[2])
        except ValueError as exc:
            # Raised when the frontier grid and the map disagree about their own
            # geometry, which happens for one cycle after SLAM resizes the map.
            # Skipping the cycle is correct: the next pair will agree.
            self.get_logger().warn(f'skipping cycle: {exc}')
            return

        self._cycle += 1
        if decision.chosen is None:
            self._on_nothing_to_explore(decision, position)
            return

        self._empty_cycles = 0
        # Both recovery budgets reset here, on the cycle that chose a goal. They
        # bound consecutive fruitless recovery, not recovery for the lifetime of
        # the run, and a goal being chosen is the evidence that the last round of
        # it worked. Counting them per run instead is what ended one run at 141 s
        # with 190 s of budget left and a third of the maze unexplored: the eight
        # look-around cycles had been spent much earlier, on a situation the robot
        # then drove out of, and the recovery was gone for good.
        #
        # `_exhaust_resets` deliberately does not reset here. Forgetting retired
        # goals is what puts targets back on the table, so resetting its budget on
        # reaching one of those targets is a loop with extra steps.
        self._unstick_attempts = 0
        self._look_around_cycles = 0
        self._started_exploring = True
        self._committed_goal = decision.chosen.goal
        self._path = list(decision.chosen.path or [])
        self._path_age = 0.0
        self._publish_path(self._path)
        self._reset_progress(position)

        chosen = decision.chosen
        self.get_logger().info(
            f'cycle {self._cycle} [{reason}]: {decision.clusters_found} clusters, '
            f'{len(decision.candidates)} scored, goal '
            f'({chosen.goal[0]:.2f}, {chosen.goal[1]:.2f}), '
            f'I={chosen.info_gain:.0f} d={chosen.path_length:.2f} m '
            f'H={chosen.score:.2f}'
            + (' [held]' if decision.kept_committed else ''))
        self._publish_markers(decision, position)

    def _retire_goal(self, reason: str) -> None:
        """Release the commitment, and blacklist the goal when it is spent.

        The commitment exists to stop the robot thrashing between two similar
        frontiers *while it is driving to one of them*. It has no business
        surviving arrival, and leaving it in place is half of a loop this node
        spent a whole Gazebo run in: arrive at a goal, find the frontier still
        there, hold the commitment, replan 0.3 m to the same goal, arrive again.
        Thirty-one cycles of it, with the robot moving a few centimetres back
        and forth. Evidence tier: sim, lab3_maze_small, 2026-09-23.

        Releasing the commitment alone does not fix it, because the same goal
        then wins on merit anyway: it is the nearest frontier and nothing in
        H = sum(d) - w I knows the robot has already been there. So a goal that
        has been arrived at, or that the robot stalled trying to reach, is
        retired: no candidate within `goal.exhaust_radius` of it is offered
        again for the rest of the run.

        Retiring on arrival is safe rather than aggressive. A frontier that the
        robot actually cleared disappears from the frontier grid on the next map
        update and would never be proposed again in any case; one that survives
        the robot standing next to it is one that standing there again will not
        clear, usually because it is a staircase of cells around a corner the
        LiDAR cannot see into from that side. Either way there is nothing left
        to gain by going back.

        'stale' does not retire anything. It means the path aged out, not that
        the goal is spent, and the robot may well still be making good progress
        toward it.
        """
        if reason not in ('arrived', 'stalled'):
            return
        if self._committed_goal is not None:
            self._exhausted.append(self._committed_goal)
            self.get_logger().info(
                f'retiring goal ({self._committed_goal[0]:.2f}, '
                f'{self._committed_goal[1]:.2f}) after {reason}; '
                f'{len(self._exhausted)} retired')
            self._committed_goal = None

    def _decide(self, position: Point, yaw: float) -> ExplorationDecision:
        """One planning cycle: build the grid once, then plan per candidate.

        The `PlanningGrid` and the `RRTStarPlanner` are constructed here, once,
        and the planner is reused for every candidate in this cycle. Inflation
        is the expensive part of grid construction and it does not depend on
        which goal is being planned to, so doing it per candidate would multiply
        the cycle's cost by the candidate count for no benefit.
        """
        assert self._map is not None and self._frontier is not None
        occupancy = _as_occupancy_map(self._map)
        frontier = _as_occupancy_map(self._frontier)
        grid = PlanningGrid(occupancy, self.inflation_radius, self.occupied_threshold,
                            self.min_obstacle_neighbours)
        self._grid = grid

        planner = RRTStarPlanner(
            grid,
            max_iterations=int(self.get_parameter('rrt.max_iterations').value),
            step_size_m=float(self.get_parameter('rrt.step_size').value),
            goal_sample_rate=float(self.get_parameter('rrt.goal_sample_rate').value),
            goal_tolerance_m=float(self.get_parameter('rrt.goal_tolerance').value),
            rewire_radius_m=float(self.get_parameter('rrt.rewire_radius').value),
            unknown_lookahead_m=float(
                self.get_parameter('rrt.unknown_lookahead').value),
            extra_iterations_after_solution=int(
                self.get_parameter('rrt.extra_iterations_after_solution').value),
            max_plan_time_s=float(self.get_parameter('rrt.max_plan_time').value),
        )

        return select_best_frontier(
            frontier, grid, position, planner,
            config=self.gain_config,
            min_cluster_size=int(self.get_parameter('cluster.min_size').value),
            max_cluster_size=int(self.get_parameter('cluster.max_size').value),
            max_candidates=int(self.get_parameter('cluster.max_candidates').value),
            committed_goal=self._committed_goal,
            switch_margin=float(self.get_parameter('commit.switch_margin').value),
            commit_match_radius_m=float(
                self.get_parameter('commit.match_radius').value),
            exhausted_goals=self._exhausted,
            exhaust_radius_m=float(self.get_parameter('goal.exhaust_radius').value),
            robot_yaw=yaw,
            retry_plan_time_scale=float(
                self.get_parameter('rrt.retry_plan_time_scale').value),
        )

    def _replan_reason(self, position: Point) -> str | None:
        """Why this tick should replan, or None to leave the current path alone."""
        if not self._path:
            return 'no path'
        if _distance(position, self._path[-1]) <= self.arrival_tolerance:
            return 'arrived'
        if self._progress_age >= self.stall_timeout:
            return 'stalled'
        if self._path_age >= self.max_path_age:
            return 'stale'
        return None

    def _track_progress(self, position: Point) -> None:
        """Age the current path and the stall timer.

        Ages advance by the timer period rather than by a clock difference, so
        that everything here obeys `use_sim_time` without a second code path.
        """
        self._path_age += self._period
        if self._progress_pose is None:
            self._reset_progress(position)
            return
        if _distance(position, self._progress_pose) >= self.stall_distance:
            self._reset_progress(position)
        else:
            self._progress_age += self._period

    def _reset_progress(self, position: Point) -> None:
        self._progress_pose = position
        self._progress_age = 0.0

    def _on_nothing_to_explore(
        self, decision: ExplorationDecision, position: Point
    ) -> None:
        """No candidate survived. Count it, and stop once it keeps happening.

        Two guards, and the first run in Gazebo needed both.

        The node starts before SLAM has published anything, so its first few
        ticks legitimately see no map and no frontiers. With the original
        five-cycle bound at 1 Hz that is a five second fuse burning during
        startup, and the measured result was a node that announced "exploration
        complete" twenty seconds after launch having driven to two goals. So
        termination is gated on `_started_exploring`: until this node has chosen
        a goal at least once, an empty cycle means "not ready yet", not "done".
        It never times out on that, because a node that is still waiting for a
        map has no business deciding the maze is explored.

        The second guard is the bound itself. The frontier grid genuinely dips:
        measured on lab3_maze_small it fell from 206 cells to 27 and recovered
        within a few seconds as SLAM redrew the map behind the robot, and 27
        scattered cells contain no run of five connected ones. Five consecutive
        empties is well inside that dip. Ten is not, and at 1 Hz against a 1 Hz
        map update it is ten independent looks at the world.

        Evidence tier: sim, lab3_maze_small, 2026-09-23.
        """
        if not self._started_exploring:
            self.get_logger().info(
                'waiting for the first frontier: '
                f'{decision.clusters_found} clusters on the current map',
                throttle_duration_sec=5.0)
            self._publish_markers(decision, position)
            return

        if self._try_forget_exhausted(decision):
            self._publish_markers(decision, position)
            return

        if self._try_look_around(decision, position):
            return

        if self._try_unstick(decision, position):
            return

        self._empty_cycles += 1
        self.get_logger().info(
            f'cycle {self._cycle}: nothing to explore '
            f'({self._empty_cycles}/{self.max_empty_cycles}), '
            f'{decision.clusters_found} clusters found, '
            f'{sum(1 for c in decision.candidates if not c.reachable)} unreachable')
        self._publish_markers(decision, position)
        if self._empty_cycles < self.max_empty_cycles:
            return

        self._finished = True
        self._path = []
        self._park(position)
        self._finished_at = position
        self.get_logger().info(
            f'EXPLORATION COMPLETE after {self._cycle} cycles, '
            f'{self._elapsed():.0f} s. No reachable frontier for '
            f'{self.max_empty_cycles} consecutive cycles. Parking.')
        self._publish_finished()

    def _try_forget_exhausted(self, decision: ExplorationDecision) -> bool:
        """Let the robot look again at frontiers it has already stood next to.

        Every run in the wide maze ends with the same line in the log, and it is
        not the line it looks like:

            nothing to explore (10/10), 3 clusters found, 0 unreachable

        Three clusters found and none of them unreachable, because none of them
        was ever scored. They were dropped before planning by the exhausted goal
        filter, which removes any goal within `goal.exhaust_radius` of one the
        robot has already arrived at or stalled against. So the node declared the
        maze explored while three frontiers stood in it.

        The filter earns its place. Without it H is memoryless and a frontier the
        robot is standing on keeps winning on distance forever, which is the
        arrive-reselect-arrive loop that cost a whole Gazebo run. The defect is
        that it is permanent. A goal is retired on arrival, and arrival means
        reaching the goal point, which `cluster_goal_point` deliberately walks
        back from the frontier for clearance. The robot can therefore arrive,
        retire the goal, never have observed the frontier itself, and be blind to
        it for the rest of the run.

        So exhaustion becomes forgetful rather than permanent. When the filter has
        emptied the candidate list and clusters are still there, the list is
        cleared and the next cycle starts again with everything on the table. It
        is bounded by `max_exhaust_resets` because an unbounded version cannot
        terminate: a frontier that genuinely cannot be cleared would be retried
        forever and the run would never end. Each reset buys another full pass at
        whatever is left, and when they run out the countdown proceeds as before.

        Measured across six runs in lab3_maze_wide on 2026-09-24, final coverage
        of the real maze ranged from 64.6 to 100 percent on one configuration, and
        every one of the short runs ended in this state with clusters still on the
        map. Evidence tier: sim, 2026-09-24.
        """
        if decision.clusters_found == 0:
            return False          # genuinely nothing on the map, not forgetting
        if decision.candidates:
            return False          # they were scored and lost, not filtered out
        if not self._exhausted:
            return False          # nothing to forget
        if self._exhaust_resets >= self.max_exhaust_resets:
            return False

        self._exhaust_resets += 1
        forgotten = len(self._exhausted)
        self._exhausted = []
        self._empty_cycles = 0
        self.get_logger().warn(
            f'cycle {self._cycle}: {decision.clusters_found} clusters still on '
            f'the map and every goal filtered as already visited. Forgetting '
            f'{forgotten} retired goals and trying again, '
            f'{self._exhaust_resets}/{self.max_exhaust_resets}')
        return True

    def _try_look_around(
        self, decision: ExplorationDecision, position: Point
    ) -> bool:
        """Turn on the spot when the robot is sealed in, before trying to move.

        This is the recovery that matches how the map goes wrong. slam_toolbox
        builds occupancy by ray tracing every scan in the pose graph: cells a
        beam passes through are evidence of free, the cell it ends in is
        evidence of occupied. A phantom wall is a cell that collected occupied
        evidence from a scan taken at a bad pose, and the only thing that
        removes it is later beams passing through it from somewhere the robot
        can actually see it from. New viewing angles are what clears it.

        Rotation provides those and translation barely does. The previous
        recovery drove to the furthest traversable point within 0.6 m, which is
        sound when the robot has somewhere to go and useless in the case that
        actually occurs: a pocket of 13 free cells offers a few centimetres of
        translation and essentially no new angles. A single turn on the spot
        sweeps a 360 degree LiDAR across every bearing.

        Doing it without a second publisher on `cmd_vel` takes one observation
        about the supplied follower. It forces linear velocity to zero whenever
        the heading error to its target exceeds 0.3 rad, and otherwise sets
        angular velocity to the error times a gain, clipped. So a waypoint
        placed at a large fixed bearing off the robot's nose is a pure rotation
        command: the error stays at `look_around_step` because this republishes
        it against the current heading every cycle, the follower keeps linear
        velocity at zero, and the robot turns at its limit without translating.
        The waypoint sits `look_around_radius` away, a few centimetres, so that
        if this stops republishing mid-turn the worst the follower can do is
        creep that far.

        Evidence tier: sim, 2026-09-24.
        """
        if decision.clusters_found == 0:
            return False  # genuinely nothing left, not stuck
        if any(c.reachable for c in decision.candidates):
            return False  # not stuck; selection simply had nothing better
        if self._look_around_cycles >= self.max_look_around_cycles:
            return False

        pose = self.get_robot_pose()
        if pose is None:
            return False
        yaw = pose[2]

        self._look_around_cycles += 1
        target = look_around_target(position, yaw, self.look_around_radius,
                                    self.look_around_step)
        self.get_logger().warn(
            f'cycle {self._cycle}: {decision.clusters_found} clusters, none '
            f'reachable. Turning in place to re-observe, '
            f'{self._look_around_cycles}/{self.max_look_around_cycles}')
        self._path = [position, target]
        self._path_age = 0.0
        self._committed_goal = None
        self._publish_path(self._path)
        # A turn is not a stall, and the progress watchdog measures translation.
        self._reset_progress(position)
        self._publish_markers(decision, position)
        return True

    def _try_unstick(
        self, decision: ExplorationDecision, position: Point
    ) -> bool:
        """Move a short distance when the robot is sealed in, before giving up.

        There is a failure that looks exactly like "the maze is explored" and is
        not. Frontier clusters are still being found, every one of them is
        reported unreachable, and the robot is standing still. What has happened
        is that the robot's own surroundings in the map have closed around it: a
        scatter of spurious occupied cells, each inflated by a 0.15 m collar,
        leaves its position in a small sealed pocket. Measured on a 7.2 m maze
        run, the pocket was 21 cells against 9096 traversable cells on the same
        map, and the six frontier goals outside it were all plainly reachable in
        reality.

        A stationary robot cannot map its way out of that, because the phantom
        walls only disappear when new scans contradict them. So the answer is to
        move: publish a short path to the nearest genuinely traversable point,
        let SLAM re-observe from somewhere else, and try again next cycle.

        Bounded by `max_unstick_attempts`, because a robot that is sealed in and
        cannot move is a robot that should stop rather than twitch forever. Any
        successful cycle resets the count.

        Returns True when a recovery path was published, in which case the cycle
        does not count toward termination: nothing was explored, but nothing was
        concluded either.
        """
        if decision.clusters_found == 0:
            return False  # genuinely nothing left, not stuck
        if any(c.reachable for c in decision.candidates):
            return False  # not stuck; the goal selection simply had nothing better
        if self._unstick_attempts >= self.max_unstick_attempts:
            return False

        try:
            occupancy = _as_occupancy_map(self._map)  # type: ignore[arg-type]
        except (ValueError, AttributeError):
            return False
        grid = PlanningGrid(occupancy, self.inflation_radius,
                            self.occupied_threshold, self.min_obstacle_neighbours)
        # Furthest, not nearest. The point of the move is to see the surroundings
        # from somewhere else; going to the nearest free point puts the robot
        # beside it and makes the next attempt a centimetre long.
        target = grid.farthest_unblocked(position, self.unstick_radius)
        if target is None or _distance(position, target) < self.unstick_min_distance:
            return False

        self._unstick_attempts += 1
        self.get_logger().warn(
            f'cycle {self._cycle}: {decision.clusters_found} clusters, none '
            f'reachable, own pose blocked={grid.is_blocked(position)}. '
            f'Unsticking {_distance(position, target):.2f} m to '
            f'({target[0]:.2f}, {target[1]:.2f}), attempt '
            f'{self._unstick_attempts}/{self.max_unstick_attempts}')
        self._path = [position, target]
        self._path_age = 0.0
        self._committed_goal = None
        self._publish_path(self._path)
        self._reset_progress(position)
        self._publish_markers(decision, position)
        return True

    def _park(self, position: Point) -> None:
        """Stop the robot with the only message the follower responds to.

        A single-waypoint path at the robot's own position. The follower's
        distance term goes to zero so it stops translating, then its yaw term
        chases `atan2(0.0, 0.0)`, which is 0.0, and it rotates to face world
        east before settling. The rotation is cosmetic and documented; what
        matters is that the robot stops driving.
        """
        self._publish_path([position])

    # ------------------------------------------------------------ publishing

    def _publish_path(self, points: Sequence[Point]) -> None:
        # Centre before densifying. Centring moves the planner's own waypoints,
        # which are the ones whose segments were collision checked; densifying
        # afterwards interpolates along the moved path so the follower still has
        # a waypoint inside its look-ahead everywhere. Doing it the other way
        # round would push a hundred interpolated points independently and bend
        # the path into something nobody checked.
        if self._grid is not None and self.path_centring_shift > 0.0:
            points = centre_path(points, self._grid, self.path_centring_shift)
        points = densify_path(points, self.waypoint_spacing)
        msg = Path()
        msg.header.frame_id = self.global_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        for index, point in enumerate(points):
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = float(point[0])
            pose.pose.position.y = float(point[1])
            # The follower ignores orientation entirely; it is set so RViz draws
            # arrows along the path rather than a row of identical ones, which is
            # the difference between a figure that shows direction and one that
            # does not.
            if index + 1 < len(points):
                nxt = points[index + 1]
                pose.pose.orientation = yaw_to_quaternion(
                    math.atan2(nxt[1] - point[1], nxt[0] - point[0]))
            elif index > 0:
                pose.pose.orientation = _pose_orientation(msg.poses[-1])
            else:
                pose.pose.orientation = yaw_to_quaternion(0.0)
            msg.poses.append(pose)
        self.path_pub.publish(msg)

    def _publish_markers(
        self, decision: ExplorationDecision, position: Point
    ) -> None:
        if not self.publish_markers:
            return
        self._publish_tree(decision)
        self._publish_goals(decision)
        self._publish_status(decision, position)

    def _publish_tree(self, decision: ExplorationDecision) -> None:
        """The winning candidate's tree, as one LINE_LIST.

        One Marker holding every edge rather than one Marker per edge: a tree of
        1500 nodes is 1499 edges, and 1499 markers is enough to make RViz
        unusable and enough traffic to matter on the lab router.
        """
        marker = Marker()
        marker.header.frame_id = self.global_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'rrt_tree'
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale = Vector3(x=0.01, y=0.0, z=0.0)
        marker.color = ColorRGBA(r=0.4, g=0.4, b=0.9, a=0.6)
        marker.pose.orientation.w = 1.0
        marker.lifetime = DurationMsg(sec=0)
        edges = decision.chosen.edges if decision.chosen is not None else []
        for start, end in edges:
            marker.points.append(PointMsg(x=float(start[0]), y=float(start[1]), z=0.0))
            marker.points.append(PointMsg(x=float(end[0]), y=float(end[1]), z=0.0))
        self.tree_pub.publish(marker)

    def _publish_goals(self, decision: ExplorationDecision) -> None:
        """One sphere per candidate: green chosen, blue reachable, red not."""
        array = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        for index, candidate in enumerate(decision.candidates):
            marker = Marker()
            marker.header.frame_id = self.global_frame
            marker.header.stamp = stamp
            marker.ns = 'frontier_goals'
            marker.id = index
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = float(candidate.goal[0])
            marker.pose.position.y = float(candidate.goal[1])
            marker.pose.orientation.w = 1.0
            marker.scale = Vector3(x=0.12, y=0.12, z=0.12)
            if candidate is decision.chosen:
                marker.color = ColorRGBA(r=0.1, g=0.9, b=0.2, a=0.9)
            elif candidate.reachable:
                marker.color = ColorRGBA(r=0.2, g=0.5, b=0.9, a=0.7)
            else:
                marker.color = ColorRGBA(r=0.9, g=0.2, b=0.2, a=0.7)
            array.markers.append(marker)
        # Markers are numbered from zero every cycle, so a cycle with fewer
        # candidates than the last one would leave the extras on screen forever.
        # DELETE on the tail rather than DELETEALL, which flickers the whole set.
        for index in range(len(decision.candidates), len(decision.candidates) + 12):
            stale = Marker()
            stale.header.frame_id = self.global_frame
            stale.header.stamp = stamp
            stale.ns = 'frontier_goals'
            stale.id = index
            stale.action = Marker.DELETE
            array.markers.append(stale)
        self.goals_pub.publish(array)

    def _elapsed(self) -> float:
        return (self.get_clock().now() - self._started_at).nanoseconds * 1e-9

    def _publish_finished(self) -> None:
        """The terminal status, republished every cycle once exploration ends.

        There is no "done" topic in the course package and nothing downstream
        subscribes to one, so this is a marker like the running status rather
        than a new interface. What matters is that it is unambiguous and that it
        is still being published when somebody looks, which is the part that was
        missing: the exit condition existed and fired, and left nothing behind
        that said so.
        """
        if self._finished_at is None:
            return
        marker = Marker()
        marker.header.frame_id = self.global_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'exploration_status'
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = float(self._finished_at[0])
        marker.pose.position.y = float(self._finished_at[1])
        marker.pose.position.z = 0.5
        marker.pose.orientation.w = 1.0
        marker.scale = Vector3(x=0.0, y=0.0, z=0.14)
        marker.color = ColorRGBA(r=0.2, g=1.0, b=0.2, a=1.0)
        marker.text = (f'EXPLORATION COMPLETE | {self._cycle} cycles | '
                       f'{self._elapsed():.0f} s')
        self.status_pub.publish(marker)

    def _publish_status(
        self, decision: ExplorationDecision, position: Point
    ) -> None:
        marker = Marker()
        marker.header.frame_id = self.global_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'exploration_status'
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = float(position[0])
        marker.pose.position.y = float(position[1])
        marker.pose.position.z = 0.5
        marker.pose.orientation.w = 1.0
        marker.scale = Vector3(x=0.0, y=0.0, z=0.12)
        marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.9)
        if decision.chosen is None:
            detail = f'idle {self._empty_cycles}/{self.max_empty_cycles}'
        else:
            detail = (f'H={decision.chosen.score:.2f} '
                      f'I={decision.chosen.info_gain:.0f} '
                      f'd={decision.chosen.path_length:.2f}m')
        marker.text = (f'cycle {self._cycle} | clusters {decision.clusters_found} | '
                       f'{detail}')
        self.status_pub.publish(marker)


# ------------------------------------------------------------------ helpers


def _as_occupancy_map(msg: OccupancyGrid) -> OccupancyMap:
    """Unpack a ROS message into the ROS-free representation grid.py defines.

    The single place in this package that reads OccupancyGrid field names.
    """
    return OccupancyMap.from_message(
        data=msg.data,
        width=msg.info.width,
        height=msg.info.height,
        resolution=msg.info.resolution,
        origin_x=msg.info.origin.position.x,
        origin_y=msg.info.origin.position.y,
    )


def _pose_orientation(pose: PoseStamped) -> Quaternion:
    return pose.pose.orientation


def _distance(a: Point, b: Point) -> float:
    return math.hypot(b[0] - a[0], b[1] - a[1])


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = NavigationNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # ExternalShutdownException is what rclpy raises when the launch system
        # signals the process, which is every ordinary Ctrl-C of the stack.
        # Letting it propagate exits 1 and prints a traceback, so a clean
        # shutdown looks like a crash in the log.
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
