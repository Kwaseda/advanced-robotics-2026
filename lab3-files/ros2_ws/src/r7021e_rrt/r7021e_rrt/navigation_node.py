#!/usr/bin/env python3
"""The Lab 3 navigation loop: the only file in this package that imports rclpy.

Tasks 1 and 5 of the lab instructions both live here. Task 1 is "send a Path to
the follower and send a new one when the robot reaches the end"; Task 5 is that
same loop with the goal chosen by the exploration gain instead of written down.
They are the same node, so building Task 1 as a throwaway script would have meant
building it twice.

Everything algorithmic is somewhere else. This file owns subscriptions, timers,
message types, headers, tf and parameters, and nothing that could be tested at a
desk, which is why the planner and the gain have unit tests at all.

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
status, done or goal-reached topic anywhere in it, so completion is detected here
by comparing the tf pose against the last waypoint.

Stopping is the same problem one step further on. The follower pops waypoints
only `while len(self.path) > 1`, so its list never empties and it keeps
publishing forever; and it returns early on an empty path *without* publishing
zero, so there is no message that stops it. What does work is a path holding a
single waypoint at the robot's own position: the distance term goes to zero and
the robot stops translating. It then rotates to face world east, because
`atan2(0.0, 0.0)` is 0.0, and parks there. That is the park manoeuvre this node
performs on termination. The rotation is cosmetic; what matters is that the
robot stops driving.

The alternative was publishing zero `TwistStamped` ourselves, which would put a
second publisher on `cmd_vel` against a follower that never stops publishing.
Two publishers on one `cmd_vel` do not error, they interleave, and the robot
does something that looks like a tuning problem and is not.

The frontier topic defect
-------------------------
`frontier_detector_node.py` publishes on `frontiers`. The course template's
navigation node declares `frontier_topic` defaulting to `frontier`, singular, and
`exploration.launch.py` carries no remapping, so as shipped that subscription
never receives anything. This node defaults to the plural, the name the publisher
actually uses.
"""

from __future__ import annotations

import math
from typing import Sequence

import rclpy
from builtin_interfaces.msg import Duration as DurationMsg
from geometry_msgs.msg import PoseStamped, Point as PointMsg, Quaternion, Vector3
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
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
from .rrt_star import RRTStarPlanner, densify_path

Point = tuple[float, float]

# Two different profiles, and they are not interchangeable.
#
# slam_toolbox publishes /map TRANSIENT_LOCAL, so a late subscriber still gets
# the current map instead of waiting up to a full map_update_interval for the
# next one. The course's frontier_detector_node creates its publisher with a bare
# `QoSProfile(depth=1)`, which is VOLATILE, so /frontiers is volatile whatever we
# would prefer.
#
# DDS does not fall back: an incompatible subscription receives nothing at all.
# Both sides log it, and the wording is worth recognising:
#
#   [frontier_detector] New subscription discovered on topic 'frontiers',
#       requesting incompatible QoS. No messages will be sent to it.
#       Last incompatible policy: DURABILITY
#   [navigation_node] New publisher discovered on topic 'frontiers', offering
#       incompatible QoS. No messages will be received from it.
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

        self.declare_parameter('gain.mode', 'reduced_range')
        self.declare_parameter('gain.radius', 0.75)
        self.declare_parameter('gain.weight_per_cell', 0.10)

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
        self._unstick_attempts = 0
        self._cycle = 0
        self._finished = False
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
            return

        pose = self.get_robot_pose()
        if pose is None or self._map is None or self._frontier is None:
            return
        position = (pose[0], pose[1])

        self._track_progress(position)
        reason = self._replan_reason(position)
        if reason is None:
            return

        self._retire_goal(reason)

        try:
            decision = self._decide(position)
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
        self._unstick_attempts = 0
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
        surviving arrival: leaving it in place produces a loop where the robot
        arrives at a goal, finds the frontier still there, holds the commitment,
        replans 0.3 m to the same goal and arrives again.

        Releasing the commitment alone does not fix it, because the same goal then
        wins on merit anyway: it is the nearest frontier, and nothing in
        H = sum(d) - w I knows the robot has already been there. So a goal that
        has been arrived at, or that the robot stalled trying to reach, is
        retired: no candidate within `goal.exhaust_radius` of it is offered again
        for the rest of the run.

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

    def _decide(self, position: Point) -> ExplorationDecision:
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

        Two guards, and both are needed.

        The node starts before SLAM has published anything, so its first ticks
        legitimately see no map and no frontiers. Termination is therefore gated
        on `_started_exploring`: until this node has chosen a goal at least once,
        an empty cycle means "not ready yet", not "done". It never times out on
        that, because a node still waiting for a map has no business deciding the
        maze is explored.

        The second guard is the bound itself. The frontier count genuinely dips
        and recovers as SLAM redraws the map behind the robot, from a couple of
        hundred cells to a couple of dozen, and a couple of dozen scattered cells
        contain no run of five connected ones. Ten consecutive empty cycles at
        1 Hz, against a 1 Hz map update, is ten independent looks at the world.
        """
        if not self._started_exploring:
            self.get_logger().info(
                'waiting for the first frontier: '
                f'{decision.clusters_found} clusters on the current map',
                throttle_duration_sec=5.0)
            self._publish_markers(decision, position)
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
        self.get_logger().info(
            f'exploration complete after {self._cycle} cycles. Parking.')

    def _try_unstick(
        self, decision: ExplorationDecision, position: Point
    ) -> bool:
        """Move a short distance when the robot is sealed in, before giving up.

        There is a failure that looks exactly like "the maze is explored" and is
        not. Frontier clusters are still being found, every one of them is
        reported unreachable, and the robot is standing still. What has happened
        is that the robot's own surroundings in the map have closed around it: a
        scatter of spurious occupied cells, each inflated by a 0.15 m collar,
        leaves its position in a small sealed pocket. On a 7.2 m maze run the
        pocket was 21 cells against 9096 traversable cells on the same map, and
        the six frontier goals outside it were plainly reachable in reality.

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
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
