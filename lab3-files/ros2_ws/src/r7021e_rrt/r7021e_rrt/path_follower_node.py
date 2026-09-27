#!/usr/bin/env python3
"""ROS wrapper around `path_follower.follow`.

Drop-in for the supplied `r7021e_exploration/path_follower_node`: same `path`
subscription, same `cmd_vel` publication, same `TwistStamped`, same parameter
names for the five the course declares. The launch file picks one or the other
with `follower:=rrt` or `follower:=course`, which is what makes the two
comparable in a single run of the harness.

Two behaviours differ from the supplied node and both are deliberate.

It publishes zero. The supplied node returns early on an empty path without
sending anything, and the Gazebo differential drive plugin has no command
timeout, so the last velocity stands until something replaces it. Measured over
two runs on 2026-09-24 the stack never actually leaves a gap, because the
navigation node parks by publishing a waypoint rather than by stopping, so this
has never bitten. It is still the difference between a follower that stops when
told and one that happens not to be asked. On the physical robot that is a
safety property, not a tidiness one.

It survives a missing transform. The supplied node calls `lookup_transform`
outside any handler, so the first cycle before tf is populated raises through
the timer callback. Here a failed lookup stops the robot and logs at most once a
second, because a follower that cannot see where it is has no business guessing.
"""

import math

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import (Buffer, ConnectivityException, ExtrapolationException,
                     LookupException, TransformListener)

from r7021e_rrt.path_follower import (FollowerLimits, follow,
                                      forward_clearance, reactive_limit)


class PathFollower(Node):

    def __init__(self) -> None:
        super().__init__('path_follower')

        # The five the course declares, with the course's own defaults, so that
        # a parameter file written for its node works unchanged against this one.
        self.declare_parameter('max_v', 0.15)
        self.declare_parameter('kp_vel', 1.0)
        self.declare_parameter('max_w', 1.0)
        self.declare_parameter('kp_yaw', 2.0)
        self.declare_parameter('look_ahead', 0.2)
        # The two this law adds.
        self.declare_parameter('goal_tolerance', 0.05)
        self.declare_parameter('turn_in_place', 1.2)
        self.declare_parameter('stop_distance', 0.18)
        self.declare_parameter('slow_distance', 0.30)
        self.declare_parameter('half_width', 0.10)
        self.declare_parameter('scan_timeout', 0.5)
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('robot_frame', 'base_link')

        self.limits = FollowerLimits(
            max_v=float(self.get_parameter('max_v').value),
            max_w=float(self.get_parameter('max_w').value),
            kp_vel=float(self.get_parameter('kp_vel').value),
            kp_yaw=float(self.get_parameter('kp_yaw').value),
            look_ahead=float(self.get_parameter('look_ahead').value),
            goal_tolerance=float(self.get_parameter('goal_tolerance').value),
            turn_in_place=float(self.get_parameter('turn_in_place').value),
            stop_distance=float(self.get_parameter('stop_distance').value),
            slow_distance=float(self.get_parameter('slow_distance').value),
            half_width=float(self.get_parameter('half_width').value),
        )
        self.scan_timeout = float(self.get_parameter('scan_timeout').value)
        self.global_frame = str(self.get_parameter('global_frame').value)
        self.robot_frame = str(self.get_parameter('robot_frame').value)

        self._path: list[tuple[float, float]] = []
        self._frame = self.global_frame
        self._index = 0
        # True rather than False so the first tick, which runs before any path
        # has arrived, is not reported as a path completing.
        self._arrived = True
        self._last_warn = 0.0

        self._clearance = float('inf')
        self._scan_at = None

        self.create_subscription(Path, 'path', self._on_path, 1)
        self.create_subscription(LaserScan, 'scan', self._on_scan,
                                 qos_profile_sensor_data)
        self._cmd = self.create_publisher(TwistStamped, 'cmd_vel', 1)

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.create_timer(0.1, self._tick)

    def _on_path(self, msg: Path) -> None:
        # A new path resets the index. The old index counted waypoints in a list
        # that no longer exists, and carrying it over would skip the start of the
        # new path by however far the robot had got along the old one.
        self._frame = msg.header.frame_id or self.global_frame
        self._path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self._index = 0
        self._arrived = False

    def _on_scan(self, msg: LaserScan) -> None:
        self._clearance = forward_clearance(
            msg.ranges, msg.angle_min, msg.angle_increment, self.limits,
            msg.range_min, msg.range_max)
        self._scan_at = self.get_clock().now().nanoseconds * 1e-9

    def _tick(self) -> None:
        pose = self._robot_pose()
        if pose is None:
            self._publish(0.0, 0.0)
            return

        position, yaw = pose
        command = follow(self._path, position, yaw, self.limits, self._index)
        self._index = command.index
        # A stale scan is not a clear path. The LiDAR runs at 5 Hz against this
        # 10 Hz loop, so one missing scan is normal and a second is not.
        now = self.get_clock().now().nanoseconds * 1e-9
        fresh = (self._scan_at is not None
                 and now - self._scan_at <= self.scan_timeout)
        if not fresh and command.v > 0.0:
            # Refusing to drive on a stale scan is the safe way round, and it is
            # also indistinguishable from a robot that simply will not move, so
            # it says so rather than sitting there.
            if now - self._last_warn > 1.0:
                self._last_warn = now
                self.get_logger().warn(
                    'no recent scan, holding forward motion at zero')
        command = reactive_limit(command,
                                 self._clearance if fresh else 0.0,
                                 self.limits)
        if command.arrived and not self._arrived:
            self._arrived = True
            self.get_logger().info('path complete, holding position')
        self._publish(command.v, command.w)

    def _robot_pose(self):
        try:
            ts = self.buffer.lookup_transform(
                self.global_frame, self.robot_frame, rclpy.time.Time())
        except (LookupException, ConnectivityException,
                ExtrapolationException) as exc:
            now = self.get_clock().now().nanoseconds * 1e-9
            if now - self._last_warn > 1.0:
                self._last_warn = now
                self.get_logger().warn(f'no {self.global_frame} to '
                                       f'{self.robot_frame} transform: {exc}')
            return None
        t = ts.transform.translation
        q = ts.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return (t.x, t.y), yaw

    def _publish(self, v: float, w: float) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame
        msg.twist.linear.x = float(v)
        msg.twist.angular.z = float(w)
        self._cmd.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PathFollower()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # ExternalShutdownException is what rclpy raises when the launch system
        # signals the process, which is every ordinary Ctrl-C of the stack.
        # Letting it propagate exits 1 and prints a traceback, so a clean
        # shutdown looks like a crash in the log.
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
