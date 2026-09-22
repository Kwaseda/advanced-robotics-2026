"""Trajectory setpoint generator: a figure of eight or a circle.

    /new_position    geometry_msgs/Pose  the goal the controller chases
    /reference_path  nav_msgs/Path       where that goal will be over the next N steps

Published here rather than generated inside the controller, so /new_position stays
the single control entry point: a terminal setpoint, this node, or a later planner
all drive the same controller through the same message.

The curve itself is in trajectories.py; `shape` picks which one.

/reference_path is off by default. An MPC scores every step of its horizon against the
setpoint, so handing it one point and holding it constant plans every future step toward
a goal it already knows will have moved. The path carries the reference's own future,
sampled at the controller's t_step. A controller that cannot use it simply ignores it.
"""

import math

from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Path
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from .geometry import quaternion_from_yaw
from .trajectories import CircleTrajectory, FigureEightTrajectory

SHAPE_EIGHT = 'eight'
SHAPE_CIRCLE = 'circle'
SHAPE_CHOICES = (SHAPE_EIGHT, SHAPE_CIRCLE)


class TrajectoryNode(Node):
    """Publishes the chosen curve as a stream of position setpoints."""

    def __init__(self) -> None:
        super().__init__('trajectory_node')

        # Defaults match config/circle.yaml for the circle; the eight's numbers are
        # kept so the same node still serves a figure of eight.
        self.declare_parameter('shape', SHAPE_EIGHT)
        self.declare_parameter('radius', 0.8)
        self.declare_parameter('width', 1.0)
        self.declare_parameter('height', 0.5)
        self.declare_parameter('centre_x', 0.0)
        self.declare_parameter('centre_y', 0.0)
        self.declare_parameter('lap_period', 50.0)
        self.declare_parameter('laps', 2)
        self.declare_parameter('publish_period', 0.05)
        self.declare_parameter('start_delay', 3.0)

        self.declare_parameter('publish_reference_path', False)
        self.declare_parameter('reference_horizon', 20)
        self.declare_parameter('reference_step', 0.1)
        self.declare_parameter('reference_frame_id', 'odom')

        shape = self.get_parameter('shape').value
        if shape not in SHAPE_CHOICES:
            raise ValueError('shape must be one of %r, got %r' % (SHAPE_CHOICES, shape))
        self.shape = shape

        common = dict(
            centre_x=self.get_parameter('centre_x').value,
            centre_y=self.get_parameter('centre_y').value,
            lap_period=self.get_parameter('lap_period').value,
            laps=self.get_parameter('laps').value,
            start_delay=self.get_parameter('start_delay').value,
        )
        if shape == SHAPE_CIRCLE:
            self.trajectory = CircleTrajectory(
                radius=self.get_parameter('radius').value, **common)
        else:
            self.trajectory = FigureEightTrajectory(
                width=self.get_parameter('width').value,
                height=self.get_parameter('height').value, **common)

        self.publish_reference = self.get_parameter('publish_reference_path').value
        self.reference_horizon = self.get_parameter('reference_horizon').value
        self.reference_step = self.get_parameter('reference_step').value
        self.reference_frame_id = self.get_parameter('reference_frame_id').value

        self.goal_pub = self.create_publisher(Pose, '/new_position', 10)
        self.path_pub = (
            self.create_publisher(Path, '/reference_path', 10)
            if self.publish_reference else None
        )
        self.finished_announced = False

        # Clock starts on the first tick, not here: under use_sim_time the clock
        # reads zero until the first /clock message arrives, so a start time
        # captured here would put lap one in the past.
        self.start_time = None
        publish_period = self.get_parameter('publish_period').value
        self.timer = self.create_timer(publish_period, self.on_publish_tick)

        if shape == SHAPE_CIRCLE:
            span = 'circle of radius %.2f m' % self.trajectory.radius
        else:
            span = 'figure of eight %.2f by %.2f m' % (
                2.0 * self.trajectory.width, 2.0 * self.trajectory.height)

        self.get_logger().info(
            'trajectory_node up. %s about (%.2f, %.2f), '
            '%.1f s per lap, %d laps, %.1f s start delay, %.0f Hz%s'
            % (
                span,
                self.trajectory.centre_x,
                self.trajectory.centre_y,
                self.trajectory.lap_period,
                self.trajectory.laps,
                self.trajectory.start_delay,
                1.0 / publish_period,
                (', publishing /reference_path over %d steps of %.3f s'
                 % (self.reference_horizon, self.reference_step))
                if self.publish_reference else '',
            )
        )

    def on_publish_tick(self) -> None:
        """Publish the setpoint for the current time."""
        now = self.get_clock().now()
        if self.start_time is None:
            self.start_time = now
        elapsed = (now - self.start_time).nanoseconds * 1e-9

        x, y, finished = self.trajectory.point_at(elapsed)

        msg = Pose()
        msg.position.x = x
        msg.position.y = y
        # Orientation set to the path's tangent rather than left as an invalid
        # all-zero quaternion. The controller ignores it; this is so a bag of this
        # topic can still be drawn by other tools.
        tangent = self.heading_at(elapsed)
        qx, qy, qz, qw = quaternion_from_yaw(tangent)
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw
        self.goal_pub.publish(msg)

        if self.path_pub is not None:
            self.path_pub.publish(self.reference_path_at(elapsed))

        if finished and not self.finished_announced:
            self.get_logger().info(
                '%d laps done after %.1f s, holding the final setpoint at (%.3f, %.3f)'
                % (self.trajectory.laps, elapsed, x, y)
            )
            self.finished_announced = True

    def reference_path_at(self, elapsed: float) -> Path:
        """Where the setpoint will be at each step of the controller's horizon.

        reference_step must match the controller's t_step, or index k means a different
        amount of time to each of them. The launch file passes the same number to both.
        """
        path = Path()
        path.header.frame_id = self.reference_frame_id
        path.header.stamp = self.get_clock().now().to_msg()
        for k in range(self.reference_horizon + 1):
            x, y, _ = self.trajectory.point_at(elapsed + k * self.reference_step)
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        return path

    def heading_at(self, elapsed: float) -> float:
        """Direction the setpoint is travelling, radians.

        Finite difference over one publish period, since the value only fills a
        quaternion nothing steers on.
        """
        step = 0.05
        x0, y0, _ = self.trajectory.point_at(elapsed)
        x1, y1, _ = self.trajectory.point_at(elapsed + step)
        if x1 == x0 and y1 == y0:
            return 0.0
        return math.atan2(y1 - y0, x1 - x0)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrajectoryNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # rclpy raises ExternalShutdownException on Ctrl-C, not KeyboardInterrupt.
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
