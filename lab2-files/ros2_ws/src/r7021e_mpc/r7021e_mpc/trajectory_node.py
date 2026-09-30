"""Publishes the task 4 circle as a moving goal.

/new_position is the current point. /reference_path is where the point will be at each
of the MPC's next horizon steps, so the controller can plan ahead instead of lagging.
"""

from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Path
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from .trajectories import CircleTrajectory


class TrajectoryNode(Node):

    def __init__(self):
        super().__init__('trajectory_node')
        self.declare_parameter('radius', 0.8)
        self.declare_parameter('centre_x', 0.0)
        self.declare_parameter('centre_y', 0.0)
        self.declare_parameter('lap_period', 60.0)
        self.declare_parameter('laps', 2)
        self.declare_parameter('start_delay', 8.0)
        self.declare_parameter('publish_period', 0.05)
        # Must match the MPC's n_horizon and t_step; the launch file passes the same values.
        self.declare_parameter('n_horizon', 20)
        self.declare_parameter('t_step', 0.1)

        p = lambda name: self.get_parameter(name).value
        self.circle = CircleTrajectory(p('radius'), p('centre_x'), p('centre_y'),
                                       p('lap_period'), p('laps'), p('start_delay'))
        self.n_horizon = p('n_horizon')
        self.t_step = p('t_step')

        self.goal_pub = self.create_publisher(Pose, '/new_position', 10)
        self.path_pub = self.create_publisher(Path, '/reference_path', 10)
        # Start timing on the first tick: with sim time the clock reads zero until /clock arrives.
        self.start_time = None
        self.finished = False
        self.create_timer(p('publish_period'), self.on_tick)
        self.get_logger().info('trajectory_node up: circle r=%.2f m, %.0f s per lap, %d laps'
                               % (p('radius'), p('lap_period'), p('laps')))

    def on_tick(self):
        now = self.get_clock().now()
        if self.start_time is None:
            self.start_time = now
        elapsed = (now - self.start_time).nanoseconds * 1e-9

        x, y, finished = self.circle.point_at(elapsed)
        goal = Pose()
        goal.position.x = x
        goal.position.y = y
        goal.orientation.w = 1.0
        self.goal_pub.publish(goal)

        path = Path()
        path.header.frame_id = 'odom'
        path.header.stamp = now.to_msg()
        for k in range(self.n_horizon + 1):
            px, py, _ = self.circle.point_at(elapsed + k * self.t_step)
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = px
            pose.pose.position.y = py
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.path_pub.publish(path)

        if finished and not self.finished:
            self.get_logger().info('laps done, holding the final point')
            self.finished = True


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():  # errors after Ctrl-C are shutdown noise
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
