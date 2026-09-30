"""MPC node for Lab 2.

Subscribes /odom, /new_position and (task 4) /reference_path.
Publishes /cmd_vel, plus /mpc_prediction and /mpc_obstacles for RViz.
"""

import math

from geometry_msgs.msg import Pose, PoseStamped, TwistStamped
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from visualization_msgs.msg import Marker, MarkerArray

from .mpc_controller import MPCController, inflate_obstacles

# A trajectory moves the goal a few mm per message; only a real jump should drop the warm start.
GOAL_JUMP = 0.1


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class MPCNode(Node):

    def __init__(self):
        super().__init__('mpc_node')

        self.declare_parameter('t_step', 0.1)
        self.declare_parameter('n_horizon', 20)
        self.declare_parameter('goal_tolerance', 0.05)
        self.declare_parameter('odom_timeout', 0.5)
        self.declare_parameter('max_linear_velocity', 0.22)
        self.declare_parameter('max_angular_velocity', 0.8)
        self.declare_parameter('min_linear_velocity', 0.0)
        self.declare_parameter('boundary_x', [-1.0, 1.0])
        self.declare_parameter('boundary_y', [-1.0, 1.0])
        self.declare_parameter('q_position', 1.0)
        self.declare_parameter('q_terminal', 1.0)
        self.declare_parameter('r_input', 0.01)
        # No default, so a task with no obstacles simply leaves them out of its YAML.
        self.declare_parameter('obstacle_x', Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('obstacle_y', Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('obstacle_radius', Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('obstacle_inflation', 0.105)
        self.declare_parameter('obstacle_soft', False)
        self.declare_parameter('obstacle_penalty', 10000.0)

        p = lambda name: self.get_parameter(name).value
        t_step = p('t_step')
        self.odom_timeout = p('odom_timeout')

        self.obstacles = inflate_obstacles(self.array('obstacle_x'), self.array('obstacle_y'),
                                           self.array('obstacle_radius'), p('obstacle_inflation'))
        self.controller = MPCController(
            t_step=t_step, n_horizon=p('n_horizon'),
            max_linear_velocity=p('max_linear_velocity'),
            max_angular_velocity=p('max_angular_velocity'),
            min_linear_velocity=p('min_linear_velocity'),
            x_bounds=p('boundary_x'), y_bounds=p('boundary_y'), obstacles=self.obstacles,
            q_position=p('q_position'), q_terminal=p('q_terminal'), r_input=p('r_input'),
            goal_tolerance=p('goal_tolerance'),
            obstacle_soft=p('obstacle_soft'), obstacle_penalty=p('obstacle_penalty'))

        self.pose = None
        self.last_odom_time = None
        self.goal = None
        self.reference = None
        self.announced = None

        self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(Pose, '/new_position', self.on_goal, 10)
        self.create_subscription(Path, '/reference_path', self.on_reference, 10)
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.prediction_pub = self.create_publisher(Path, '/mpc_prediction', 10)
        self.obstacle_pub = self.create_publisher(MarkerArray, '/mpc_obstacles', 10)

        # The solver runs on its own timer at t_step, not in the odom callback.
        self.create_timer(t_step, self.on_control_tick)
        self.create_timer(1.0, self.publish_obstacles)

        self.get_logger().info(
            'mpc_node up: t_step %.2f s, horizon %d, v <= %.2f m/s, |omega| <= %.2f rad/s, '
            'box x%s y%s, %d %s obstacle(s)' % (
                t_step, p('n_horizon'), p('max_linear_velocity'), p('max_angular_velocity'),
                p('boundary_x'), p('boundary_y'), len(self.obstacles),
                'soft' if p('obstacle_soft') else 'hard'))

    def array(self, name):
        try:
            return list(self.get_parameter(name).value or [])
        except rclpy.exceptions.ParameterUninitializedException:
            return []

    def on_odom(self, msg):
        pose = msg.pose.pose
        self.pose = (pose.position.x, pose.position.y, yaw_from_quaternion(pose.orientation))
        self.last_odom_time = self.get_clock().now()

    def on_goal(self, msg):
        goal = (msg.position.x, msg.position.y)
        if self.goal is None or math.dist(goal, self.goal) > GOAL_JUMP:
            self.controller.reset()
            self.announced = None
        self.goal = goal

    def on_reference(self, msg):
        self.reference = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def publish(self, v, omega):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = v
        msg.twist.angular.z = omega
        self.cmd_pub.publish(msg)

    def announce(self, level, kind, text):
        """Log a stop reason once, not ten times a second."""
        if self.announced != kind:
            level(text)
            self.announced = kind

    def on_control_tick(self):
        if self.pose is None or self.goal is None:
            return
        age = (self.get_clock().now() - self.last_odom_time).nanoseconds * 1e-9
        if age > self.odom_timeout:
            self.announce(self.get_logger().warn, 'odom', 'no odometry for %.1f s, stopping' % age)
            self.publish(0.0, 0.0)
            return

        x, y, theta = self.pose
        goal_x, goal_y = self.goal
        self.controller.set_reference_horizon(self.reference)
        v, omega = self.controller.compute(x, y, theta, goal_x, goal_y)
        distance = math.hypot(goal_x - x, goal_y - y)
        self.publish_prediction()
        self.publish(v, omega)

        if not self.controller.last_success:
            self.announce(self.get_logger().warn, 'failed', 'solver failed (%s), holding at (%.3f, %.3f)'
                          % (self.controller.last_status, x, y))
        elif v == 0.0 and omega == 0.0:
            self.announce(self.get_logger().info, 'reached', 'goal reached, error %.3f m' % distance)
        else:
            self.announced = None
            self.get_logger().info(
                'error %.3f m, v %.3f, omega %.3f, solve %.0f ms'
                % (distance, v, omega, self.controller.last_solve_time * 1e3),
                throttle_duration_sec=1.0)

    def publish_prediction(self):
        path = Path()
        path.header.frame_id = 'odom'
        path.header.stamp = self.get_clock().now().to_msg()
        for px, py in self.controller.predicted_path():
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = px
            pose.pose.position.y = py
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        if path.poses:
            self.prediction_pub.publish(path)

    def publish_obstacles(self):
        # Drawn at the inflated radius, which is what the solver actually enforces.
        markers = MarkerArray()
        for i, (ox, oy, radius) in enumerate(self.obstacles):
            m = Marker()
            m.header.frame_id = 'odom'
            m.ns = 'mpc_obstacles'
            m.id = i
            m.type = Marker.CYLINDER
            m.pose.position.x = ox
            m.pose.position.y = oy
            m.pose.position.z = 0.1
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = 2.0 * radius
            m.scale.z = 0.2
            m.color.r, m.color.g, m.color.b, m.color.a = 0.9, 0.2, 0.2, 0.35
            markers.markers.append(m)
        if markers.markers:
            self.obstacle_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = MPCNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        # Ctrl-C can land mid-solve, and the publish after it then fails. Only re-raise real errors.
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
