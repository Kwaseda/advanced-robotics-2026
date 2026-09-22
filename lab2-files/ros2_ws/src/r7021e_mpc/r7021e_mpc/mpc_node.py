"""MPC position and trajectory tracking node.

    /odom            nav_msgs/Odometry               where the robot is
    /new_position    geometry_msgs/Pose              where it should be
    /reference_path  nav_msgs/Path                   where it should be over the horizon
    /cmd_vel         geometry_msgs/TwistStamped      what it is told to do
    /mpc_prediction  nav_msgs/Path                   the horizon the solver just planned
    /mpc_obstacles   visualization_msgs/MarkerArray  the keep-out zones, for RViz

The node owns the ROS side only: subscriptions, the timer, the message types, the header.
The control law is in mpc_controller.py, behind the Controller interface.

/reference_path is optional. An MPC scores every step of its horizon against the setpoint,
so holding one point constant across the horizon plans every future step toward a goal it
already knows will have moved. For a fixed setpoint that is correct and the topic is not
needed; for trajectory tracking it is the difference between tracking a curve and lagging
behind it.

The solver runs on its own timer at t_step, never inside the odometry callback, or it
would run at odometry rate and the horizon would span a different amount of real time on
every tick.
"""

from geometry_msgs.msg import Pose, PoseStamped, TwistStamped
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.exceptions import InvalidHandle
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from visualization_msgs.msg import Marker, MarkerArray

from .controller_base import Pose2D
from .geometry import yaw_from_quaternion
from .mpc_controller import MPCController, Obstacle


class MPCNode(Node):
    """Drives the robot to the setpoint on /new_position using an MPC."""

    def __init__(self) -> None:
        super().__init__('mpc_node')

        # Every number comes from YAML. Defaults match config/mpc.yaml.
        self.declare_parameter('t_step', 0.1)
        self.declare_parameter('n_horizon', 20)
        self.declare_parameter('goal_tolerance', 0.05)
        self.declare_parameter('cmd_frame_id', 'base_link')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('odom_timeout', 0.5)
        self.declare_parameter('reference_timeout', 1.0)

        # The lab's stated constraints.
        self.declare_parameter('lab.min_linear_velocity', 0.0)
        self.declare_parameter('lab.max_linear_velocity', 0.5)
        self.declare_parameter('lab.max_angular_velocity', 0.8)
        self.declare_parameter('boundary_x', [-1.0, 1.0])
        self.declare_parameter('boundary_y', [-1.0, 1.0])

        # The robot's real limits, from robot.yaml. Both sets apply; declared separately
        # so the tighter one can bind without either number being edited to mean both.
        self.declare_parameter('robot.max_linear_velocity', 0.22)
        self.declare_parameter('robot.max_angular_velocity', 2.84)

        self.declare_parameter('q_position', 1.0)
        self.declare_parameter('q_terminal', 1.0)
        self.declare_parameter('r_input', 0.01)
        self.declare_parameter('n_robust', 0)
        self.declare_parameter('solver_verbose', False)

        # Three parallel arrays rather than a list of structures, because ROS 2
        # parameters have no nested type. Declared without defaults so an empty obstacle
        # list is expressible.
        self.declare_parameter('obstacle_x', Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('obstacle_y', Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('obstacle_radius', Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('obstacle_inflation', 0.105)
        self.declare_parameter('obstacle_soft', False)
        self.declare_parameter('obstacle_penalty', 10000.0)

        t_step = self.get_parameter('t_step').value
        self.cmd_frame_id = self.get_parameter('cmd_frame_id').value
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.odom_timeout = self.get_parameter('odom_timeout').value
        self.reference_timeout = self.get_parameter('reference_timeout').value

        # A model allowed to predict 0.5 m/s on a robot that saturates at 0.22 plans a
        # future the robot cannot reach, and every plan is wrong in the same direction.
        max_v = min(self.get_parameter('lab.max_linear_velocity').value,
                    self.get_parameter('robot.max_linear_velocity').value)
        max_w = min(self.get_parameter('lab.max_angular_velocity').value,
                    self.get_parameter('robot.max_angular_velocity').value)
        min_v = self.get_parameter('lab.min_linear_velocity').value

        self.obstacles = Obstacle.from_arrays(
            self.parameter_array('obstacle_x'),
            self.parameter_array('obstacle_y'),
            self.parameter_array('obstacle_radius'),
            self.get_parameter('obstacle_inflation').value,
        )

        self.controller = MPCController(
            t_step=t_step,
            n_horizon=self.get_parameter('n_horizon').value,
            max_linear_velocity=max_v,
            max_angular_velocity=max_w,
            min_linear_velocity=min_v,
            x_bounds=self.get_parameter('boundary_x').value,
            y_bounds=self.get_parameter('boundary_y').value,
            obstacles=self.obstacles,
            q_position=self.get_parameter('q_position').value,
            q_terminal=self.get_parameter('q_terminal').value,
            r_input=self.get_parameter('r_input').value,
            goal_tolerance=self.get_parameter('goal_tolerance').value,
            n_robust=self.get_parameter('n_robust').value,
            solver_verbose=self.get_parameter('solver_verbose').value,
            obstacle_soft=self.get_parameter('obstacle_soft').value,
            obstacle_penalty=self.get_parameter('obstacle_penalty').value,
        )

        # None until the first message of each kind arrives. With no odometry the
        # controller does not know where it is; with no setpoint it has no goal.
        self.pose = None
        self.last_odom_time = None
        self.goal = None
        self.reference = None
        self.last_reference_time = None
        self.goal_reached_announced = False
        self.stopped = True
        self.infeasible_announced = False

        self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(Pose, '/new_position', self.on_goal, 10)
        self.create_subscription(Path, '/reference_path', self.on_reference, 10)

        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.prediction_pub = self.create_publisher(Path, '/mpc_prediction', 10)
        self.obstacle_pub = self.create_publisher(MarkerArray, '/mpc_obstacles', 10)

        self.timer = self.create_timer(t_step, self.on_control_tick)
        # The markers are static; republishing at 1 Hz costs nothing and means an RViz
        # started later still sees them.
        self.marker_timer = self.create_timer(1.0, self.publish_obstacles)

        self.get_logger().info(
            'mpc_node up. t_step=%.3f s (%.0f Hz), horizon=%d (%.1f s lookahead), '
            'v in [%.2f, %.2f] m/s, omega in [%.2f, %.2f] rad/s, box x%s y%s, '
            '%d %s obstacle(s), inflation %.3f m'
            % (
                t_step, 1.0 / t_step,
                self.controller.n_horizon, self.controller.n_horizon * t_step,
                min_v, max_v, -max_w, max_w,
                tuple(self.controller.x_bounds), tuple(self.controller.y_bounds),
                len(self.obstacles),
                'soft' if self.get_parameter('obstacle_soft').value else 'hard',
                self.get_parameter('obstacle_inflation').value,
            )
        )

    def parameter_array(self, name):
        """Read a double array parameter that may legitimately be unset."""
        try:
            value = self.get_parameter(name).value
        except rclpy.exceptions.ParameterUninitializedException:
            return []
        return list(value) if value is not None else []

    def on_odom(self, msg: Odometry) -> None:
        """Store the pose: the three states of the unicycle model, nothing else."""
        q = msg.pose.pose.orientation
        self.pose = Pose2D(
            x=msg.pose.pose.position.x,
            y=msg.pose.pose.position.y,
            theta=yaw_from_quaternion(q.x, q.y, q.z, q.w),
        )
        self.last_odom_time = self.get_clock().now()

    def on_goal(self, msg: Pose) -> None:
        """Read the setpoint. Position only; the cost scores x and y, not heading."""
        new_goal = (msg.position.x, msg.position.y)
        if self.goal is None or new_goal != self.goal:
            self.goal_reached_announced = False
            self.infeasible_announced = False
            self.controller.reset()
        self.goal = new_goal

    def on_reference(self, msg: Path) -> None:
        """Store the reference's future over the horizon. Index k is k steps ahead."""
        self.reference = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.last_reference_time = self.get_clock().now()

    def age(self, stamp) -> float:
        """Seconds since `stamp`, or infinity if nothing has arrived yet."""
        if stamp is None:
            return float('inf')
        return (self.get_clock().now() - stamp).nanoseconds * 1e-9

    def publish(self, v: float, omega: float) -> None:
        """Publish one velocity command, stamped.

        An unstamped header on a stamped message is a message that arrived at the epoch.
        Gazebo does not notice; a real robot with a tf tree does, silently.
        """
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.cmd_frame_id
        msg.twist.linear.x = v
        msg.twist.angular.z = omega
        self.cmd_pub.publish(msg)

    def stop(self, reason: str) -> None:
        """Command zero velocity once, and say why the first time."""
        if not self.stopped:
            self.get_logger().warn('stopping: %s' % reason)
        self.stopped = True
        self.publish(0.0, 0.0)

    def publish_obstacles(self) -> None:
        """Draw the keep-out zones in RViz from the same numbers the solver uses."""
        markers = MarkerArray()
        for index, obstacle in enumerate(self.obstacles):
            marker = Marker()
            marker.header.frame_id = self.odom_frame_id
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'mpc_obstacles'
            marker.id = index
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = obstacle.x
            marker.pose.position.y = obstacle.y
            marker.pose.position.z = 0.1
            marker.pose.orientation.w = 1.0
            # Diameter, and the inflated one: what is drawn is the circle the solver
            # enforces, so a path that hugs it reads as correct rather than as a near miss.
            marker.scale.x = 2.0 * obstacle.radius
            marker.scale.y = 2.0 * obstacle.radius
            marker.scale.z = 0.2
            marker.color.r = 0.9
            marker.color.g = 0.2
            marker.color.b = 0.2
            marker.color.a = 0.35
            markers.markers.append(marker)
        if markers.markers:
            self.obstacle_pub.publish(markers)

    def publish_prediction(self) -> None:
        """Publish the horizon the solver just planned, as a drawable Path."""
        points = self.controller.predicted_path()
        if not points:
            return
        path = Path()
        path.header.frame_id = self.odom_frame_id
        path.header.stamp = self.get_clock().now().to_msg()
        for x, y in points:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.prediction_pub.publish(path)

    def on_control_tick(self) -> None:
        """One receding horizon solve, at t_step."""
        if self.pose is None:
            # Not even zero: that would fight a teleop node legitimately driving.
            return
        if self.age(self.last_odom_time) > self.odom_timeout:
            self.stop('odometry older than %.2f s' % self.odom_timeout)
            return
        if self.goal is None:
            return

        # A stale reference is worse than none: it describes where the trajectory was
        # going before its publisher stopped.
        if self.reference and self.age(self.last_reference_time) <= self.reference_timeout:
            self.controller.set_reference_horizon(self.reference)
        else:
            self.controller.set_reference_horizon(None)

        goal_x, goal_y = self.goal
        command = self.controller.compute(self.pose, goal_x, goal_y)
        _, _, distance = self.controller.tracking_error(self.pose, goal_x, goal_y)
        self.publish_prediction()

        if not self.controller.last_success:
            if not self.infeasible_announced:
                self.get_logger().warn(
                    'solver did not converge: %s. holding position at (%.3f, %.3f), '
                    'goal (%.3f, %.3f) is %.3f m away'
                    % (self.controller.last_status, self.pose.x, self.pose.y,
                       goal_x, goal_y, distance))
                self.infeasible_announced = True
            self.stopped = True
            self.publish(0.0, 0.0)
            return
        self.infeasible_announced = False

        if command.v == 0.0 and command.omega == 0.0:
            if not self.goal_reached_announced:
                self.get_logger().info(
                    'goal reached: error %.3f m, inside tolerance %.3f m'
                    % (distance, self.controller.goal_tolerance))
                self.goal_reached_announced = True
            self.stopped = True
            self.publish(0.0, 0.0)
            return

        self.stopped = False
        self.publish(command.v, command.omega)

        # Solve time is in the line because it decides whether t_step is achievable.
        self.get_logger().info(
            'error %.3f m, v %.3f m/s, omega %.3f rad/s, solve %.1f ms of %.0f ms budget'
            % (distance, command.v, command.omega,
               self.controller.last_solve_time * 1e3, self.controller.t_step * 1e3),
            throttle_duration_sec=1.0,
        )

    def stop_if_possible(self) -> None:
        """Best effort zero command on the way out.

        On Ctrl-C the context is already shut down and this does nothing; it earns its
        place for every other exit. Not a safety guarantee.
        """
        try:
            if rclpy.ok():
                self.publish(0.0, 0.0)
        except (RCLError, InvalidHandle):
            pass


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MPCNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # ExternalShutdownException is what rclpy raises on Ctrl-C, not KeyboardInterrupt.
        pass
    finally:
        node.stop_if_possible()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
