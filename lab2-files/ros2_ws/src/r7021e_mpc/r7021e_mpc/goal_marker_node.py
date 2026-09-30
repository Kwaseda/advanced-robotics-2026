"""Redraws /new_position as a marker, since a bare Pose has no frame RViz can draw."""

from geometry_msgs.msg import Pose
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from visualization_msgs.msg import Marker


class GoalMarkerNode(Node):
    def __init__(self) -> None:
        super().__init__('goal_marker_node')

        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('marker_scale', 0.08)

        self.frame_id = self.get_parameter('frame_id').value
        self.marker_scale = self.get_parameter('marker_scale').value

        self.create_subscription(Pose, '/new_position', self.on_goal, 10)
        self.marker_pub = self.create_publisher(Marker, '/goal_marker', 10)

        self.get_logger().info(
            'goal_marker_node up. drawing /new_position as a %.2f m sphere in %s'
            % (self.marker_scale, self.frame_id)
        )

    def on_goal(self, msg: Pose) -> None:
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'goal'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = msg.position.x
        marker.pose.position.y = msg.position.y
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.marker_scale
        marker.scale.y = self.marker_scale
        marker.scale.z = self.marker_scale
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        self.marker_pub.publish(marker)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GoalMarkerNode()
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
