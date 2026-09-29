"""The injected odometry noise must not depend on the filter.

Two node runs with different filters (FS1 at N=7, FS2 at N=3) and different
filter seeds replay the same odometry and scans. The corrupted odometry the
filters see has to be identical, or the Task 7 sweep compares different
inputs. This is the one test that needs ROS; it skips without rclpy.
"""
import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')

from conftest import PARAMS_YAML, simulate_scan  # noqa: E402


def _odom(t, x, y, yaw):
    from nav_msgs.msg import Odometry
    m = Odometry()
    m.header.stamp.sec = int(t)
    m.header.stamp.nanosec = int((t - int(t)) * 1e9)
    m.pose.pose.position.x = float(x)
    m.pose.pose.position.y = float(y)
    m.pose.pose.orientation.z = float(np.sin(yaw / 2))
    m.pose.pose.orientation.w = float(np.cos(yaw / 2))
    return m


def _scan(t, pose):
    from sensor_msgs.msg import LaserScan
    _, ranges = simulate_scan(pose)
    m = LaserScan()
    m.header.stamp.sec = int(t)
    m.header.stamp.nanosec = int((t - int(t)) * 1e9)
    m.angle_min = -np.pi
    m.angle_increment = 2 * np.pi / 360
    m.range_min = 0.12
    m.range_max = 3.5
    m.ranges = [float(r) for r in ranges]
    return m


def _run(overrides):
    rclpy.init(args=['--ros-args', '--params-file', str(PARAMS_YAML),
                     '-p', 'log_dir:=/tmp/r7021e_fast_slam_test_runs',
                     *sum((['-p', o] for o in overrides), [])])
    try:
        from r7021e_fast_slam.grid_slam_node import GridSlamNode
        node = GridSlamNode()
        seen = []
        for k in range(1, 101):                     # 2 s of odometry at 50 Hz
            t = 0.02 * k
            pose = (0.1 * t, 0.02 * t, 0.2 * t)
            node.odom_cb(_odom(t, *pose))
            if k % 10 == 0:                         # a scan every 0.2 s
                node.scan_cb(_scan(t, pose))
            seen.append(node.odom_pose.copy())
        node.destroy_node()
        return np.array(seen)
    finally:
        rclpy.shutdown()


def test_corrupted_odometry_is_independent_of_the_filter():
    a = _run(['num_particles:=3', 'seed:=1', 'use_improved_proposal:=true'])
    b = _run(['num_particles:=7', 'seed:=5', 'use_improved_proposal:=false'])
    np.testing.assert_array_equal(a, b)
    ## And the noise is really there: the corrupted path is not the clean one.
    assert np.abs(a[-1, 0] - 0.1 * 2.0) > 1e-4
