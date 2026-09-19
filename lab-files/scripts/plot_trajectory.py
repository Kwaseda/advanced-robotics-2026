#!/usr/bin/env python3
"""Plot the robot's actual trajectory against the commanded/provided one, from a bag.

Usage (ROS environment must be sourced first, for rosbag2_py and the message types):

    source /opt/ros/jazzy/setup.bash
    source <workspace>/install/setup.bash
    python3 scripts/plot_trajectory.py bags/task1-position -o task1_trajectory.png

Reads /odom (nav_msgs/Odometry) for the robot's actual path and /new_position
(geometry_msgs/Pose) for the commanded setpoints, and plots both in 2D. Works for
any task that publishes those two topics -- a handful of discrete setpoints (task 1)
plot as a scatter of goals; a continuously-published trajectory (task 2, the figure
eight) plots as a line.
"""

import argparse
import pathlib

import matplotlib.pyplot as plt
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py


def read_xy(bag_path: str, topic: str):
    """Return parallel (t, x, y) lists for every message on `topic` in the bag.

    Works for both nav_msgs/Odometry and geometry_msgs/Pose: both carry a `position`
    (directly, or under `pose.pose`), and this reads whichever shape is present.
    """
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='mcap')
    converter_options = rosbag2_py.ConverterOptions('', '')
    reader.open(storage_options, converter_options)

    type_by_topic = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topic not in type_by_topic:
        return [], [], []
    msg_type = get_message(type_by_topic[topic])

    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))

    t0 = None
    times, xs, ys = [], [], []
    while reader.has_next():
        _, data, t = reader.read_next()
        msg = deserialize_message(data, msg_type)
        position = msg.pose.pose.position if hasattr(msg, 'pose') else msg.position
        if t0 is None:
            t0 = t
        times.append((t - t0) * 1e-9)
        xs.append(position.x)
        ys.append(position.y)
    return times, xs, ys


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', help='path to the bag directory')
    parser.add_argument('--odom-topic', default='/odom')
    parser.add_argument('--goal-topic', default='/new_position')
    parser.add_argument('-o', '--out', default=None,
                         help='output image path, defaults to <bag name>_trajectory.png')
    args = parser.parse_args()

    _, robot_x, robot_y = read_xy(args.bag, args.odom_topic)
    _, goal_x, goal_y = read_xy(args.bag, args.goal_topic)

    if not robot_x:
        raise SystemExit(f'no messages found on {args.odom_topic} in {args.bag}')

    fig, ax = plt.subplots(figsize=(7, 7))
    ax.plot(robot_x, robot_y, '-', color='#1D9E75', linewidth=1.5, label='robot trajectory (/odom)')
    ax.plot(robot_x[0], robot_y[0], 'o', color='#5F5E5A', markersize=8, label='start')

    if goal_x:
        # Distinguish "a moving trajectory" from "a few repeated discrete goals" by
        # counting distinct setpoints, not raw message count -- a terminal goal
        # republished at 5 Hz for a few seconds is still one goal, not a path.
        distinct = {(round(x, 3), round(y, 3)) for x, y in zip(goal_x, goal_y)}
        if len(distinct) > 20:
            # Continuously published setpoint (a moving trajectory): draw as a line.
            ax.plot(goal_x, goal_y, '--', color='#D85A30', linewidth=1.5,
                    label='provided trajectory (/new_position)')
        else:
            # A handful of discrete terminal setpoints: draw as goal markers, not
            # connected by a line that was never actually commanded.
            dx, dy = zip(*distinct)
            ax.plot(dx, dy, 'x', color='#D85A30', markersize=10, markeredgewidth=2,
                    linestyle='none', label='commanded goal(s) (/new_position)')

    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.legend()
    ax.set_title(pathlib.Path(args.bag).name)

    out = args.out or f'{pathlib.Path(args.bag).name}_trajectory.png'
    fig.savefig(out, dpi=150, bbox_inches='tight')
    print(f'saved {out}')


if __name__ == '__main__':
    main()
