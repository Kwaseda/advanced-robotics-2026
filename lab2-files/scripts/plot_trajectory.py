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

For Lab 2, pass --constraints <mpc_task*.yaml> to draw the boundary box and the obstacle
keep-out circles from the same parameter file the controller was launched with. The
circles are drawn at the inflated radius, which is the constraint the solver actually
enforced, so a path that appears to touch a circle is touching the real constraint rather
than a decorative one. Reading them from the config rather than retyping them is the
point: a plot whose obstacles disagree with the controller's is worse than no plot.
"""

import argparse
import pathlib

import matplotlib.pyplot as plt
import yaml
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


def draw_constraints(ax, constraints_path: str, base_path: str) -> None:
    """Draw the boundary box and the inflated obstacle circles from the parameter files.

    The overlay file carries whatever the task changed; the base file carries the rest,
    including obstacle_inflation. Loading both in the same order the launch file does
    keeps the picture and the constraint in agreement.
    """
    def parameters(path):
        with open(path) as handle:
            return yaml.safe_load(handle)['/mpc_node']['ros__parameters']

    merged = {}
    for path in (base_path, constraints_path):
        if pathlib.Path(path).is_file():
            merged.update(parameters(path))

    box_x = merged.get('boundary_x')
    box_y = merged.get('boundary_y')
    if box_x and box_y:
        ax.plot(
            [box_x[0], box_x[1], box_x[1], box_x[0], box_x[0]],
            [box_y[0], box_y[0], box_y[1], box_y[1], box_y[0]],
            ':', color='#8A5CF6', linewidth=1.5, label='boundary constraint')

    inflation = merged.get('obstacle_inflation', 0.0)
    xs = merged.get('obstacle_x') or []
    ys = merged.get('obstacle_y') or []
    radii = merged.get('obstacle_radius') or []
    for index, (ox, oy, radius) in enumerate(zip(xs, ys, radii)):
        # Two circles per obstacle: the physical object, and the inflated radius the
        # solver enforced. The gap between them is the robot's own footprint, and
        # showing both is what makes a path that hugs the outer circle readable as
        # correct rather than as a near miss.
        ax.add_patch(plt.Circle((ox, oy), radius, color='#D85A30', alpha=0.45,
                                label='obstacle' if index == 0 else None))
        ax.add_patch(plt.Circle((ox, oy), radius + inflation, color='#D85A30', alpha=0.15,
                                label='enforced keep-out (inflated)' if index == 0 else None))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', help='path to the bag directory')
    parser.add_argument('--odom-topic', default='/odom')
    parser.add_argument('--goal-topic', default='/new_position')
    parser.add_argument('-o', '--out', default=None,
                         help='output image path, defaults to <bag name>_trajectory.png')
    parser.add_argument('--constraints', default=None,
                         help='Lab 2 mpc_task*.yaml, to draw the boundary box and the '
                              'obstacles the controller actually enforced')
    parser.add_argument('--base-config', default=None,
                         help='where obstacle_inflation is defined. Defaults to mpc.yaml '
                              'beside --constraints, which is where it lives in both '
                              'workspace layouts')
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

    if args.constraints:
        # Default the base file to a sibling of the overlay rather than a fixed relative
        # path: the two workspaces nest config/ differently, and a base file that quietly
        # fails to load would draw the obstacles without inflation, which is a picture
        # that disagrees with the constraint the solver enforced.
        base = args.base_config or str(pathlib.Path(args.constraints).with_name('mpc.yaml'))
        draw_constraints(ax, args.constraints, base)

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
