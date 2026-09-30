#!/usr/bin/env python3
"""Plot the robot's path (/odom) and the goals (/new_position) from a bag.

    python3 scripts/plot_trajectory.py bags/task2-obstacle \
        --constraints ros2_ws/src/r7021e_mpc_bringup/config/mpc_task2.yaml -o task2.png

--constraints draws the boundary box and obstacles from the same YAML the controller used.
Source ROS and the workspace first.
"""

import argparse
import pathlib

import matplotlib.pyplot as plt
from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message
import yaml


def read_xy(bag_path, topic):
    """x and y of every Odometry or Pose message on `topic`."""
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_path, storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topic not in types:
        return [], []
    msg_type = get_message(types[topic])
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    xs, ys = [], []
    while reader.has_next():
        msg = deserialize_message(reader.read_next()[1], msg_type)
        position = msg.pose.pose.position if hasattr(msg, 'pose') else msg.position
        xs.append(position.x)
        ys.append(position.y)
    return xs, ys


def load_parameters(*paths):
    """Merge the ros__parameters of several YAML files, later files winning."""
    merged = {}
    for path in paths:
        if pathlib.Path(path).is_file():
            data = yaml.safe_load(open(path))
            block = data.get('/**') or data.get('/mpc_node') or {}
            merged.update(block.get('ros__parameters', {}))
    return merged


def draw_constraints(ax, params):
    box_x, box_y = params.get('boundary_x'), params.get('boundary_y')
    if box_x and box_y:
        ax.plot([box_x[0], box_x[1], box_x[1], box_x[0], box_x[0]],
                [box_y[0], box_y[0], box_y[1], box_y[1], box_y[0]],
                ':', color='#8A5CF6', linewidth=1.5, label='boundary constraint')
    inflation = params.get('obstacle_inflation', 0.0)
    obstacles = zip(params.get('obstacle_x') or [], params.get('obstacle_y') or [],
                    params.get('obstacle_radius') or [])
    for i, (ox, oy, r) in enumerate(obstacles):
        # The physical obstacle, and the inflated circle the solver actually enforces.
        ax.add_patch(plt.Circle((ox, oy), r, color='#D85A30', alpha=0.45,
                                label='obstacle' if i == 0 else None))
        ax.add_patch(plt.Circle((ox, oy), r + inflation, color='#D85A30', alpha=0.15,
                                label='enforced keep-out (inflated)' if i == 0 else None))


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('bag')
    parser.add_argument('-o', '--out')
    parser.add_argument('--constraints', help='mpc_task<N>.yaml')
    parser.add_argument('--base-config', help='defaults to mpc.yaml beside --constraints')
    args = parser.parse_args()

    robot_x, robot_y = read_xy(args.bag, '/odom')
    goal_x, goal_y = read_xy(args.bag, '/new_position')
    if not robot_x:
        raise SystemExit('no /odom messages in %s' % args.bag)

    fig, ax = plt.subplots(figsize=(7, 7))
    ax.plot(robot_x, robot_y, '-', color='#1D9E75', linewidth=1.5, label='robot trajectory (/odom)')
    ax.plot(robot_x[0], robot_y[0], 'o', color='#5F5E5A', markersize=8, label='start')

    if goal_x:
        distinct = {(round(x, 3), round(y, 3)) for x, y in zip(goal_x, goal_y)}
        if len(distinct) > 20:  # a moving reference, not a few repeated goals
            ax.plot(goal_x, goal_y, '--', color='#D85A30', linewidth=1.5,
                    label='provided trajectory (/new_position)')
        else:
            dx, dy = zip(*distinct)
            ax.plot(dx, dy, 'x', color='#D85A30', markersize=10, markeredgewidth=2,
                    linestyle='none', label='commanded goal(s) (/new_position)')

    if args.constraints:
        base = args.base_config or pathlib.Path(args.constraints).with_name('mpc.yaml')
        draw_constraints(ax, load_parameters(base, args.constraints))

    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.legend()
    ax.set_title(pathlib.Path(args.bag).name)
    out = args.out or '%s_trajectory.png' % pathlib.Path(args.bag).name
    fig.savefig(out, dpi=150, bbox_inches='tight')
    print('saved', out)


if __name__ == '__main__':
    main()
