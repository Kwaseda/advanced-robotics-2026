#!/usr/bin/env python3
"""Replay a bag through the real grid_slam_node, offline, every scan processed.

Live replay drops scans once a filter step takes longer than the scan period
(from about N = 20), so large-N runs would see different input. This builds
the same GridSlamNode and calls its callbacks in bag order instead. If the bag
has /ground_truth, the true pose per logged step is saved as
runs/<run_name>.truth.npy.

    python3 tools/offline_replay.py <bag> <run_name> [-p name:=value ...]
"""
import argparse
from pathlib import Path

import numpy as np
import rclpy
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

HERE = Path(__file__).resolve().parent
DEFAULT_PARAMS = HERE.parent / 'config' / 'params.yaml'


def _yaw(q):
    return float(np.arctan2(2.0 * (q.w * q.z + q.x * q.y),
                            1.0 - 2.0 * (q.y * q.y + q.z * q.z)))


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('bag')
    ap.add_argument('run_name')
    ap.add_argument('--params-file', default=str(DEFAULT_PARAMS))
    ap.add_argument('--storage', default='mcap', help='mcap or sqlite3')
    ap.add_argument('-p', action='append', default=[], metavar='NAME:=VALUE',
                    help='parameter override, repeatable')
    args = ap.parse_args()

    ros_args = ['--ros-args', '--params-file', args.params_file,
                '-p', f'run_name:={args.run_name}']
    for p in args.p:
        ros_args += ['-p', p]
    rclpy.init(args=ros_args)

    from r7021e_fast_slam.grid_slam_node import GridSlamNode
    from r7021e_fast_slam.utils import matrix_to_pose, pose_to_matrix
    node = GridSlamNode()

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=args.bag, storage_id=args.storage),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    topics = [t for t in ('/scan', '/odom', '/tf_static', '/ground_truth')
              if t in types]
    missing = {'/scan', '/odom'} - set(topics)
    if missing:
        raise SystemExit(f'bag has no {sorted(missing)}')
    reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
    msg_type = {t: get_message(types[t]) for t in topics}

    ## transforms[0] of Gazebo's pose bridge is the model pose.
    truth = truth0 = None
    truth_rows = []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        msg = deserialize_message(data, msg_type[topic])
        if topic == '/ground_truth':
            if msg.transforms:
                tr = msg.transforms[0].transform
                truth = np.array([tr.translation.x, tr.translation.y,
                                  _yaw(tr.rotation)])
        elif topic == '/odom':
            if truth0 is None and truth is not None:
                truth0 = truth.copy()
            node.odom_cb(msg)
        elif topic == '/scan':
            before = len(node.logger._rows)
            node.scan_cb(msg)
            if len(node.logger._rows) > before:
                truth_rows.append(
                    np.full(3, np.nan) if truth0 is None else matrix_to_pose(
                        np.linalg.inv(pose_to_matrix(truth0)) @ pose_to_matrix(truth)))
        else:
            for tf in msg.transforms:
                node.tf_buffer.set_transform_static(tf, 'bag')

    node.shutdown()
    if truth_rows and np.isfinite(truth_rows[-1]).all():
        out = node.logger.dir / f'{args.run_name}.truth.npy'
        np.save(out, np.array(truth_rows))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
