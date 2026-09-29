#!/usr/bin/env python3
"""Check that a simulation bag's clean /odom stayed close to the true pose.

The node logs clean /odom as ground truth; that is only fair if Gazebo's
wheels did not slip. Needs /ground_truth (maze_sim.launch.py ground_truth:=true).

    python3 tools/check_ground_truth.py <bag>
"""
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from r7021e_fast_slam.utils import matrix_to_pose, pose_to_matrix, wrap_angle


def _yaw(q):
    return float(np.arctan2(2.0 * (q.w * q.z + q.x * q.y),
                            1.0 - 2.0 * (q.y * q.y + q.z * q.z)))


def _rebase(poses):
    inv0 = np.linalg.inv(pose_to_matrix(poses[0, 1:]))
    return np.array([matrix_to_pose(inv0 @ pose_to_matrix(p[1:])) for p in poses])


def main():
    if len(sys.argv) != 2:
        raise SystemExit(__doc__)
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if '/ground_truth' not in types:
        raise SystemExit('no /ground_truth in this bag: record it with '
                         'maze_sim.launch.py ground_truth:=true')
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/odom', '/ground_truth']))
    odom, truth = [], []
    ## Bag receive time: the pose bridge leaves its stamps at zero.
    while reader.has_next():
        topic, data, t = reader.read_next()
        msg = deserialize_message(data, get_message(types[topic]))
        if topic == '/odom':
            p = msg.pose.pose
            odom.append((t * 1e-9, p.position.x, p.position.y, _yaw(p.orientation)))
        elif msg.transforms:
            tr = msg.transforms[0].transform
            truth.append((t * 1e-9, tr.translation.x, tr.translation.y, _yaw(tr.rotation)))
    odom, truth = np.array(odom), np.array(truth)
    o, g = _rebase(odom), _rebase(truth)
    idx = np.clip(np.searchsorted(truth[:, 0], odom[:, 0]), 0, len(truth) - 1)
    err = np.hypot(*(o[:, :2] - g[idx, :2]).T)
    head = np.degrees(np.abs(wrap_angle(o[:, 2] - g[idx, 2])))
    path = np.sum(np.hypot(*np.diff(g[:, :2], axis=0).T))
    print(f'path length {path:.1f} m over {odom[-1, 0] - odom[0, 0]:.0f} s')
    print(f'clean /odom vs true pose: max {err.max():.3f} m, final {err[-1]:.3f} m, '
          f'max heading {head.max():.1f} deg')
    verdict = 'OK' if err.max() < 0.10 else 'TOO FAR: drive more gently and re-record'
    print(f'verdict: {verdict}')


if __name__ == '__main__':
    main()
