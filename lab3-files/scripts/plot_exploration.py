#!/usr/bin/env python3
"""Render one exploration run from a bag: map, frontiers, RRT* tree, path, trajectory.

Usage (ROS environment must be sourced first, for rosbag2_py and the message types):

    source /opt/ros/jazzy/setup.bash
    source ~/ros2_ws/install/setup.bash
    python3 scripts/plot_exploration.py bags/lab3-small -o lab3_small.png

    # the coverage curve instead of the map figure
    python3 scripts/plot_exploration.py bags/lab3-small --coverage -o lab3_coverage.png

Why this exists rather than an extension of plot_trajectory.py
---------------------------------------------------------------
plot_trajectory.py draws a robot trajectory against a set of constraints read
from a parameter file. It has no concept of an occupancy grid, and the four
things this lab's report needs to show at once are a grid, a second grid drawn
over it, a tree of line segments and a polyline. Extending it would have meant
one script with two disjoint halves.

What the figure shows, and where each layer comes from
-------------------------------------------------------
    /map              the SLAM occupancy grid, greyscale, unknown left blank
    /frontiers        frontier cells, drawn as an overlay
    /frontier_goals   the scored candidates, green chosen, blue reachable,
                      red unreachable, in the colours the node publishes
    /rrt_tree         the winning candidate's tree, as line segments
    /path             the path that was sent to the follower
    /tf               where the robot actually went, in the map frame

By default every layer is taken from the last message on its topic, so the
figure is the end of the run with the whole driven trajectory on it. `--at`
takes a time in seconds from the start of the bag instead, which is how you get
a mid-run figure showing a tree that has not yet been replaced.

The coverage curve
------------------
`--coverage` plots explored free area against time, as a percentage of what the
run eventually reached, and marks the two times the optional competition scores.
`--summary` prints the same run's numbers as absolute areas in square metres,
which is what makes two runs comparable: the occupancy grid is sized to whatever
the pose graph covers, so its cell count is not the same denominator twice.
"""

import argparse
import math
import pathlib

import matplotlib
import numpy as np

matplotlib.use('Agg')  # no display in a headless sweep; must precede pyplot
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.collections import LineCollection  # noqa: E402

from rclpy.serialization import deserialize_message  # noqa: E402
from rosidl_runtime_py.utilities import get_message  # noqa: E402
import rosbag2_py  # noqa: E402


def read_topic(bag_path: str, topic: str):
    """Yield (seconds_from_bag_start, message) for every message on `topic`.

    A topic that is not in the bag yields nothing rather than raising. That is
    deliberate: a bag recorded before the markers existed, or from a run with
    publish_markers false, should still produce a map and a trajectory.
    """
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id='mcap'),
        rosbag2_py.ConverterOptions('', ''),
    )
    type_by_topic = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topic not in type_by_topic:
        return
    msg_type = get_message(type_by_topic[topic])
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))

    t0 = None
    while reader.has_next():
        _, data, stamp = reader.read_next()
        if t0 is None:
            t0 = stamp
        yield (stamp - t0) * 1e-9, deserialize_message(data, msg_type)


# A Burger tops out at 0.22 m/s and base_footprint arrives at 50 Hz, so real
# motion cannot exceed about 4.4 mm per sample. Three times that separates
# driving from a map to odom correction.
MAX_REAL_STEP_M = 3 * 0.22 / 50


def robot_track(bag_path: str, at: float | None) -> list[list[tuple[float, float]]]:
    """The robot's path in the MAP frame, composed from /tf, split at corrections.

    Not /odom: /odom is in the odom frame, which drifts, and drawing it over a
    map-frame map plots the drift as motion. This composes
    map -> base_link = (map -> odom) * (odom -> base_link), both planar.

    Every loop closure rewrites map to odom and the composed pose steps without
    the robot moving, up to 1.74 m in one sample here. Joined into one polyline
    those draw as straight lines through walls and summed they inflate distance
    driven, so the track is returned as segments broken at each such step.
    """
    map_to_odom = (0.0, 0.0, 0.0)   # x, y, yaw
    segments: list[list[tuple[float, float]]] = [[]]
    for seconds, msg in read_topic(bag_path, '/tf'):
        if at is not None and seconds > at:
            break
        for transform in msg.transforms:
            t = transform.transform.translation
            yaw = _quat_yaw(transform.transform.rotation)
            if transform.child_frame_id.endswith('odom'):
                map_to_odom = (t.x, t.y, yaw)
            elif transform.child_frame_id.endswith('base_footprint') or \
                    transform.child_frame_id.endswith('base_link'):
                mx, my, myaw = map_to_odom
                cos_y, sin_y = math.cos(myaw), math.sin(myaw)
                point = (mx + cos_y * t.x - sin_y * t.y,
                         my + sin_y * t.x + cos_y * t.y)
                if segments[-1] and math.dist(segments[-1][-1], point) > MAX_REAL_STEP_M:
                    segments.append([])
                segments[-1].append(point)
    return [seg for seg in segments if len(seg) > 1]


def track_points(segments) -> tuple[list[float], list[float]]:
    """Every point of every segment, for callers that only need positions."""
    xs = [x for seg in segments for x, _ in seg]
    ys = [y for seg in segments for _, y in seg]
    return xs, ys


def track_length(segments) -> float:
    """Distance driven, summed inside segments so frame corrections are excluded."""
    return sum(math.dist(seg[i], seg[i + 1])
               for seg in segments for i in range(len(seg) - 1))


def _quat_yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def last_before(bag_path: str, topic: str, at: float | None):
    """The last message on `topic` at or before `at` seconds, or None."""
    chosen = None
    for seconds, msg in read_topic(bag_path, topic):
        if at is not None and seconds > at:
            break
        chosen = msg
    return chosen


def grid_extent(info) -> tuple[float, float, float, float]:
    """(left, right, bottom, top) in metres, for imshow.

    imshow's extent is measured to the outer edges of the image, not to cell
    centres, so this adds no half cell. That differs from grid.cell_to_world on
    purpose: one names a pixel's boundary, the other names a cell's middle.
    """
    return (
        info.origin.position.x,
        info.origin.position.x + info.width * info.resolution,
        info.origin.position.y,
        info.origin.position.y + info.height * info.resolution,
    )


def draw_map(ax, map_msg) -> None:
    grid = np.asarray(map_msg.data, dtype=np.int16).reshape(
        (map_msg.info.height, map_msg.info.width))
    # Unknown is masked rather than drawn as a value. Plotting -1 on the same
    # scale as 0 to 100 makes unknown space look like confidently free space,
    # which in an exploration figure is the one thing that must not happen.
    shown = np.ma.masked_where(grid < 0, grid)
    cmap = plt.get_cmap('Greys').copy()
    cmap.set_bad(color='#e9e4d9')
    ax.imshow(shown, origin='lower', extent=grid_extent(map_msg.info),
              cmap=cmap, vmin=0, vmax=100, interpolation='nearest')


def draw_frontiers(ax, frontier_msg) -> None:
    grid = np.asarray(frontier_msg.data, dtype=np.int16).reshape(
        (frontier_msg.info.height, frontier_msg.info.width))
    shown = np.ma.masked_where(grid != 100, grid)
    cmap = matplotlib.colors.ListedColormap(['#d81b60'])
    ax.imshow(shown, origin='lower', extent=grid_extent(frontier_msg.info),
              cmap=cmap, interpolation='nearest', alpha=0.9)


def draw_tree(ax, marker) -> None:
    """The LINE_LIST marker: consecutive pairs of points are one segment each."""
    points = [(p.x, p.y) for p in marker.points]
    segments = [(points[i], points[i + 1]) for i in range(0, len(points) - 1, 2)]
    if not segments:
        return
    ax.add_collection(LineCollection(
        segments, colors='#5566dd', linewidths=0.4, alpha=0.5, zorder=2))
    ax.plot([], [], color='#5566dd', linewidth=0.8,
            label=f'RRT* tree ({len(segments)} edges)')


def draw_goals(ax, array) -> None:
    """Candidate goals, in the colours the node already chose.

    Reading the colour off the message rather than recomputing it keeps this
    figure and RViz showing the same thing. Markers with action DELETE are the
    tail cleanup the node publishes and carry no position.
    """
    for marker in array.markers:
        if marker.action != 0:  # 0 is ADD
            continue
        colour = (marker.color.r, marker.color.g, marker.color.b)
        ax.plot(marker.pose.position.x, marker.pose.position.y, marker='o',
                markersize=7, color=colour, markeredgecolor='black',
                markeredgewidth=0.5, zorder=5)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('bag', help='path to the bag directory')
    parser.add_argument('-o', '--out', default=None,
                        help='output PNG. Defaults to <bag>_exploration.png')
    parser.add_argument('--at', type=float, default=None,
                        help='seconds from the start of the bag. Default: the end')
    parser.add_argument('--coverage', action='store_true',
                        help='plot the coverage curve instead of the map figure')
    parser.add_argument('--summary', action='store_true',
                        help='print the run\'s numbers and exit, drawing nothing')
    parser.add_argument('--map-topic', default='/map')
    parser.add_argument('--frontier-topic', default='/frontiers')
    parser.add_argument('--path-topic', default='/path')
    parser.add_argument('--odom-topic', default='/odom')
    parser.add_argument('--title', default=None)
    args = parser.parse_args()

    bag = args.bag
    out = args.out or f'{pathlib.Path(bag).name}_exploration.png'

    if args.summary:
        print_summary(bag, args.map_topic)
        return

    if args.coverage:
        plot_coverage(bag, args.map_topic, out, args.title)
        return

    fig, ax = plt.subplots(figsize=(9, 9))

    map_msg = last_before(bag, args.map_topic, args.at)
    if map_msg is None:
        raise SystemExit(f'no {args.map_topic} messages in {bag}')
    draw_map(ax, map_msg)

    frontier_msg = last_before(bag, args.frontier_topic, args.at)
    if frontier_msg is not None:
        draw_frontiers(ax, frontier_msg)
        ax.plot([], [], color='#d81b60', linewidth=4, label='frontier cells')

    tree = last_before(bag, '/rrt_tree', args.at)
    if tree is not None:
        draw_tree(ax, tree)

    goals = last_before(bag, '/frontier_goals', args.at)
    if goals is not None:
        draw_goals(ax, goals)
        ax.plot([], [], marker='o', linestyle='', color=(0.1, 0.9, 0.2),
                markeredgecolor='black', label='chosen goal')
        ax.plot([], [], marker='o', linestyle='', color=(0.2, 0.5, 0.9),
                markeredgecolor='black', label='scored candidate')

    segments = robot_track(bag, args.at)
    if not segments:
        # No tf in the bag. Fall back to raw odom and say so on the figure,
        # because the two are not the same curve.
        fallback: list[tuple[float, float]] = []
        for seconds, odom in read_topic(bag, args.odom_topic):
            if args.at is not None and seconds > args.at:
                break
            fallback.append((odom.pose.pose.position.x, odom.pose.pose.position.y))
        if fallback:
            segments = [fallback]
            ax.set_xlabel('x [m]  (trajectory from /odom, uncorrected)')
    if segments:
        # One plot call per segment, so gaps at loop closures stay gaps.
        for i, seg in enumerate(segments):
            ax.plot([x for x, _ in seg], [y for _, y in seg],
                    color='#ff8c00', linewidth=1.6, zorder=4,
                    label='driven trajectory' if i == 0 else None)
        ax.plot(segments[0][0][0], segments[0][0][1], marker='s', color='black',
                markersize=7, zorder=6, label='start')

    path_msg = last_before(bag, args.path_topic, args.at)
    if path_msg is not None and path_msg.poses:
        px = [p.pose.position.x for p in path_msg.poses]
        py = [p.pose.position.y for p in path_msg.poses]
        ax.plot(px, py, color='#00a000', linewidth=2.2, zorder=6,
                label=f'planned path ({len(px)} waypoints)')

    ax.set_aspect('equal')
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_title(args.title or f'Exploration, {pathlib.Path(bag).name}')
    ax.legend(loc='upper right', fontsize=8, framealpha=0.9)
    ax.grid(alpha=0.2)
    fig.tight_layout()
    fig.savefig(out, dpi=150)
    print(f'wrote {out}')


def print_summary(bag: str, map_topic: str) -> None:
    """One block of numbers per run, for comparing runs that differ in one setting.

    Coverage is reported as an AREA in square metres, not as a percentage of the
    map. The percentage is the tempting number and it is not comparable between
    runs: a SLAM occupancy grid is sized to whatever the pose graph covers, so a
    run whose map drifted outward has a larger denominator and scores a lower
    percentage for having explored more. Across the five runs behind this
    comment, the grids ranged from 8832 to 18592 cells for the same 4 by 4 m
    maze. Cells times resolution squared is the same unit in every run.

    Free area, rather than known area, is the one to compare: known counts
    occupied cells too, and a run that spent longer staring at walls accumulates
    those without exploring anything.

    The two times are what the lab's optional competition scores: time to 90
    percent of the free area the run eventually reached, and time to reach it.

    Distance driven comes from the tf track, so it is distance in the map frame
    rather than integrated wheel odometry. Where the two disagree the difference
    is SLAM correcting drift, and the corrected number is the one that says how
    far the robot actually went.
    """
    times: list[float] = []
    known_area: list[float] = []
    free_area: list[float] = []
    cells = 0
    for seconds, msg in read_topic(bag, map_topic):
        grid = np.asarray(msg.data, dtype=np.int16)
        cell_m2 = msg.info.resolution ** 2
        times.append(seconds)
        known_area.append(float(np.count_nonzero(grid >= 0)) * cell_m2)
        free_area.append(float(np.count_nonzero((grid >= 0) & (grid <= 50))) * cell_m2)
        cells = grid.size
    if not times:
        raise SystemExit(f'no {map_topic} messages in {bag}')

    final = free_area[-1]
    t90 = next((t for t, a in zip(times, free_area) if a >= 0.9 * final), float('nan'))
    t100 = next((t for t, a in zip(times, free_area) if a >= final), float('nan'))

    segments = robot_track(bag, None)
    driven = track_length(segments)
    paths = sum(1 for _ in read_topic(bag, '/path'))

    print(f'{pathlib.Path(bag).name}')
    print(f'  duration            {times[-1]:8.1f} s')
    print(f'  free area explored  {final:8.2f} m2')
    print(f'  known area          {known_area[-1]:8.2f} m2')
    print(f'  time to 90% of it   {t90:8.1f} s')
    print(f'  time to all of it   {t100:8.1f} s')
    print(f'  distance driven     {driven:8.2f} m   (tf, map frame, '
          f'{len(segments)} segments)')
    print(f'  paths published     {paths:8d}')
    print(f'  final grid          {cells:8d} cells')


def plot_coverage(bag: str, map_topic: str, out: str, title: str | None) -> None:
    """Known cells over time, as a percentage of the final map's cell count."""
    times, percent = [], []
    areas: list[float] = []
    for seconds, msg in read_topic(bag, map_topic):
        grid = np.asarray(msg.data, dtype=np.int16)
        times.append(seconds)
        areas.append(float(np.count_nonzero((grid >= 0) & (grid <= 50)))
                     * msg.info.resolution ** 2)
    if not times:
        raise SystemExit(f'no {map_topic} messages in {bag}')

    # Against the final area, not against the grid's cell count: the grid is
    # sized to whatever the pose graph covers, so it is not the same denominator
    # in two different runs.
    percent = [100.0 * a / areas[-1] for a in areas]
    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.plot(times, percent, color='#1f77b4', linewidth=1.8)

    # The two numbers the optional competition scores: A, time to full
    # exploration, and B, time to 90 percent. Full is taken as the final value
    # rather than 100 percent, because a SLAM map is padded with cells outside
    # the maze that never become known and never should.
    final = percent[-1]
    for fraction, colour, label in ((0.9, '#d62728', '90% of final'),
                                    (1.0, '#2ca02c', 'final')):
        target = final * fraction
        reached = next((t for t, p in zip(times, percent) if p >= target), None)
        if reached is not None:
            ax.axvline(reached, color=colour, linestyle='--', linewidth=1.2,
                       label=f'{label}: {reached:.0f} s')
    ax.set_xlabel('time [s]')
    ax.set_ylabel('free area explored [% of final]')
    ax.set_title(title or f'Coverage, {pathlib.Path(bag).name}')
    ax.legend(fontsize=9)
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out, dpi=150)
    print(f'wrote {out}  (final free area {areas[-1]:.2f} m2)')


if __name__ == '__main__':
    main()
