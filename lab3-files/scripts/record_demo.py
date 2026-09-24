#!/usr/bin/env python3
"""Record a rosbag and a screen-capture video of RViz at the same time.

One-time setup on a new machine:

    sudo apt install x11-utils gstreamer1.0-tools gstreamer1.0-plugins-base \
                     gstreamer1.0-plugins-good python3-xlib

    # If python3-xlib is unavailable, this works without sudo:
    #   pip install --user --break-system-packages python-xlib

    x11-utils                    xwininfo, used to find the RViz window
    gstreamer1.0-tools           gst-launch-1.0
    gstreamer1.0-plugins-base    videoconvert
    gstreamer1.0-plugins-good    ximagesrc, vp8enc, webmmux
    python3-xlib                 raising the window before capture starts

Check the plugins are really there before a lab session, not during one:

    gst-inspect-1.0 ximagesrc && gst-inspect-1.0 vp8enc && gst-inspect-1.0 webmmux

Usage: start your lab's launch file yourself (with rviz:=true), then run this in
another terminal:

    source /opt/ros/jazzy/setup.bash
    source <workspace>/install/setup.bash
    python3 scripts/record_demo.py bags/task2-obstacle

It finds the RViz window, raises it, starts `ros2 bag record` and a GStreamer screen
capture of that window together, and stops both cleanly on Ctrl-C. Run your task
(publish goals, etc.) in a third terminal while this runs.

Ctrl-C once, then wait. With a Gazebo simulation running, `ros2 bag record` has taken
between 20 and 101 seconds to finish writing, measured, for bags of only a few
megabytes. That is the recorder, not a lost signal: a direct SIGINT to its own PID
takes just as long. This script waits for it and prints both file sizes when it is
genuinely finished. A second Ctrl-C, or closing the terminal, truncates the bag, and
`ros2 bag info` will still call the truncated file a valid bag.

Needs a real X display. Under Wayland, run RViz through XWayland, or ximagesrc has no
window id to capture.

Why GStreamer's ximagesrc and not ffmpeg's x11grab: x11grab captures a fixed screen
region from the (possibly composited) root window, which on this desktop returns a
black frame for a window under a compositing window manager. ximagesrc captures by
window ID directly and does not have that problem. Confirmed the hard way.

Why VP8/WebM and not H.264: the H.264 encoder lives in gstreamer1.0-plugins-ugly and
carries a patent-licensing caveat. VP8 (`vp8enc`) is in plugins-good, plays in any
modern browser and video player, and is all a report or a submission needs.
"""

import argparse
import pathlib
import re
import signal
import subprocess
import sys
import time

# Every topic a Lab 1 or Lab 2 run can produce. ros2 bag record warns about a topic that
# nothing is publishing and records the rest, so one list serves both labs rather than a
# flag that has to be remembered correctly under time pressure on a lab day.
#
# /closest_wall_point exists only in Lab 1 (scan_monitor_node); /mpc_prediction,
# /mpc_obstacles and /reference_path only in Lab 2. Expect warnings for whichever half
# is not running.
BAG_TOPICS = [
    # Labs 1, 2 and 3 share one list. `ros2 bag record` warns about whichever
    # topics are not running and records the rest, so one list serving three labs
    # costs a few warnings and saves three lists drifting apart.
    '/odom', '/cmd_vel', '/scan', '/tf', '/tf_static',
    # Labs 1 and 2
    '/new_position', '/goal_marker', '/closest_wall_point',
    '/mpc_prediction', '/mpc_obstacles', '/reference_path',
    # Lab 3. /map and /frontiers are full occupancy grids at 1 Hz, so a maze run
    # produces a much larger bag than either earlier lab, and the recorder's
    # already slow shutdown gets slower with it. Budget for that in the session,
    # and run `ros2 bag info` immediately afterwards to check duration and counts.
    '/map', '/frontiers', '/path',
    '/rrt_tree', '/frontier_goals', '/exploration_status',
]


def find_rviz_window_id() -> int:
    """Return the top-level (decorated) RViz window's X id, via xwininfo."""
    out = subprocess.run(
        ['xwininfo', '-root', '-tree'], capture_output=True, text=True, check=True
    ).stdout
    for line in out.splitlines():
        if 'RViz' in line and 'mutter-x11-frames' in line:
            match = re.search(r'(0x[0-9a-f]+)', line)
            if match:
                return int(match.group(1), 16)
    raise SystemExit('No RViz window found. Is it running with rviz:=true?')


def raise_window(win_id: int) -> None:
    """Bring the window to the front so screen capture doesn't grab whatever
    happens to be on top of it. Needs python-xlib; see the module docstring."""
    from Xlib import X, display
    from Xlib.protocol import event

    d = display.Display()
    root = d.screen().root
    win = d.create_resource_object('window', win_id)
    net_active = d.intern_atom('_NET_ACTIVE_WINDOW')
    ev = event.ClientMessage(
        window=win, client_type=net_active, data=(32, [1, X.CurrentTime, 0, 0, 0]))
    root.send_event(ev, event_mask=X.SubstructureRedirectMask | X.SubstructureNotifyMask)
    win.configure(stack_mode=X.Above)
    win.set_input_focus(X.RevertToParent, X.CurrentTime)
    d.sync()
    time.sleep(1)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                      formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('out_basename',
                         help='bag goes to <out_basename>/, video to <out_basename>.webm')
    args = parser.parse_args()

    win_id = find_rviz_window_id()
    raise_window(win_id)

    bag_proc = subprocess.Popen(
        ['ros2', 'bag', 'record', '-o', args.out_basename] + BAG_TOPICS)
    video_proc = subprocess.Popen([
        'gst-launch-1.0', '-e', 'ximagesrc', f'xid={win_id}', '!', 'videoconvert',
        '!', 'vp8enc', '!', 'webmmux', '!', 'filesink',
        f'location={args.out_basename}.webm',
    ])

    print(f'Recording bag to {args.out_basename}/ and video to '
          f'{args.out_basename}.webm -- run your task now. Ctrl-C here to stop both.',
          flush=True)

    def finish(name, proc, timeout):
        """Wait for one recorder to shut down, escalating only if it will not.

        Neither of these is a normal process to kill. `ros2 bag record` finishes
        writing after SIGINT, and with a Gazebo simulation still running that takes a
        surprisingly long time: measured at 20 to 101 seconds for Lab 2 bags of only a
        few megabytes. It is not a signal that failed to arrive. A direct SIGINT
        straight to the recorder's own PID took the same 101 seconds.

        So this waits, and waits again, and only then escalates. Killing the recorder
        early truncates the bag, and the truncation is silent: `ros2 bag info` still
        reports a readable bag, just a shorter one than the run you recorded.
        """
        started = time.time()
        try:
            proc.wait(timeout=timeout)
            print(f'  {name} stopped after {time.time() - started:.1f}s', flush=True)
            return True
        except subprocess.TimeoutExpired:
            print(f'  {name} still shutting down after {timeout}s, waiting longer...',
                  flush=True)
        try:
            proc.wait(timeout=timeout)
            print(f'  {name} stopped after {time.time() - started:.1f}s', flush=True)
            return True
        except subprocess.TimeoutExpired:
            print(f'  {name} did not exit; terminating. The output may be incomplete.',
                  flush=True)
            proc.terminate()
            try:
                proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                proc.kill()
            return False

    def stop(*_):
        # A second Ctrl-C while the bag is flushing would re-enter this handler and
        # truncate the very write it is waiting for.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        print('\nStopping. The bag can take up to two minutes to finish writing while '
              'a simulator\nis still running. Do not press Ctrl-C again and do not close '
              'this terminal:\nboth truncate the bag silently. Sizes are printed when it '
              'is done.', flush=True)

        bag_proc.send_signal(signal.SIGINT)
        video_proc.send_signal(signal.SIGINT)

        # The video pipeline finalises the WebM container on EOS and is quick about
        # it. The bag is the slow one, and it is the one worth waiting for.
        video_ok = finish('video', video_proc, 30)
        bag_ok = finish('bag', bag_proc, 120)

        bag_path = pathlib.Path(args.out_basename)
        video_path = pathlib.Path(f'{args.out_basename}.webm')
        bag_bytes = sum(f.stat().st_size for f in bag_path.glob('*')) if bag_path.is_dir() else 0
        video_bytes = video_path.stat().st_size if video_path.is_file() else 0

        print()
        print(f'  bag   {bag_path}/  {bag_bytes / 1e6:.1f} MB')
        print(f'  video {video_path}  {video_bytes / 1e6:.1f} MB')
        if not (bag_ok and video_ok) or bag_bytes == 0 or video_bytes == 0:
            print('  WARNING: one of the recorders did not finish cleanly. Check both '
                  'before relying on this run.')
        print()
        print(f'Verify with:  ros2 bag info {args.out_basename}')
        print(f'              gst-discoverer-1.0 {args.out_basename}.webm', flush=True)
        sys.exit(0 if (bag_ok and video_ok and bag_bytes and video_bytes) else 1)

    signal.signal(signal.SIGINT, stop)
    signal.pause()


if __name__ == '__main__':
    main()
