# r7021e_bringup

Launch files, parameter files and the RViz configuration for R7021E 2026. No nodes of
its own: it starts the ones in `r7021e_control` and decides which of them run together.

## Where the numbers live

`config/` in this package is a symlink to `config/` at the repository root. That is
where the parameter files actually are, so `config/robot.yaml` is one file rather than
one per package. Editing it edits what every node reads.

`robot.yaml` is loaded first for every node, then that node's own file. Later files win
on a repeated key, so a node file could in principle override a physical limit. None of
them do, and none of them should.

## Running it

One launch file covers all four tasks. `sim:=true` is what actually includes the
Gazebo world -- it defaults to false, so leaving it off starts the ROS nodes with no
simulator behind them and nothing will move. `rviz:=true` is optional but useful for
watching any of these. The mode argument decides which node drives `/cmd_vel`,
because `controller_node` and `wall_follower_node` both publish there and two
publishers on one topic do not error, they interleave.

Task 1, position tracking from terminal setpoints:

```bash
ros2 launch r7021e_bringup lab1.launch.py sim:=true rviz:=true
```

then, in another terminal with the same `ROS_DOMAIN_ID`:

```bash
ros2 topic pub --once /new_position geometry_msgs/Pose "{position: {x: 1.0, y: 0.5}}"
```

Tasks 2 and 3, the figure of eight with the closest wall point recorded alongside it:

```bash
ros2 launch r7021e_bringup lab1.launch.py sim:=true trajectory:=true rviz:=true
```

Task 4, wall following until the loop closes:

```bash
ros2 launch r7021e_bringup lab1.launch.py sim:=true mode:=wall_following rviz:=true
```

On the robot rather than in simulation, drop `sim:=true` (there's no Gazebo world to
include), add the domain, and turn simulated time off:

```bash
ros2 launch r7021e_bringup lab1.launch.py use_sim_time:=false domain_id:=34
```

## Lab day checklist

**Step one, before debugging anything else, with a simulation or the robot running:**

```bash
ros2 topic info /cmd_vel -v
```

It prints the message type on both ends. A `Twist` publisher against a `TwistStamped`
subscriber is not an error, it is two endpoints that never connect: nothing logs, the
robot sits still, and everything looks correct. Thirty seconds against a plausible hour
of a 180 minute session. See `wiki/twiststamped-header-trap.md`.

Then, in order:

1. `echo $ROS_DOMAIN_ID` in every terminal. The launch file sets it for the nodes it
   starts, not for the shell you typed it in, so `ros2 topic list` in a second terminal
   needs its own `export ROS_DOMAIN_ID=34`.
2. `ros2 param get /controller_node robot.max_linear_velocity`. If it is not 0.22, the
   parameter files did not load and every node is running on its in-code defaults.
3. Read `max_linear_velocity` off the robot itself before quoting 0.22 in the report.
   Firmware revisions have shipped different values.
4. **Run this before every simulation launch, no exceptions -- not only when
   something looks wrong.** Ctrl-C stops `ros2 launch` and its ROS nodes cleanly,
   but the `gz sim` server and GUI it started do not die with it. Confirmed, not
   occasional: it happened on every run tested, every time.

   ```bash
   ps aux | grep -E "gzserver|gzclient|gz sim|ros2 launch|rviz2|robot_state_publisher" | grep -v grep
   ```

   If anything shows up, `kill -9` the PIDs it lists, then rerun the command and
   confirm the output is empty, before launching anything new. A leftover `gz sim`
   process keeps running on its own clock and keeps publishing `/tf`, `/odom` and
   `/scan`. Several of these at once, each at a different simulated time, is what
   produces `TF_OLD_DATA` warnings in RViz and a robot that appears to jump between
   positions, with every node logging as if nothing is wrong.

## Recording

Decision D6 in PRD 01. This topic list is what the graded video needs, and a bag
missing one of them cannot produce it, which means the run has to be done again.

```bash
ros2 bag record -o bags/$(date +%Y-%m-%d)-lab1-eight /odom /cmd_vel /scan /new_position /tf /tf_static
```

Check the recorded topic list before leaving the lab, not afterwards:

```bash
ros2 bag info bags/<the bag>
```

`/closest_wall_point` is not in D6's list and does not need to be: it is computed from
`/scan`, which is recorded, so the task 3 plot can be regenerated from the bag by
running `scan_monitor_node` against a replay. Adding it costs nothing and saves that
step, which is a judgement call rather than a requirement.

## Known gap: the video cannot show the goal yet

The assignment requires the video to show the current goal at every instant. It cannot
today, and this is written down rather than discovered in week 12.

`/new_position` is a bare `geometry_msgs/Pose`. It has no header, so it has no frame
and no timestamp, and RViz has no display that can draw one. The topic name and type
are fixed by the course, so the message cannot simply be changed to `PoseStamped`.

Three ways out, none of them chosen yet, because the choice belongs to Dominic:

1. A small visualisation node that subscribes `/new_position` and republishes it as a
   `PoseStamped` or a `Marker` for RViz. It is the writer rule "visualisation is its
   own node" applied honestly, and it makes five nodes where decision D3 says four.
2. Draw the goal in post, from the bag, with a plotting script rather than in RViz.
   The assignment says the video is generated in RViz, so this probably does not
   satisfy it.
3. Ask whether a goal marker is required in the video or whether the trajectory plot
   in the report covers it.

Everything else the video needs is in `rviz/lab1.rviz` and works: current position, the
path so far as an odometry trail, and the laser scan.
