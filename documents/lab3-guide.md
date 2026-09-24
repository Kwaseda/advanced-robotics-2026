# Lab 3 guide: autonomous exploration with sampling-based planning

A TurtleBot3 Burger explores an unknown maze on its own. The course supplies SLAM, a
frontier extractor and a path follower; this package supplies the RRT* planner, the
exploration gain that decides where to go next, and the loop that joins them.

Everything here runs in simulation first. The hardware is only available during the
session, so a bug found at the bench is a bug that costs the session.

## What Lab 3 is

Five tasks from `Lab_Instructions_Exploration.pdf`:

1. **System baseline.** A node that sends a `nav_msgs/Path` to the path follower and
   sends a new one when the robot reaches the end.
2. **Path planning.** An RRT or RRT* written from scratch, planning on the live SLAM map,
   rooted at the robot's current position. Computation time is explicitly not a priority.
3. **Collision avoidance (optional).** The RRT treats the robot as a point, so something
   has to keep it off the walls.
4. **Exploration gain.** Given frontier points, decide where to go next. The graded
   centre of the lab.
5. **The complete system.** Choose a target, plan to it, drive, repeat until done.

Tasks 1 and 5 are the same node here. Task 1 is the loop with the goal written down and
Task 5 is the loop with the goal chosen by Task 4, so building Task 1 as a throwaway
script would have meant building it twice.

Task 3 is answered by map inflation: every occupied cell is stamped with a disc of the
robot's radius before the planner sees the map. The planner keeps treating the robot as a
point, exactly as the instructions allow, and the margin is enforced by geometry rather
than by a behaviour that can be tuned away at run time.

## One-time setup on a new machine

### The course package

This lab reuses two nodes from the course's own package, unmodified. Download
`r7021e_lab_package.zip` from Canvas, extract `r7021e_exploration` into your workspace's
`src/`, and build it alongside these packages:

```bash
cd lab3-files/ros2_ws/src
# extract r7021e_exploration here
```

Without it, `lab3.launch.py` will fail to find `frontier_detector_node` and
`path_follower_node`.

### System packages

```bash
sudo apt install ros-jazzy-slam-toolbox
```

And, if you want to record runs (screen capture plus bag in one pass):

```bash
sudo apt install x11-utils gstreamer1.0-tools gstreamer1.0-plugins-base \
                 gstreamer1.0-plugins-good gstreamer1.0-plugins-base-apps python3-xlib
```

### Simulation

Gazebo Sim and the TurtleBot3 models, in whatever workspace you keep them:

```bash
export TURTLEBOT3_MODEL=burger
source ~/turtlebot3_ws/install/setup.bash
```

`turtlebot3_gazebo` is resolved at launch time rather than at parse time, so a machine
with no simulator can still launch everything except the world.

## Build

```bash
cd lab3-files/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

Three packages should build: `r7021e_rrt`, `r7021e_rrt_bringup`, and the course's
`r7021e_exploration`.

## Running it

In simulation, on the small maze, with RViz:

```bash
ros2 launch r7021e_rrt_bringup lab3.launch.py sim:=true rviz:=true
```

On the full maze:

```bash
ros2 launch r7021e_rrt_bringup lab3.launch.py sim:=true rviz:=true world:=lab3_maze
```

On the robot, with `turtlebot3_bringup robot.launch.py` already running over ssh:

```bash
ros2 launch r7021e_rrt_bringup lab3.launch.py use_sim_time:=false domain_id:=34 rviz:=true
```

### Arguments

| Argument | Default | What it does |
|---|---|---|
| `sim` | `false` | bring up Gazebo and spawn the Burger |
| `world` | `lab3_maze_small` | which maze. `lab3_maze_small` is 4.0 m, `lab3_maze` is 7.2 m |
| `gui` | `true` | Gazebo's own GUI. `false` for a headless sweep |
| `rviz` | `false` | RViz2 with the saved Lab 3 configuration |
| `slam` | `true` | start slam_toolbox |
| `domain_id` | `34` | `ROS_DOMAIN_ID`, which the lab sets to `3<robot number>` |
| `use_sim_time` | `true` | `true` in Gazebo, `false` on the robot |
| `gain_mode` | `reduced_range` | which information gain. `cluster_size` is the alternative |
| `inflation` | `0.105` | obstacle inflation radius in metres |
| `follower_max_w` | `1.0` | the follower's turn rate limit in rad/s. Lowering it has been measured and made the map worse, see section 15 of lab3-report-notes.md |

`gain_mode` and `inflation` are arguments because the report compares runs that differ in
exactly one of them. Everything else lives in `config/lab3.yaml`.

**The launch file sets `ROS_DOMAIN_ID` for the nodes it starts.** A `ros2 topic echo` in
another terminal sees nothing until that terminal exports the same value. This is worth
knowing before you spend half an hour concluding the simulator is broken.

## Topics

| Topic | Type | Who |
|---|---|---|
| `/map` | `nav_msgs/OccupancyGrid` | slam_toolbox, in |
| `/frontiers` | `nav_msgs/OccupancyGrid` | frontier_detector_node, in |
| `/path` | `nav_msgs/Path` | navigation_node, out, to the follower |
| `/cmd_vel` | `geometry_msgs/TwistStamped` | path_follower_node, out |
| `/rrt_tree` | `visualization_msgs/Marker` | navigation_node, the winning tree |
| `/frontier_goals` | `visualization_msgs/MarkerArray` | one sphere per candidate |
| `/exploration_status` | `visualization_msgs/Marker` | cycle, cluster count, current H |

Note the topic name: the frontier detector publishes on **`frontiers`**, plural. The
course's own `navigation_node` template declares `frontier`, singular, and its launch file
has no remapping, so as shipped that subscription never receives anything. If you start
from the template rather than from this package, fix that first.

Note the QoS too. `/map` is transient-local and `/frontiers` is volatile, so they need
different subscription profiles. A mismatched profile receives nothing at all, silently,
and both sides log an "incompatible QoS" warning that is easy to scroll past.

## Parameters worth knowing

All in `config/lab3.yaml`, each with the reason for its value next to it. Read them back
off the running node rather than trusting the file:

```bash
ros2 param get /navigation_node inflation_radius
ros2 param get /navigation_node gain.radius
```

| Parameter | Value | Why |
|---|---|---|
| `inflation_radius` | 0.105 m | The Burger's circumscribing radius is 0.113 m. At 0.05 m cells this stamps 3 cells, so the enforced collar is 0.15 m |
| `gain.mode` | `reduced_range` | Frontier cells within `gain.radius` of the candidate, across the whole grid |
| `gain.radius` | 0.75 m | A fifth of the Burger's real 3.5 m LiDAR. The instructions ask for a greatly reduced range, and without it every frontier scores alike |
| `gain.weight_per_cell` | 0.10 m/cell | Converts a cell count into metres so `H = sum(d) - w I` can be evaluated. This is the greedy versus complete knob |
| `rrt.step_size` | 0.30 m | Six cells. Short enough not to jump a wall, long enough to cross a maze |
| `rrt.unknown_lookahead` | 0.50 m | How far a branch may run through unmapped space before it must touch known-free ground again |
| `commit.switch_margin` | 0.5 | A rival must beat the committed goal by this much in H to take it. Stops frontier oscillation |
| `goal.exhaust_radius` | 0.30 m | A goal already reached, or stalled on, is never offered again |
| `path_waypoint_spacing` | 0.10 m | Half the follower's look-ahead, so it cannot cut corners off the checked path |
| `max_empty_cycles` | 10 | Consecutive cycles with no cluster before the run ends |

### Two parameters that exist because of a specific failure

`goal.exhaust_radius`. Without it the robot reaches a frontier it cannot clear from where
it stands, then chooses the same goal again, and again. Nothing in `H = sum(d) - w I`
knows where the robot has been, so the nearest surviving frontier keeps winning forever.

`path_waypoint_spacing`. The follower drops a waypoint as soon as it is within its 0.20 m
look-ahead and steers at the next one, so across a corner it aims through the corner and
drives the chord rather than the segments the planner checked. In a 0.7 m corridor that is
enough to wedge the robot against a wall, and a wedged robot spins its wheels, runs
odometry away from the truth, and takes SLAM's scan matching with it.

## Recording

`scripts/record_demo.py` records a bag and a screen capture of RViz in one pass. Its topic
list already covers Labs 1, 2 and 3; `ros2 bag record` warns about whichever topics are
not running and records the rest.

```bash
# 1. the launch file, with RViz, started by you
ros2 launch r7021e_rrt_bringup lab3.launch.py sim:=true rviz:=true

# 2. the recorder, once RViz is actually on screen
python3 scripts/record_demo.py --out-basename lab3-run
```

### Stopping it, which is the part that needs care

`ros2 bag record` keeps writing after Ctrl-C, for anywhere from twenty seconds to several
minutes, for bags of a few megabytes. That is not a bug in the script and not a signal
that failed to arrive. Lab 3's maze runs record two full occupancy grids at 1 Hz, so its
bags are much larger than Labs 1 and 2, and the wait is longer. Budget for it.

A second Ctrl-C truncates the bag silently, and `ros2 bag info` still calls a truncated
bag valid.

If the recorder is killed rather than interrupted, it never writes its `metadata.yaml` and
rosbag2 then refuses to open the bag at all. Regenerate it:

```bash
ros2 bag reindex -s mcap bags/lab3-run
```

Do not pass `--use-sim-time` to `ros2 bag record`. It waits for `/clock` before starting
and, in these runs, never started: the result is a bag file of the right size containing
no messages.

### Always verify before you leave

```bash
ros2 bag info bags/lab3-run
```

Check the duration and the message counts against what you expect. Then plot it:

```bash
python3 scripts/plot_exploration.py bags/lab3-run -o lab3-run.png
python3 scripts/plot_exploration.py bags/lab3-run --coverage -o lab3-coverage.png
```

The first draws the map, the frontier cells, the RRT* tree, the scored candidate goals,
the planned path and the driven trajectory. The second plots coverage against time and
marks the time to 90 percent and to final coverage, which are the two numbers the optional
competition scores.

The trajectory is composed from `/tf`, not taken from `/odom`. `/odom` is expressed in the
odom frame, which drifts, and drawing it over a map drawn in the map frame plots the drift
as though it were motion: it can show the robot leaving a sealed maze.

## Lab day checklist

1. `export ROS_DOMAIN_ID=3<robot number>` in **every** terminal, and pass
   `domain_id:=3<robot number>` to the launch file.
2. `ssh turtle@192.168.50.<number>0`, password `turtle`, then
   `ros2 launch turtlebot3_bringup robot.launch.py`. Leave it running.
3. `ros2 topic info /cmd_vel -v` from your laptop. If the robot's subscription is not
   listed, nothing you do to the planner will help.
4. `ros2 topic hz /scan` and `ros2 topic hz /map`. No map means no frontiers means no
   exploration, and it will look like a planner bug.
5. Launch with `use_sim_time:=false`. On the robot the clock is real.
6. `ros2 param get /navigation_node inflation_radius` before the graded run. An empty
   parameter directory builds clean and launches clean.
7. Consider `publish_markers:=false` on the robot. A 1500-node tree is 1499 line segments
   republished every cycle over a shared lab router.
8. After every run, check for stray processes. `gz sim`'s server and GUI do not die on
   Ctrl-C:

```bash
ps aux | grep -E "[g]z sim|[g]zserver|[g]zclient|[r]viz2|[s]lam_toolbox" | grep -v grep
```

Use the bracket. `pkill -f <pattern>` matches the shell running it when the pattern
appears in that shell's own command line, which kills the terminal doing the cleanup.

## If something goes wrong

**The robot does not move at all.** Check `/path` is being published
(`ros2 topic hz /path`) and that the follower is subscribed. Then check tf: the follower
looks up `map` to `base_link` with no timeout and no error handling, so if tf is not ready
when a path arrives it raises inside its timer callback and the node dies. Nothing in
these packages publishes a path until a tf lookup has succeeded once, which avoids it in
practice.

**The robot stops early and says exploration is complete.** Expect this: every run
measured so far parked with frontiers still on the map. Look at `/frontier_goals` in RViz
before assuming a bug. If the last candidates are red, the node found them, scored them and
could not plan to any of them, which is the common case and is covered in section 12 of
lab3-report-notes.md. If instead frontier cells are visible on `/frontiers` but the node reports
zero clusters, they are below `cluster.min_size` or fragmented. Clustering here is
8-connected precisely because the boundary of what a rotating LiDAR has seen is a curve,
and a curve on a grid is a staircase: under 4-connectivity a diagonal frontier is N
clusters of one cell, all below the noise floor, all discarded.

**The robot drives back and forth over a few centimetres.** That is the arrive-reselect
loop, and it means `goal.exhaust_radius` is too small for the situation. Raise it.

**The map comes out smeared, with doubled walls.** Something is corrupting SLAM's input.
Check for two publishers on `/clock` in simulation, which makes time appear to step
backwards; tf reports it as "Detected jump back in time". Also check whether the robot is
wedged anywhere, since slipping wheels move odometry without moving the robot, and
slam_toolbox's correlation search space is only 0.5 m wide.

**Nothing responds to `ros2 topic echo`, or `ros2 node list` is empty.**
`ROS_DOMAIN_ID`. Every time. Note that the launch file sets it for every node it starts,
from its own `domain_id` argument, so exporting a different one in your shell before
launching does not move the nodes: it moves only your command line tools, away from them.
Pass `domain_id:=<n>` to the launch instead, and use the same number in the shell you run
`ros2 param get` from.
