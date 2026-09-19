# Lab 1 complete guide: ROS 2 position control and wall following

## Overview

This guide covers the R7021E Lab 1 implementation: closed-loop position control and
wall following on a TurtleBot3 Burger. The position controller uses Near Identity
Diffeomorphism (NID) to work around the fact that the robot can't move sideways,
which is what lets a plain proportional controller drive it toward a goal.

## Project structure

Two ROS 2 packages.

`r7021e_control` has the control algorithms and the nodes that run them:
- `controller_node.py` -- closed-loop position tracking using NID (Tasks 1 and 2)
- `trajectory_node.py` -- generates the figure-eight trajectory (Task 2)
- `scan_monitor_node.py` -- finds the closest wall point from the lidar (Task 3)
- `wall_follower_node.py` -- wall following (Task 4)
- `goal_marker_node.py` -- republishes the current goal as an RViz marker
- Plain-Python support modules with no ROS imports, so they're unit-testable on
  their own: `geometry.py`, `nid_controller.py`, `trajectories.py`,
  `wall_following.py`, `scan_utils.py`

`r7021e_bringup` has the launch file, the RViz configuration, and each node's own
parameter files under its `config/`.

## Configuration files

### robot.yaml
Physical limits of the TurtleBot3 Burger:
- max_linear_velocity: 0.22 m/s
- max_angular_velocity: 2.84 rad/s
- Wheel radius: 0.033 m, wheel separation: 0.160 m
- Footprint: 0.178 m by 0.138 m by 0.192 m
- Lidar range: 0.12 m to 3.5 m

### controller.yaml
Position controller parameters:
- NID offset L = 0.10 m
- Proportional gain k_p = 0.8
- Control period: 0.05 s (20 Hz)
- Goal tolerance: 0.05 m

### trajectory.yaml
Figure-eight parameters:
- Width: 1.0 m, height: 0.5 m
- Lap period: 50.0 s
- Number of laps: 2
- Start delay: 3.0 s

### wall_follower.yaml
Wall-following parameters:
- Follow distance: 0.5 m
- Speed: 0.15 m/s
- Loop closure tolerance: 0.2 m
- Follow side: right

## Building the system

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Testing commands for simulation

### Task 1: position tracking

Start the controller in simulation:
```bash
ros2 launch r7021e_bringup lab1.launch.py sim:=true rviz:=true
```

In a separate terminal, send position commands:
```bash
# Move 1 meter forward
ros2 topic pub --once /new_position geometry_msgs/Pose "{position: {x: 1.0, y: 0.0}}"

# Move 1 meter to the left
ros2 topic pub --once /new_position geometry_msgs/Pose "{position: {x: 0.0, y: 1.0}}"

# Move to a diagonal position
ros2 topic pub --once /new_position geometry_msgs/Pose "{position: {x: 0.5, y: 0.5}}"
```

Watch RViz to confirm the robot reaches each goal and stops within tolerance.

### Tasks 2 and 3: figure-eight with wall-point detection

Start the figure-eight trajectory:
```bash
ros2 launch r7021e_bringup lab1.launch.py sim:=true trajectory:=true rviz:=true
```

The robot waits 3 seconds, then traces the figure eight for 2 laps. Check wall-point
detection with:
```bash
ros2 topic echo /closest_wall_point
```

### Task 4: wall following

```bash
ros2 launch r7021e_bringup lab1.launch.py sim:=true mode:=wall_following rviz:=true
```

The robot should find the nearest wall, hold roughly 0.5 m off it, turn at corners,
and close the loop.

## Recording rosbags

### Task 1
```bash
ros2 bag record -o bags/task1-position /odom /cmd_vel /scan /new_position /tf /tf_static
```

### Tasks 2 and 3
```bash
ros2 bag record -o bags/task2-eight /odom /cmd_vel /scan /new_position /tf /tf_static
```

### Task 4
```bash
ros2 bag record -o bags/task4-wall /odom /cmd_vel /scan /tf /tf_static
```

### Verifying a bag
```bash
ros2 bag info bags/<the bag>
```

Make sure all the required topics are present: `/odom`, `/cmd_vel`, `/scan`,
`/new_position`, `/tf`, `/tf_static`.

## Recording the video and trajectory plots for the report

Two scripts in `scripts/` do this. One-time setup on a new machine (no sudo needed):

```bash
pip install --user python-xlib
```

**Video** -- records the bag and a screen capture of RViz together, in one pass, so
you don't need to replay the bag separately just to make the video:

```bash
# terminal 1: start the sim as usual, with rviz:=true
ros2 launch r7021e_bringup lab1.launch.py sim:=true rviz:=true

# terminal 2, once RViz is up
source /opt/ros/jazzy/setup.bash && source install/setup.bash
python3 scripts/record_demo.py bags/task1-position

# terminal 3: run the task (publish goals, etc.), then Ctrl-C terminal 2 when done
```

This produces `bags/task1-position/` (the bag) and `bags/task1-position.webm` (the
video). WebM plays in any modern browser or video player; convert it if your
submission requires a specific container.

Why a screen capture and not `ffmpeg -f x11grab`: x11grab captures a fixed screen
region from the (possibly composited) root window, which returns a black frame on a
compositing desktop. The script captures the RViz window by its X window ID instead,
which doesn't have that problem.

**Trajectory plots** -- the robot's actual path against the commanded/provided one,
from a recorded bag:

```bash
python3 scripts/plot_trajectory.py bags/task1-position -o task1_trajectory.png
python3 scripts/plot_trajectory.py bags/task2-eight -o task2_trajectory.png
```

Works for both a handful of discrete terminal setpoints (Task 1, plotted as goal
markers) and a continuously published path (Task 2's figure eight, plotted as a
line). It tells the two apart by counting distinct setpoints, not messages.

## Block diagram

```
ROS 2 System
├─────────────────────────────────────────────────────┐
│                                                     │
│  ┌──────────────┐         ┌──────────────┐          │
│  │   Sensors    │         │   Setpoints  │          │
│  │              │         │              │          │
│  │  /odom       │         │ /new_position│          │
│  │  (Odometry)  │         │  (Terminal   │          │
│  │              │         │   or traj)   │          │
│  └──────┬───────┘         └──────┬───────┘          │
│         │                        │                  │
│         ▼                        ▼                  │
│  ┌──────────────────────────────────────────┐       │
│  │         controller_node                  │       │
│  │                                          │       │
│  │  • Subscribe: /odom, /new_position       │       │
│  │  • Timer: 20 Hz control loop             │       │
│  │  • NID controller                        │       │
│  │  • Publish: /cmd_vel (TwistStamped)      │       │
│  └──────────────────┬───────────────────────┘       │
│                     │                               │
│                     ▼                               │
│  ┌──────────────────────────────────────────┐       │
│  │      TurtleBot3 (physical/sim)           │       │
│  │                                          │       │
│  │  • Subscribe: /cmd_vel                   │       │
│  │  • Publish: /odom, /scan                 │       │
│  │  • Wheel encoders + IMU                  │       │
│  │  • LDS-01/LDS-02 lidar                   │       │
│  └──────────────────────────────────────────┘       │
│                                                     │
│  Running alongside:                                 │
│  ┌──────────────────┐    ┌────────────────────┐     │
│  │ trajectory_node  │    │ scan_monitor_node  │     │
│  │                  │    │                    │     │
│  │ • Publish:       │    │ • Subscribe: /scan │     │
│  │   /new_position  │    │ • Publish:         │     │
│  │   (figure-8)     │    │   /closest_wall_   │     │
│  │                  │    │     point          │     │
│  └──────────────────┘    └────────────────────┘     │
│                                                     │
└─────────────────────────────────────────────────────┘
```

### Control loop detail

```
Goal position (x_goal, y_goal)
        │
        ▼
┌────────────────────┐
│  Error calculation │
│  Δx = x_goal - x_p │
│  Δy = y_goal - y_p │
└─────────┬──────────┘
          │
          ▼
┌────────────────────┐
│   NID transform    │
│                    │
│  1. Track offset   │
│     point P        │
│     (L ahead)      │
│                    │
│  2. Proportional   │
│     control on P:  │
│     u = k_p * e    │
│                    │
│  3. Invert to get  │
│     v, omega from  │
│     u_x, u_y       │
└─────────┬──────────┘
          │
          ▼
┌────────────────────┐
│  Velocity limits   │
│  Clamp to:         │
│  v_max = 0.22 m/s  │
│  ω_max = 2.84 rad/s│
└─────────┬──────────┘
          │
          ▼
    Publish /cmd_vel
```

## Dynamic model

### Unicycle model (kinematic)

State: `x, y` (position), `theta` (heading). Inputs: `v` (linear velocity), `omega`
(angular velocity).

```
x_dot = v * cos(theta)
y_dot = v * sin(theta)
theta_dot = omega
```

Non-holonomic constraint (no sideways velocity in the body frame):
```
x_dot * sin(theta) - y_dot * cos(theta) = 0
```

This is a kinematic model: no mass, no inertia, no wheel slip, so it assumes the
robot reaches a commanded velocity instantly. At 0.22 m/s on a robot this size,
that's a reasonable simplification for a first control lab.

### NID transform

Offset point P, a fixed distance `L` ahead of the robot's wheel axle:
```
x_p = x + L * cos(theta)
y_p = y + L * sin(theta)
```

Turning the robot sweeps P sideways at a rate proportional to `omega` and `L`, so P
behaves like a holonomic point that a plain proportional controller can drive.

Outer loop, proportional control on P's position error:
```
u_x = k_p * (x_goal - x_p)
u_y = k_p * (y_goal - y_p)
```

Inverting the transform gives the two robot inputs from the desired velocity of P:
```
v     =  u_x * cos(theta) + u_y * sin(theta)
omega = (-u_x * sin(theta) + u_y * cos(theta)) / L
```

`v` is the component of the demand along the current heading; `omega` is the
perpendicular component, divided by `L`. As `L` approaches zero the demanded `omega`
grows without bound, which is the algebra confirming the transform stops working
once P returns to the robot's own centre.

### Choosing L

Small `L` demands a large `omega` for a small lateral error, since the demand is
divided by `L`. At `L = 0.10 m` and a demand capped at 0.22 m/s, the largest `omega`
the transform can ever ask for is `0.22 / 0.10 = 2.2 rad/s`, comfortably inside the
robot's 2.84 rad/s limit. A larger `L` gives more headroom on `omega`, but moves P
further from the robot's actual centre, so convergence of P takes longer to show up
as convergence of the physical robot.

### Physical parameters

- Wheel radius: `r = 0.033 m`
- Wheel separation: `d = 0.160 m`
- Max linear velocity: `v_max = 0.22 m/s`
- Max angular velocity: `omega_max = 2.84 rad/s`
- NID offset: `L = 0.10 m`
- Control gain: `k_p = 0.8`

## Running on the physical robot

### Lab day setup

1. Connect to the robot:
```bash
ssh turtle@192.168.50.<number x 10>
```

2. Start robot bringup (on the robot):
```bash
ros2 launch turtlebot3_bringup robot.launch.py
```

3. Set the domain ID in each terminal:
```bash
export ROS_DOMAIN_ID=34  # for turtle4
```

4. Launch the controller (from your laptop):
```bash
ros2 launch r7021e_bringup lab1.launch.py use_sim_time:=false domain_id:=34
```

### Critical lab-day checklist

**Step 1: verify message types**
```bash
ros2 topic info /cmd_vel -v
```
Both ends should show `geometry_msgs/msg/TwistStamped`. A mismatch means the robot
won't move, and it won't log an error either.

**Step 2: check parameters**
```bash
ros2 param get /controller_node robot.max_linear_velocity
```
Should return 0.22. A different value means the parameter files didn't load.

**Step 3: verify the robot's actual limits**
Read `max_linear_velocity` off the robot's own `turtlebot3_node` before quoting 0.22
in the report. Firmware revisions have shipped different values.

**Step 4: run this before every simulation launch, no exceptions, not only when
something looks wrong.**

Ctrl-C stops `ros2 launch` and its ROS nodes cleanly, but the `gz sim` server and GUI
it started do not die with it. Confirmed, not occasional: it happens on every run.

```bash
ps aux | grep -E "gzserver|gzclient|gz sim|ros2 launch|rviz2|robot_state_publisher" | grep -v grep
```

If anything shows up, `kill -9` the PIDs it lists, then rerun the command and
confirm the output is empty before launching anything new. A leftover `gz sim`
process keeps running on its own clock and keeps publishing `/tf`, `/odom` and
`/scan`. Several of these at once, each at a different simulated time, is what
produces `TF_OLD_DATA` warnings in RViz and a robot that appears to jump between
positions, with every node logging as if nothing is wrong.

## Common issues and solutions

### Robot doesn't move
- Check the `/cmd_vel` message type match
- Verify `ROS_DOMAIN_ID` is set correctly
- Confirm the parameter files loaded

### Parameters not loading
- Check that `r7021e_bringup/config/` has the expected yaml files
- Confirm you sourced the workspace after building

### Lidar returns invalid ranges
- Check the range gating in `robot.yaml` (0.12 m to 3.5 m)
- Verify the lidar is actually running
- Check whether the robot is in open space with no walls in range

### Loop never closes
- The robot may start too far from a wall
- Check the `loop_min_distance` parameter (2.0 m)
- Verify the robot actually finds a wall to follow

### Simulation looks glitchy, robot jumps between positions
- Almost always stray `gz sim` processes from a previous run that was never cleanly
  killed. See checklist step 4 above.

## Key ROS topics

- `/odom` -- robot odometry (`nav_msgs/Odometry`)
- `/cmd_vel` -- velocity commands (`geometry_msgs/TwistStamped`)
- `/new_position` -- position setpoints (`geometry_msgs/Pose`)
- `/scan` -- lidar data (`sensor_msgs/LaserScan`)
- `/closest_wall_point` -- nearest wall point (`geometry_msgs/PointStamped`)

## Closed: the video can now show the goal

The assignment requires the video to show the current goal at every instant.
`/new_position` is a bare `geometry_msgs/Pose` with no header and no frame, so RViz
had no display that could draw one directly, and the topic name and type are fixed
by the course, so the message itself couldn't change.

`goal_marker_node` subscribes `/new_position` and republishes it as a
`visualization_msgs/Marker` (a green sphere) on `/goal_marker`, in the `odom` frame.
`lab1.launch.py` starts it unconditionally alongside the other nodes, and
`rviz/lab1.rviz` already has a Marker display on that topic, so it shows up with no
extra setup.

Everything the video needs is now in `rviz/lab1.rviz`: current position, the path so
far as an odometry trail, the laser scan, and the current goal.

## Quick verification

Before your lab session, verify:

```bash
# Build succeeds
cd ~/ros2_ws && colcon build --symlink-install

# Topics are correct
ros2 topic list | grep -E "(odom|cmd_vel|scan|new_position)"

# Message types match
ros2 topic info /cmd_vel -v

# Parameters load correctly
ros2 param get /controller_node robot.max_linear_velocity

# No stray simulation processes from a previous run
ps aux | grep -E "gzserver|gzclient|gz sim|ros2 launch|rviz2|robot_state_publisher" | grep -v grep
```

## Important notes

- Always check the `/cmd_vel` message type first. Mismatched types fail silently.
- The domain ID must be set correctly for multi-robot environments.
- Parameter files load in order: `robot.yaml` first, then each node's own file.
- Record rosbags with all required topics before leaving the lab.
- Verify bag contents with `ros2 bag info` after recording.
- The physical robot may have different limits than the documented 0.22 m/s.
- Before every simulation launch, not just when something looks wrong: check for and
  kill stray processes from a previous run (see the lab day checklist above). Ctrl-C
  does not clean up `gz sim` on its own.
