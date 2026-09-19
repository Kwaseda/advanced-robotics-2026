# Lab 1 Complete Guide: ROS 2 Position Control and Wall Following

## Overview

This guide explains the R7021E Lab 1 implementation. The system does closed-loop position control and wall following on a TurtleBot3 robot. It uses Near Identity Diffeomorphism (NID) to linearize the non-holonomic unicycle model, which lets us control position with a simple proportional controller.

## Project Structure

The codebase has two ROS 2 packages.

r7021e_control contains the control algorithms and ROS nodes:
- controller_node.py does closed-loop position tracking using NID
- trajectory_node.py generates the figure-8 trajectory
- scan_monitor_node.py finds the closest wall point from lidar data
- wall_follower_node.py handles wall following
- Support modules: geometry.py, nid_controller.py, trajectories.py, wall_following.py, scan_utils.py

r7021e_bringup has launch files and configuration:
- lab1.launch.py is the main launch file for all four tasks
- rviz/lab1.rviz is the RViz configuration for visualization
- Parameter files are linked from config/

## Configuration Files

### robot.yaml
This file has the physical limits of the TurtleBot3 Burger:
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
Figure-8 parameters:
- Width: 1.0 m, Height: 0.5 m
- Lap period: 50.0 s
- Number of laps: 2
- Start delay: 3.0 s

### wall_follower.yaml
Wall following parameters:
- Follow distance: 0.5 m
- Speed: 0.15 m/s
- Loop closure tolerance: 0.2 m
- Follow side: right

## Building the System

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Testing Commands for Simulation

### Task 1 - Position Tracking

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

# Move to diagonal position
ros2 topic pub --once /new_position geometry_msgs/Pose "{position: {x: 0.5, y: 0.5}}"
```

Watch RViz to verify the robot reaches each goal and stops within tolerance.

### Tasks 2 & 3 - Figure of Eight with Wall Point Detection

Start the figure-eight trajectory:
```bash
ros2 launch r7021e_bringup lab1.launch.py sim:=true trajectory:=true rviz:=true
```

The robot will wait 3 seconds, then trace a figure-8 pattern for 2 laps. Verify wall point detection:
```bash
ros2 topic echo /closest_wall_point
```

### Task 4 - Wall Following

Start wall following:
```bash
ros2 launch r7021e_bringup lab1.launch.py sim:=true mode:=wall_following rviz:=true
```

The robot should locate the nearest wall, maintain 0.5m distance, navigate corners, and complete a full loop.

## Recording Rosbags

### Task 1 Recording
```bash
ros2 bag record -o bags/task1-position /odom /cmd_vel /scan /new_position /tf /tf_static
```

### Tasks 2 & 3 Recording
```bash
ros2 bag record -o bags/task2-eight /odom /cmd_vel /scan /new_position /tf /tf_static
```

### Task 4 Recording
```bash
ros2 bag record -o bags/task4-wall /odom /cmd_vel /scan /tf /tf_static
```

### Verify Bag Contents
```bash
ros2 bag info bags/<the bag>
```

Ensure all required topics are present: `/odom`, `/cmd_vel`, `/scan`, `/new_position`, `/tf`, `/tf_static`.

## Block Diagram

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
│  │  • NID Controller                        │       │
│  │  • Publish: /cmd_vel (TwistStamped)      │       │
│  └──────────────────┬───────────────────────┘       │
│                     │                               │
│                     ▼                               │
│  ┌──────────────────────────────────────────┐       │
│  │      TurtleBot3 (Physical/Sim)           │       │
│  │                                          │       │
│  │  • Subscribe: /cmd_vel                   │       │
│  │  • Publish: /odom, /scan                 │       │
│  │  • Wheel encoders + IMU                  │       │
│  │  • LDS-01/LDS-02 Lidar                   │       │
│  └──────────────────────────────────────────┘       │
│                                                     │
│  Parallel Processing:                               │
│  ┌──────────────────┐    ┌────────────────────┐     │
│  │ trajectory_node  │    │ scan_monitor_node  │     │
│  │                  │    │                    │     │
│  │ • Publish:       │    │ • Subscribe: /scan │     │
│  │   /new_position  │    │ • Publish:         │     │
│  │   (Figure-8)     │    │   /closest_wall_   │     │
│  │                  │    │     point          │     │
│  └──────────────────┘    └────────────────────┘     │
│                                                     │
└─────────────────────────────────────────────────────┘
```

### Control Loop Detail

```
Goal Position (x_goal, y_goal)
        │
        ▼
┌────────────────────┐
│  Error Calculation │
│  Δx = x_goal - x   │
│  Δy = y_goal - y   │
└─────────┬──────────┘
          │
          ▼
┌────────────────────┐
│   NID Transform    │
│                    │
│  1. Transform to   │
│     offset point   │
│     (L ahead)      │
│                    │
│  2. Linear control │
│     v_offset =     │
│     k_p * distance │
│                    │
│  3. Inverse NID    │
│     v, ω from      │
│     v_offset       │
└─────────┬──────────┘
          │
          ▼
┌────────────────────┐
│  Velocity Limits   │
│  Clamp to:         │
│  v_max = 0.22 m/s  │
│  ω_max = 2.84 rad/s│
└─────────┬──────────┘
          │
          ▼
    Publish /cmd_vel
```

## Dynamic Model

### Unicycle Model (Kinematic)

State variables: `x, y` (position), `θ` (heading)
Control inputs: `v` (linear velocity), `ω` (angular velocity)

Kinematic equations:
```
ẋ = v * cos(θ)
ẏ = v * sin(θ)  
θ̇ = ω
```

Non-holonomic constraint:
```
ẋ * sin(θ) - ẏ * cos(θ) = 0
```

### NID Transformation

Offset point (controlled point):
```
x_offset = x + L * cos(θ)
y_offset = y + L * sin(θ)
```

Where `L = 0.10 m` is the NID offset parameter.

Transformed dynamics (linearized around offset point):
```
ẋ_offset = v * cos(θ) - L * ω * sin(θ)
ẏ_offset = v * sin(θ) + L * ω * cos(θ)
```

Control law (proportional):
```
v_offset = k_p * sqrt((x_goal - x_offset)² + (y_goal - y_offset)²)
```

Inverse transform (robot commands from offset control):
```
v = v_offset * cos(θ)
ω = (v_offset / L) * sin(θ)
```

### Physical Parameters

- Wheel radius: `r = 0.033 m`
- Wheel separation: `d = 0.160 m`
- Max linear velocity: `v_max = 0.22 m/s`
- Max angular velocity: `ω_max = 2.84 rad/s`
- NID offset: `L = 0.10 m`
- Control gain: `k_p = 0.8`

## NID Explanation

### What NID Solves

The unicycle model is non-holonomic. The robot cannot move sideways. This makes direct position control hard because you cannot independently control x and y with just v and ω.

NID solves this by controlling a virtual point L meters ahead of the robot's axle. This turns the non-holonomic system into a holonomic one around the offset point, so we can use simple linear control.

### Visual Explanation

```
Robot at position (x, y) with heading θ:
     ↑
     │  θ
     │
     ●────→ (offset point)
    L
     
The offset point is L meters ahead of the robot.
When we control this point to follow a straight line,
the robot naturally follows a curved path that tracks the line.
```

### Why L Matters

Small L means the offset point is close to the robot. The transform then demands high ω for small lateral errors, which can saturate the angular velocity limit.

Large L puts the offset point far ahead. Tracking errors at the robot center become large, and the robot may overshoot goals.

With L = 0.10 m and v_max = 0.22 m/s, the maximum demanded ω is 0.22 / 0.10 = 2.2 rad/s. This is within the robot's 2.84 rad/s limit.

### Control Law

The proportional control law on the offset point is straightforward:

```
error = distance(offset_point, goal)
v_offset = k_p * error
```

The offset point behaves like a holonomic point that can move in any direction. The actual robot follows the non-holonomic constraints.

## Running on Physical Robot

### Lab Day Setup

1. Connect to the robot:
```bash
ssh turtle@192.168.50.<number x 10>
```

2. Start robot bringup (on the robot):
```bash
ros2 launch turtlebot3_bringup robot.launch.py
```

3. Set domain ID in each terminal:
```bash
export ROS_DOMAIN_ID=34  # for turtle4
```

4. Launch your controller (from your laptop):
```bash
ros2 launch r7021e_bringup lab1.launch.py use_sim_time:=false domain_id:=34
```

### Critical Lab Day Checklist

**Step 1: Verify message types**
```bash
ros2 topic info /cmd_vel -v
```
Both ends should show `geometry_msgs/msg/TwistStamped`. A mismatch means the robot won't move.

**Step 2: Check parameters**
```bash
ros2 param get /controller_node robot.max_linear_velocity
```
Should return 0.22. If it returns a different value, parameter files didn't load.

**Step 3: Verify robot limits**
Read the actual max_linear_velocity from the robot's turtlebot3_node before quoting 0.22 in the report. Firmware revisions may differ.

**Step 4: Run this before every simulation launch, no exceptions -- not only when something looks wrong**

Ctrl-C stops `ros2 launch` and its ROS nodes cleanly, but the `gz sim` server and GUI it started do not die with it. Confirmed, not occasional: it happens on every run, every time.

```bash
ps aux | grep -E "gzserver|gzclient|gz sim|ros2 launch|rviz2|robot_state_publisher" | grep -v grep
```
If anything shows up, `kill -9` the PIDs it lists, then rerun the command and confirm the output is empty, before launching anything new. A leftover `gz sim` process keeps running on its own clock and keeps publishing `/tf`, `/odom` and `/scan`. Several of these at once, each at a different simulated time, produces `TF_OLD_DATA` warnings in RViz and a robot that appears to jump between positions, with every node logging as if nothing is wrong.

## Common Issues and Solutions

### Robot doesn't move
- Check `/cmd_vel` message type match
- Verify ROS_DOMAIN_ID is set correctly
- Ensure parameter files loaded

### Parameters not loading
- Check that `config/` symlink exists in r7021e_bringup
- Verify YAML files are in the repository root `config/`
- Confirm you sourced the workspace after building

### Lidar returns invalid ranges
- Check range gating in robot.yaml (0.12 m to 3.5 m)
- Verify the lidar is actually running
- Check if the robot is in open space with no walls

### Loop never closes
- Robot may start too far from walls
- Check loop_min_distance parameter (2.0 m)
- Verify the robot actually finds a wall to follow

### Simulation looks glitchy, robot jumps between positions
- Almost always stray `gz sim` processes from a previous run that was never cleanly killed. See checklist step 4 above.

## Key ROS Topics

- `/odom` - Robot odometry (nav_msgs/Odometry)
- `/cmd_vel` - Velocity commands (geometry_msgs/TwistStamped)
- `/new_position` - Position setpoints (geometry_msgs/Pose)
- `/scan` - Lidar data (sensor_msgs/LaserScan)
- `/closest_wall_point` - Nearest wall point (geometry_msgs/PointStamped)

## Closed: the Video Can Now Show the Goal

The assignment requires the video to show the current goal at every instant. `/new_position` is a bare `geometry_msgs/Pose` with no header and no frame, so RViz had no display that could draw one directly, and the topic name and type are fixed by the course so the message itself couldn't change.

`goal_marker_node` subscribes `/new_position` and republishes it as a `visualization_msgs/Marker` (a green sphere) on `/goal_marker`, in the `odom` frame. `lab1.launch.py` starts it unconditionally alongside the other nodes, and `rviz/lab1.rviz` already has a Marker display on that topic, so it shows up with no extra setup.

Everything the video needs is in `rviz/lab1.rviz`: current position, the path so far as an odometry trail, the laser scan, and now the current goal.

## Assessment Questions and Answers

### What is NID and why is it needed?

NID (Near Identity Diffeomorphism) is a coordinate transformation that linearizes the non-holonomic unicycle model. The robot cannot move sideways because of the non-holonomic constraint, so direct position control is difficult. NID transforms the problem into controlling a point L meters ahead of the robot. This point behaves like a holonomic system that we can control with simple proportional control.

### How does the offset parameter L affect performance?

Small L values demand high angular velocities for small lateral errors. This can saturate the robot's ω limit. Large L values cause the robot center to track the goal with large offsets, which can lead to overshooting. The chosen L = 0.10 m balances these trade-offs. At v_max = 0.22 m/s, the maximum demanded ω is 2.2 rad/s, which stays within the robot's 2.84 rad/s limit.

### What happens if you increase k_p?

Higher k_p makes the controller more aggressive. This reduces settling time but increases the risk of overshoot and saturation. At k_p = 0.8, a 1 m error demands 0.8 m/s. This saturates at 0.22 m/s until the error falls below 0.275 m. Higher k_p would saturate longer and risk overshoot.

### Why use TwistStamped instead of Twist?

TwistStamped includes a header with frame_id and timestamp. This matters for tf transforms and time synchronization. A Twist publisher against a TwistStamped subscriber fails silently. Nothing logs, but the robot does not move because the endpoints never connect.

### How does wall following detect corners?

The wall follower uses a state machine with three states. In the acquiring state, the robot turns toward the nearest wall. In the following state, it maintains distance using proportional control. In the cornering state, it stops and turns when a front obstacle appears. Two lidar beams fit the wall as a line, which gives both distance and angle information.

### What happens if the lidar sees no walls?

If every beam is outside the valid range (0.12 to 3.5 m), the scan_monitor_node logs a warning and does not publish a closest wall point. The wall follower then enters the acquiring state and turns toward the nearest valid beam.

### Why is the control rate 20 Hz?

20 Hz is faster than the robot's mechanical response but slower than the lidar (5-10 Hz). This ensures the control loop never waits on sensor data it cannot use, while still being fast enough for effective control.

### How does the system handle odometry timeouts?

If no odometry arrives for 0.5 seconds, the controller stops commanding. It does not steer on stale pose data. This prevents the robot from executing commands based on where it was half a second ago.

### What happens if you change the goal_defines parameter?

The goal_defines parameter switches between two interpretations. In offset_point mode, the setpoint is where the NID point should end up. The NID closed loop stays exactly linear, but the robot center settles L meters behind the goal. In robot_centre mode, the setpoint is where the robot center should end up. The goal shifts forward by L, so the center converges to the commanded point, but the loop loses exact linearity.

### How does the figure-eight trajectory work?

The trajectory node publishes a Gerono lemniscate. The equations are x(s) = centre_x + width * sin(s) and y(s) = centre_y + height * sin(2s), where s = 2π * (elapsed / lap_period). The double frequency in y creates the figure-8 crossing. The node publishes the setpoint at 20 Hz, so the controller gets a fresh goal on every control tick.

## Quick Verification

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

## Important Notes

- Always check the /cmd_vel message type first. Mismatched types fail silently.
- The domain ID must be set correctly for multi-robot environments.
- Parameter files load in order. robot.yaml loads first, then node-specific files.
- Record rosbags with all required topics before leaving the lab.
- Verify bag contents with ros2 bag info after recording.
- The physical robot may have different limits than the documented 0.22 m/s.
- TwistStamped headers must be stamped with the node clock for tf to work correctly.
- Before every simulation launch, not just when something looks wrong: check for and kill stray processes from a previous run (see the Lab Day Checklist above). Ctrl-C does not clean up `gz sim` on its own.
