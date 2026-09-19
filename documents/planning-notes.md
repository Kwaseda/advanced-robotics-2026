# Lab 1 Design Notes

Design decisions behind the Lab 1 implementation, for reference alongside the code and
`lab1-guide.md`.

## Interface facts, fixed by the course

| Thing | Value | Note |
|---|---|---|
| Odometry in | `/odom`, `nav_msgs/Odometry` | |
| Command out | `/cmd_vel`, `geometry_msgs/TwistStamped` | not `Twist` -- see below |
| Setpoint in | `/new_position`, `geometry_msgs/Pose` | |
| Lidar in | `/scan`, `sensor_msgs/LaserScan` | |
| Domain ID | `3<robot number>` | e.g. turtle4 is 34, set from the launch file, not hardcoded |
| Robot IP | `192.168.50.<number x 10>` | ssh as user `turtle` |

## The four prerequisite tasks

1. Closed-loop position controller: subscribe odometry, publish command velocity,
   take setpoints from `/new_position`.
2. Figure-of-eight trajectory, driven inside the walls.
3. From the lidar, plot the closest wall point over time while the robot drives the
   eight.
4. Wall following from lidar until the robot completes a loop.

Plus: record rosbags of the relevant topics for each task.

## Why NID (Near Identity Diffeomorphism)

The unicycle model is non-holonomic: with two inputs (`v`, `omega`) and three states
(`x`, `y`, `theta`), the robot cannot move sideways, so its own centre cannot be
driven by a linear controller. NID controls a point a fixed distance `L` ahead of the
robot instead, which turns the problem into one a proportional controller can solve.

NID with a proportional outer loop is the Lab 1 controller. It is the theory the
tutorial names directly, and Lab 2's MPC controller sits behind the same
`Controller` interface (`controller_base.py`), so the swap between labs is a matter
of changing which class the launch file constructs, not a rewrite.

## Package split

Two packages:

- **r7021e_control** -- the control algorithms and the ROS nodes that run them.
  Nodes own subscriptions, timers, and message types; the actual control laws
  (`nid_controller.py`, `wall_following.py`, `trajectories.py`, `scan_utils.py`,
  `geometry.py`) are plain Python with no ROS imports, so they can be unit tested at
  a desk without a running node.
- **r7021e_bringup** -- launch files, parameter files, and the RViz configuration.
  Holds no nodes of its own.

Within `r7021e_control`, each task gets its own node rather than being folded into
the controller:

- `scan_monitor_node` (task 3) publishes the closest wall point on its own topic,
  since the controller has no use for that number and the plot needs it at full scan
  rate for a whole run.
- `trajectory_node` (task 2) publishes the figure-eight as a stream of setpoints on
  `/new_position`, the same topic a terminal command uses for task 1. This keeps
  `/new_position` the single entry point into the controller regardless of where the
  setpoint comes from, and it makes the current goal a recorded topic rather than a
  value that only exists inside the controller's own state.
- `goal_marker_node` republishes `/new_position` as a `visualization_msgs/Marker` on
  `/goal_marker`, since `/new_position` has no header and RViz has no display that
  can draw a bare `Pose` directly. Same reasoning as `scan_monitor_node`: the
  control loop has no use for a drawable marker, so it doesn't belong in a control
  node.

`controller_node` and `wall_follower_node` both publish to `/cmd_vel`, so exactly one
of them runs at a time -- selected by the launch file's `mode` argument rather than
left to whoever types the launch command, since two publishers on one topic interleave
silently instead of erroring.

## Physical limits

TurtleBot3 Burger, from the ROBOTIS datasheet:

| Quantity | Value |
|---|---|
| Max linear velocity | 0.22 m/s |
| Max angular velocity | 2.84 rad/s |
| Wheel radius | 0.033 m |
| Wheel separation | 0.160 m |
| Footprint | 0.178 x 0.138 x 0.192 m |
| Lidar range | 0.12 to 3.5 m |

Worth double-checking against the exact robot on lab day: other TurtleBot3 models
(e.g. the Waffle Pi) have different published limits, and firmware revisions have
been known to differ from the datasheet. Read `max_linear_velocity` off the robot's
own `turtlebot3_node` before quoting a number in the report.

## TwistStamped, not Twist

`/cmd_vel` on this course is `geometry_msgs/TwistStamped`, not the plain `Twist` used
in older TurtleBot3 tutorials. A `Twist` publisher against a `TwistStamped`
subscriber does not error -- it simply never connects, so nothing logs and the robot
sits still. `ros2 topic info /cmd_vel -v` is the first thing to check on any run that
isn't moving.

## Reflection question: where does the physical robot's odometry come from

Wheel encoders on the two Dynamixel servos, integrated by the OpenCR board,
optionally fused with the onboard IMU. There is no external position sensing on a
Burger, which is the direct answer to "what changes for sim-to-real transfer": in
Gazebo, odometry is close to ground truth; on the robot, it is dead-reckoning that
drifts, which is why the wall follower's loop closure explicitly does not claim to be
a true loop closure -- it detects that the odometry estimate has returned to the
start pose, not that the robot has.

## Decided

- `goal_defines`: staying with `offset_point` (the default). NID's entire value is
  the exactly-linear closed loop on the offset point; switching to `robot_centre`
  would satisfy the raw 0.05 m number at the robot's physical centre but give up the
  property the theory was taught for. The centre's 0.148 m distance from the goal
  under `offset_point` is reported and explained in the report (`L + goal_tolerance`,
  derived in the NID section of `lab1-guide.md`), not treated as a failure to hide.

- Tracking lag on the figure-eight (0.231 m mean: 0.10 m NID offset plus ~0.153 m of
  proportional lag against the moving setpoint, `setpoint speed / k_p`): left as-is,
  explained in the report rather than tuned away. Task 2's acceptance criterion says
  nothing about tracking error, and both cures (raising `k_p_position`, or adding
  feedforward) are real trade-offs, not free fixes -- see `lab1-guide.md`. Not a
  closed question forever: worth revisiting if the physical-robot sessions surface a
  reason to, but that is a decision for then, not now.

## Open items

None currently.
