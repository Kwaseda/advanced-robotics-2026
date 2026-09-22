# Design Notes

Design decisions behind the implementation, for reference alongside the code and the
per-lab guides. Lab 1 first, then Lab 2.

## Lab 1

### Interface facts, fixed by the course

| Thing | Value | Note |
|---|---|---|
| Odometry in | `/odom`, `nav_msgs/Odometry` | |
| Command out | `/cmd_vel`, `geometry_msgs/TwistStamped` | not `Twist` -- see below |
| Setpoint in | `/new_position`, `geometry_msgs/Pose` | |
| Lidar in | `/scan`, `sensor_msgs/LaserScan` | |
| Domain ID | `3<robot number>` | e.g. turtle4 is 34, set from the launch file, not hardcoded |
| Robot IP | `192.168.50.<number x 10>` | ssh as user `turtle` |

### The four prerequisite tasks

1. Closed-loop position controller: subscribe odometry, publish command velocity,
   take setpoints from `/new_position`.
2. Figure-of-eight trajectory, driven inside the walls.
3. From the lidar, plot the closest wall point over time while the robot drives the
   eight.
4. Wall following from lidar until the robot completes a loop.

Plus: record rosbags of the relevant topics for each task.

### Why NID (Near Identity Diffeomorphism)

The unicycle model is non-holonomic: with two inputs (`v`, `omega`) and three states
(`x`, `y`, `theta`), the robot cannot move sideways, so its own centre cannot be
driven by a linear controller. NID controls a point a fixed distance `L` ahead of the
robot instead, which turns the problem into one a proportional controller can solve.

NID with a proportional outer loop is the Lab 1 controller. It is the theory the
tutorial names directly, and Lab 2's MPC controller sits behind the same
`Controller` interface (`controller_base.py`), so the swap between labs is a matter
of changing which class the launch file constructs, not a rewrite.

### Package split

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

### Physical limits

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

### TwistStamped, not Twist

`/cmd_vel` on this course is `geometry_msgs/TwistStamped`, not the plain `Twist` used
in older TurtleBot3 tutorials. A `Twist` publisher against a `TwistStamped`
subscriber does not error -- it simply never connects, so nothing logs and the robot
sits still. `ros2 topic info /cmd_vel -v` is the first thing to check on any run that
isn't moving.

### Reflection question: where does the physical robot's odometry come from

Wheel encoders on the two Dynamixel servos, integrated by the OpenCR board,
optionally fused with the onboard IMU. There is no external position sensing on a
Burger, which is the direct answer to "what changes for sim-to-real transfer": in
Gazebo, odometry is close to ground truth; on the robot, it is dead-reckoning that
drifts, which is why the wall follower's loop closure explicitly does not claim to be
a true loop closure -- it detects that the odometry estimate has returned to the
start pose, not that the robot has.

### Decided

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

### Open items

None currently.


## Lab 2

Design decisions behind the MPC implementation, alongside `lab2-guide.md`. The
measurements that justify each of them are in `lab2-report-notes.md`.

### Why a separate package rather than a second controller in `r7021e_control`

`controller_base.py` defines a `Controller` interface (`compute`, `reset`) that Lab 1
wrote before a second controller existed, on the argument that a Lab 2 MPC would sit
behind it. That argument held: `MPCController` implements the same interface unchanged,
and `controller_base.py` is carried into Lab 2 rather than rewritten.

Lab 2 still ships as its own package with its own node. The reason is delivery, not
design: `lab2-files/` has to be downloadable and runnable on its own, and a shared
controller node would drag NID, wall following and the scan monitor into a lab that does
not use any of them. The package names differ from Lab 1's for the same reason, so both
workspaces can be sourced at once without shadowing each other.

### Package split, unchanged in principle from Lab 1

- **r7021e_mpc** -- the control law and the nodes. `mpc_controller.py` is plain Python
  with no ROS imports, so it is unit tested at a desk; `mpc_node.py` owns subscriptions,
  the timer, the message types and the header, and nothing else.
- **r7021e_mpc_bringup** -- launch file, parameter files, RViz configuration. No nodes.

Parameter files are real files inside the package's own `config/`, not a symlink to a
shared directory. A symlinked config directory can install empty, which builds clean and
passes every unit test.

### Why the solver runs on its own timer

`mpc.make_step()` is called from a timer at `t_step`, never from the `/odom` callback. A
solver called from a sensor callback runs at sensor rate rather than at its designed
sample time, and the horizon then covers a different span of real time on every tick.

`t_step` is one number serving both the prediction step and the control period, so the
first predicted interval is exactly the interval the command is held for.

### Why the setpoint is a time-varying parameter

do-mpc compiles the horizon, the bounds and the obstacle set into an NLP at `setup()`
time. A setpoint written as a constant in the objective would mean rebuilding and
recompiling that NLP every time the goal changed, which takes seconds. As a `_tvp` it is
a number handed to an already-built solver, which takes microseconds.

It also makes trajectory tracking possible. The horizon can be filled with where the
reference *will* be at each future step rather than where it is now, which is the
difference between tracking a curve and lagging behind it.

### Why `/reference_path` is a separate optional topic

`/new_position` stays the single control entry point, exactly as in Lab 1, so a terminal
setpoint, the trajectory generator and any later planner all drive the controller through
the same message, and the current goal remains a recorded topic the marker node can draw.

`/reference_path` carries the same goal's future over the horizon. Only an MPC can use
it; a reactive controller ignores it. Making it a second topic rather than changing
`/new_position` means nothing that consumed the setpoint before has to change.

### Why obstacles are constraints and markers, not Gazebo bodies

The lab asks for an obstacle avoidance constraint in the controller. Drawing the same
numbers as RViz markers gives the video something to show without introducing a second
source of truth that can silently disagree with the constraint, and it works unchanged on
the real robot where there is no simulator to put a body in.

Obstacle radii are inflated by the robot's footprint before reaching the solver, because
the MPC model is a point. The inflation happens in one place, so the marker and the
constraint are always the same circle.

### Hard constraints, and the one place they are not

Boundary and obstacle constraints are hard, matching the tutorial. Tasks 2 and 3 use hard
obstacle constraints and both are verified in simulation.

Task 4 uses a soft constraint with a high penalty, and this is the one design decision
here that departs from the tutorial. A hard constraint is enforced at every horizon step
including step zero, and step zero is the measured state, not a decision variable. If the
measurement itself violates the constraint there is no input that satisfies the problem,
and the controller stops permanently. That cannot happen when the optimal path passes an
obstacle with margin, and it does happen when the optimal path rides the constraint
boundary, which is what tracking a reference through an obstacle asks for.

The penalty is four orders of magnitude above the tracking weight, so with nothing
perturbing it the soft constraint produces the same path as the hard one to within a
millimetre. The slack it uses under disturbance comes out of the footprint inflation
margin rather than out of real clearance to the object.

### Physical limits, and which bound binds

The lab states `0 < v < 0.5` and `-0.8 < omega < 0.8`. The Burger does 0.22 m/s and
2.84 rad/s. Both sets are declared as separate parameters and the node takes the tighter
of each pair, rather than one number being edited to mean both. A model allowed to predict
0.5 m/s on a robot that saturates at 0.22 plans a future the robot cannot reach, and every
plan it makes is then wrong in the same direction without the controller finding out.
