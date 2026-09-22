# Lab 2 guide: Model Predictive Control on the TurtleBot3 Burger

Everything needed to build, run and demonstrate the four Lab 2 tasks. The reasoning
behind the design choices and the measurements behind every number are in
[lab2-report-notes.md](lab2-report-notes.md).

## What Lab 2 is

A nonlinear MPC replacing Lab 1's proportional controller. Same interfaces: `/odom` in,
`/new_position` for the goal, `/cmd_vel` out as a `TwistStamped`. What changed is the
thing in the middle.

At every control step the controller predicts 2 seconds ahead using the unicycle model,
scores every candidate input sequence against a cost, throws away any sequence that
breaks a constraint at any predicted step, and applies only the first input of the best
one that survives. Then it measures again and re-solves. That is the receding horizon,
and it is what lets an MPC respect a rule that only becomes a problem a second from now,
which a reactive controller cannot do.

Four tasks:

| Task | What it demonstrates |
|---|---|
| 1 | Setpoint tracking with input constraints and a boundary box, one goal inside it and one outside |
| 2 | One static obstacle, as a hard nonlinear constraint |
| 3 | Two static obstacles at once |
| 4 | Circular trajectory tracking while avoiding an obstacle |

## One-time setup

`do-mpc` and `casadi` are pip packages, not ROS dependencies, so `rosdep` will not install
them.

```bash
pip install --user --break-system-packages "casadi==3.7.2" "do-mpc==5.1.1"
```

**Pin casadi to 3.7.2.** 3.8.0 removed a `casadi.tools` re-export that do-mpc's import
needs, and 3.6.5 breaks under NumPy 2. Pin it before installing do-mpc, not after
something fails.

The `--user` install lands in `~/.local/lib/python3.12/site-packages`, which the
`#!/usr/bin/python3` in every colcon entry point already sees, so nothing needs sourcing
before a launch. On a Python that is not externally managed, drop
`--break-system-packages`.

Check it worked:

```bash
python3 -c "import do_mpc, casadi; print(do_mpc.__version__, casadi.__version__)"
```

Three warnings about ONNX, OPC UA and approximate MPC on import are expected. They are
optional features the base install does not ship, and the nodes silence them.

## Build

`lab2-files/ros2_ws` is its own workspace. The package names differ from Lab 1's, so both
labs can be sourced at once without shadowing each other.

```bash
cd lab2-files/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Running the four tasks

Add `sim:=true` for Gazebo and `rviz:=true` to watch. On the robot, `use_sim_time:=false`
and set `domain_id` to yours.

### Task 1, setpoint tracking

```bash
ros2 launch r7021e_mpc_bringup lab2.launch.py task:=1 sim:=true rviz:=true
```

In a second terminal, with the same `ROS_DOMAIN_ID` exported:

```bash
ros2 topic pub --times 6 --rate 2 /new_position geometry_msgs/msg/Pose "{position: {x: 0.8, y: 0.5, z: 0.0}}"
```

Then the goal outside the boundary, which is the part the task actually asks for:

```bash
ros2 topic pub --times 6 --rate 2 /new_position geometry_msgs/msg/Pose "{position: {x: 1.5, y: 0.0, z: 0.0}}"
```

The robot drives to `x = 1.0`, stops, and logs the solver status. That is correct
behaviour, not a failure: the goal is outside the feasible set that the boundary
constraint defines, so the closest point the controller is allowed to reach is the
boundary itself.

**Publish the goal repeatedly, not with `--once`.** A single `--once` publish creates a
publisher and exits immediately, and a bag recorder that has not yet matched it loses the
message entirely. The controller still receives it, so nothing looks wrong until the
trajectory plot comes out with no goal on it.

### Task 1c, varying sampling time and horizon

Both are parameters, so no rebuild is needed:

```bash
ros2 launch r7021e_mpc_bringup lab2.launch.py task:=1 sim:=true \
  --ros-args -p t_step:=0.2 -p n_horizon:=10
```

The number that matters is neither one alone. It is `n_horizon * t_step * v_max`, the
distance the controller can see ahead. At the defaults that is
`20 * 0.1 * 0.22 = 0.44 m`, which is less than one inflated obstacle diameter, and it
explains most of the behaviour in tasks 2 to 4.

### Task 2, one obstacle

```bash
ros2 launch r7021e_mpc_bringup lab2.launch.py task:=2 sim:=true rviz:=true
ros2 topic pub --times 6 --rate 2 /new_position geometry_msgs/msg/Pose "{position: {x: 1.5, y: 0.0, z: 0.0}}"
```

The obstacle is a constraint in the controller and a marker in RViz, drawn from the same
numbers, not a physical body in Gazebo. That is what the task asks for and it works
identically on the real robot, where there is no simulator to put a body in.

### Task 3, two obstacles

```bash
ros2 launch r7021e_mpc_bringup lab2.launch.py task:=3 sim:=true rviz:=true
ros2 topic pub --times 6 --rate 2 /new_position geometry_msgs/msg/Pose "{position: {x: 1.8, y: 0.0, z: 0.0}}"
```

### Task 4, circular trajectory with an obstacle

```bash
ros2 launch r7021e_mpc_bringup lab2.launch.py task:=4 sim:=true rviz:=true
```

No typed setpoint. `trajectory_node` publishes the circle on `/new_position` and its
future over the next 20 steps on `/reference_path`. The robot spends the first 8 seconds
driving from the origin onto the circle, then tracks two laps, bulging outward near the
top of each lap to clear the obstacle.

## Topics

| Topic | Type | Direction |
|---|---|---|
| `/odom` | `nav_msgs/Odometry` | in |
| `/new_position` | `geometry_msgs/Pose` | in, the goal |
| `/reference_path` | `nav_msgs/Path` | in, optional, the goal's future over the horizon |
| `/cmd_vel` | `geometry_msgs/TwistStamped` | out |
| `/mpc_prediction` | `nav_msgs/Path` | out, the horizon the solver just planned |
| `/mpc_obstacles` | `visualization_msgs/MarkerArray` | out, the keep-out zones |
| `/goal_marker` | `visualization_msgs/Marker` | out, `/new_position` made drawable |

`/mpc_prediction` is the one worth watching in RViz. It is the plan itself, redrawn ten
times a second, and it bends around an obstacle before the robot gets anywhere near it.
No reactive controller has anything equivalent to show.

## Parameters worth knowing

All in `r7021e_mpc_bringup/config/`. `robot.yaml` loads first and owns the physical
limits, `mpc.yaml` holds the shared defaults, and `mpc_task<N>.yaml` changes only what
that task needs.

| Parameter | Default | What it does |
|---|---|---|
| `t_step` | 0.1 | Prediction step and control period, one number for both |
| `n_horizon` | 20 | Steps predicted ahead |
| `boundary_x`, `boundary_y` | `[-1, 1]` | The hard boundary box, widened per task |
| `obstacle_x/y/radius` | unset | Parallel arrays, one entry each per obstacle |
| `obstacle_inflation` | 0.105 | Added to every obstacle radius for the robot's footprint |
| `obstacle_soft` | false | Whether obstacles are hard constraints or penalised ones |
| `lab.max_linear_velocity` | 0.5 | The lab's stated bound |
| `robot.max_linear_velocity` | 0.22 | The Burger's real bound |

The node uses the tighter of the two velocity bounds. Both are declared so neither number
has to be quietly edited to mean the other, and a model predicting 0.5 m/s on a robot
that saturates at 0.22 plans a future it cannot reach.

## Recording

```bash
python3 scripts/record_demo.py bags/task2-obstacle
```

Records the bag and a screen capture of RViz together. Start the launch file yourself
first, with `rviz:=true`, then run this in a second terminal and publish the goal in a
third.

Plotting, with the obstacles and boundary drawn from the same parameter file the
controller loaded:

```bash
python3 scripts/plot_trajectory.py bags/task2-obstacle \
  --constraints ros2_ws/src/r7021e_mpc_bringup/config/mpc_task2.yaml \
  --base-config ros2_ws/src/r7021e_mpc_bringup/config/mpc.yaml \
  -o bags/task2_trajectory.png
```

Each obstacle is drawn twice: the physical object, and the inflated radius the solver
actually enforced. A path that hugs the outer circle is correct, not a near miss.

## Lab day checklist

1. `export ROS_DOMAIN_ID=3<robot number>` in every terminal. The launch file sets it for
   the nodes it starts, but not for the shell you typed it in.
2. `export TURTLEBOT3_MODEL=burger` if you are running the simulator.
3. Confirm `/cmd_vel` is `TwistStamped`, not `Twist`. An unstamped header on a stamped
   message is a message that arrived at the epoch; Gazebo does not notice and a real
   robot with a tf tree does.
4. `ros2 param get /mpc_node obstacle_radius` after launching. A parameter file that
   installed empty builds clean and passes every unit test.
5. Publish goals repeatedly, never with `--once`.
6. After every simulator run, check for strays. `gz sim`'s server and GUI do not die on
   Ctrl-C:

```bash
ps aux | grep -E "gzserver|gzclient|gz sim|ros2 launch|rviz2|robot_state_publisher" | grep -v grep
```

## If something goes wrong

**`ModuleNotFoundError: No module named 'do_mpc'`.** The pip install went somewhere the
node's interpreter cannot see. Check with `head -1 install/r7021e_mpc/lib/r7021e_mpc/mpc_node`
which interpreter the entry point uses, then run `python3 -c "import do_mpc"` with that
same interpreter.

**`cannot import name 'tools' from 'casadi'`.** casadi is not 3.7.2. Reinstall it pinned.

**The robot stops short of the goal and logs `Infeasible_Problem_Detected`.** No input
sequence respects every constraint over the whole horizon. Usually the goal is outside the
boundary box (task 1b, where this is the expected result), or an obstacle's inflated
radius has closed the only corridor to the goal. Widen the boundary, move the obstacle, or
check that the obstacle is not sitting exactly on the straight line between the robot and
the goal, which makes left and right equally good and leaves the solver no gradient to
follow.

**The robot stops dead in front of an obstacle and never moves again.** The measured state
is inside a hard keep-out zone, and a hard constraint is enforced on the measured state as
well as the predicted ones, so no input can satisfy it. Set `obstacle_soft: true` for that
task. The penalty is high enough that clearance is never traded for tracking.

**Solve time approaching `t_step`.** The log line reports it every second. Reduce
`n_horizon` or raise `t_step`, and remember that lookahead distance is the product of the
two with the speed limit, so halving one and doubling the other leaves it unchanged.
