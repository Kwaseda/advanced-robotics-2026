# Lab 2: decisions, justifications and measurements

Everything the written report needs, in the order the report will want it. Design
decisions with the alternative that was rejected, the measurements behind each number,
and the findings that are worth explaining rather than hiding.

Read alongside `lab2-guide.md`, which covers how to build and run the four tasks.

Every number below is marked with an evidence tier. `desk` means measured by running the
control law in plain Python against Euler integration of the same unicycle model the
controller predicts with, on 2026-09-21. `sim` means observed in Gazebo Sim 8.15.0, world
`turtlebot3_dqn_stage1`, TurtleBot3 Burger spawned at the origin, on 2026-09-22. `robot`
means observed on a physical Burger, and nothing here is `robot` yet.

All four tasks have been run in Gazebo and every headline number below is `sim`. Section
10 is the results table.

---

## 1. What was built

A nonlinear MPC for the TurtleBot3 Burger, using do-mpc 5.1.1 over CasADi 3.7.2, wired
into ROS 2 Jazzy as a node that subscribes `/odom`, takes setpoints on `/new_position`,
and publishes `geometry_msgs/TwistStamped` on `/cmd_vel`. Four tasks, one launch file,
one argument selecting between them.

| File | What it is |
|---|---|
| `mpc_controller.py` | The control law. Plain Python, no ROS imports, unit tested at a desk. |
| `mpc_node.py` | The ROS node. Subscriptions, timer, message types, headers. Nothing else. |
| `trajectories.py` | `CircleTrajectory` beside the figure of eight, same interface. |
| `trajectory_node.py` | A `shape` parameter, and optional `/reference_path` publishing. |
| `config/mpc*.yaml`, `circle.yaml` | One base file plus one overlay per task. |
| `lab2.launch.py`, `lab2.rviz` | Launch and visualisation. |

## 2. The model

State is the unicycle pose, input is the pair the robot actually has:

```
x_dot  = v cos(theta)
y_dot  = v sin(theta)
theta_dot = omega
```

Written symbolically with `ca.cos` and `ca.sin` so CasADi can differentiate it for the
solver's Jacobian. No NID transform appears anywhere in Lab 2. NID exists to make a
non-holonomic robot tractable for a linear controller; do-mpc solves the nonlinear model
directly, so the trick is not needed. The Lab 2 tutorial says as much in its own
Turtlebot section, and links the same Robotarium paper Lab 1 used.

## 3. The cost function

```
J = sum over k of  q * ((x_k - x_des)^2 + (y_k - y_des)^2)          stage cost, lterm
  +                q_terminal * (same, at the last predicted step)  terminal cost, mterm
  +                r * (change in v)^2 + r * (change in omega)^2    rterm
```

with `q = q_terminal = 1.0` and `r = 0.01`, matching the tutorial, which sets
`mterm = lterm`.

What each term buys, which is what an examiner will ask:

- Without `mterm`, nothing scores where the horizon ends, so the solver is free to
  return a plan that is travelling fast past the goal at the final step.
- `rterm` penalises the change in input between steps, not its magnitude. Without it the
  cheapest plan is often a jerky one, and jerky `v` on a real Burger is wheel slip. At
  0.01 against a tracking weight of 1.0 it is smoothing, not a competing objective.

## 4. Constraints, and which bound actually binds

Three kinds, exactly as the tutorial describes them.

| Kind | do-mpc API | This lab's values |
|---|---|---|
| Input | `bounds['...', '_u', ...]` | `0 <= v <= 0.5`, `-0.8 <= omega <= 0.8` |
| State (boundary) | `bounds['...', '_x', ...]` | `-1.0 < x < 1.0`, `-1.0 < y < 1.0` (task 1) |
| Nonlinear (obstacle) | `set_nl_cons(..., soft_constraint=False)` | `r^2 - ((x-x_o)^2 + (y-y_o)^2) <= 0` |

The lab's input bounds are looser than the robot. The Burger does 0.22 m/s and
2.84 rad/s. The node takes the minimum of each pair rather than picking one, so the
model never predicts a speed the robot cannot execute. This matters more than it sounds:
a model that plans at 0.5 m/s on a robot that saturates at 0.22 predicts a future the
robot never reaches, and every plan it makes is wrong in the same direction without the
controller ever finding out. Measured at launch: `v` is capped at 0.22, `omega` at 0.8,
with `lab.max_linear_velocity` still reading 0.5 on the running node.

Obstacle radii are inflated by 0.105 m before reaching the solver. The MPC model is a
point and the robot is not; the Burger's 178 by 138 mm footprint gives a circumscribing
radius of about 0.112 m. Without inflation the predicted point can sit legally on an
obstacle's edge with half a TurtleBot inside it. The inflation happens in one place, so
the RViz marker and the constraint are drawn from the same number and cannot disagree.

Every obstacle constraint is hard. A soft constraint is a preference the solver can buy
its way out of, and a keep-out zone is not a preference.

## 5. Horizon and sampling time

`t_step` is both the prediction step and the period of the control timer. One number for
both, so the first predicted interval is exactly the interval the command is held for.
Decoupling them would put a known modelling error into the first step of every
prediction.

Default `t_step = 0.1 s`, `n_horizon = 20`, so 2.0 s of lookahead.

`desk` Solve time with two obstacles active: 31.7 ms mean, 45.1 ms peak over 40 solves.
`sim` is not available for this number yet, but the running node logs it every second:
at launch, task 1 solved in 13.8 to 17.3 ms and task 4 in 17.6 to 19.9 ms, against a
100 ms budget.

Why not Lab 1's 20 Hz: at `t_step = 0.05` the budget halves to 50 ms and the measured
peak of 45.1 ms consumes 90 percent of it. A solve that overruns its timer does not
error, it just runs late, and the symptom looks like a badly tuned controller.

### Task 1c, varying the two parameters

The horizon in metres, not steps, is the number that matters, and it is
`n_horizon * t_step * v_max`. For the tutorial's robot at 0.8 m/s that is 1.6 m. For the
Burger at 0.22 m/s the same 20 steps of 0.1 s is **0.44 m**, which is shorter than one
inflated obstacle diameter. This single fact explains most of what follows.

`desk` Horizon sweep against the tutorial's own two-obstacle layout:

| `t_step` | `n_horizon` | Lookahead | Result |
|---|---|---|---|
| 0.1 | 20 | 2.0 s / 0.44 m | stops at (0.280, -0.111) |
| 0.1 | 40 | 4.0 s / 0.88 m | stops at (0.314, -0.142) |
| 0.1 | 60 | 6.0 s / 1.32 m | stops at (0.315, -0.142) |
| 0.2 | 30 | 6.0 s / 1.32 m | stops at (0.313, -0.140) |
| 0.2 | 40 | 8.0 s / 1.76 m | stops at (0.313, -0.140) |
| 0.3 | 30 | 9.0 s / 1.98 m | stops at (0.296, -0.123) |

Every one of them stops within 0.04 m of the same place. Lookahead was not the binding
problem there; see section 7.

## 6. Task 1b, the setpoint outside the boundary

This is the result that did not match the expectation going in, and it is the more
interesting outcome.

It behaves differently at a desk and in Gazebo, and the difference is the whole lesson.

`desk` With the box at `x in [-1, 1]` and a goal at (1.5, 0.0), the solve does **not**
fail. IPOPT returns `Solve_Succeeded` at every step and the robot drives to exactly
x = 1.000 and holds there. The optimizer finds the closest feasible point to a goal it is
forbidden to reach and parks on it. A hard state constraint does not make the problem
unsolvable; it shrinks the feasible set, and the optimum moves to the edge of what
remains.

`sim` In Gazebo the robot drives to the boundary, overshoots it by 17 mm (final position
x = 1.017), and IPOPT then reports `Infeasible_Problem_Detected`. The node publishes zero
velocity, logs the status once, and the robot holds at the boundary:

> solver did not converge: Infeasible_Problem_Detected. holding position at (1.000, 0.000),
> goal (1.500, 0.000) is 0.500 m away

Both observations are correct and they are about different things. The desk model is the
same unicycle the controller predicts with, so the predicted state and the actual state
agree exactly and the robot stops precisely on the constraint. Gazebo has wheel dynamics,
acceleration limits and a command held for a full 100 ms, so the actual state lands a
little past the predicted one. Once the *measured* state is outside the box, no input
satisfies a constraint that is also enforced at step zero, and the problem is infeasible.

The 17 mm is the tutorial's own discretisation caveat (section 7e) appearing on the
boundary constraint rather than on an obstacle: the constraint is enforced at the sampled
instants, and the motion between them is not checked.

So the failure path in the node, publish zero and log the status once, is not a
theoretical safety net. It is what holds the robot at the boundary in task 1b, and it is
what makes the task-1b run stable rather than a robot grinding into an invisible wall.

## 7. Five findings that cost real time, and are worth the report's space

### 7a. A head-on obstacle deadlocks on symmetry

With the robot at the origin facing along +x, an obstacle centred exactly on the x axis,
and the goal further along the same axis, detouring left and detouring right have
identical cost. The gradient the solver follows is zero in the direction that would
resolve the tie.

`desk` Obstacle at (0.75, 0.00), goal (1.5, 0.0): the robot drives straight to
x = 0.495, which is the inflated boundary, and stops. Maximum |y| over the whole run is
0.000. Moving the same obstacle to (0.75, 0.08) fixes it completely: the robot detours
0.184 m and reaches the goal, with a closest approach of 0.256 m against a 0.255 m
constraint radius.

Every obstacle in the tutorial's own examples is offset from the axis. The tutorial never
says why. This is why.

### 7b. The tutorial's obstacle coordinates are infeasible at the Burger's speed

`desk` The tutorial's two-obstacle layout, (0.5, 0.1) radius 0.2 and (1.4, -0.3) radius
0.3, with a goal at (1.8, 0.0): the robot stops at (0.280, -0.111), pressed against the
first obstacle's inflated boundary, and IPOPT reports
`Infeasible_Problem_Detected`. Raising the horizon does not help, as section 5 shows.

The cause is that the tutorial bounds `vx` at 0.8 m/s and the Burger does 0.22. The
tutorial's horizon covers 1.6 m of travel and sees past both obstacles; the Burger's
covers 0.44 m and does not. Inflating both obstacles by the robot's own footprint then
closes the corridor between them from a head-on start.

Lab 2's task 3 therefore uses a slalom instead: (0.7, 0.15) and (1.3, -0.15), both radius
0.15, goal (1.8, 0.0). `desk` Reaches the goal, closest approach 0.256 m against a
0.255 m constraint radius, so both constraints are active and respected rather than idle.

This is not a workaround to be quiet about. It is the lab's own instruction, "Adjust the
boundary constraints if necessary", generalised: a constraint set copied from a faster
robot does not transfer unchanged to a slower one.

### 7c. Where the obstacle sits relative to a reference trajectory decides feasibility

Found while testing task 4, and the third instance of the same underlying cause.

`desk` Same circle, same controller, five obstacle placements:

| Obstacle | Relative to the circle | Result |
|---|---|---|
| (0.00, 0.80) r 0.15 | centred on it | `Restoration_Failed` at t = 24.8 s |
| (0.00, 0.95) r 0.12 | outside | `Infeasible_Problem_Detected` at t = 21.9 s |
| (0.00, 1.00) r 0.15 | outside | `Infeasible_Problem_Detected` at t = 22.0 s |
| (0.35, 0.90) r 0.12 | outside | `Infeasible_Problem_Detected` at t = 18.5 s |
| (0.00, 0.62) r 0.12 | inside | completes the lap |

An obstacle centred on the reference puts the reference itself inside the keep-out zone,
so the robot is pinned between a hard constraint and a cost pulling it into that
constraint, and the solver's restoration phase fails.

An obstacle outside the circle can only be passed by swinging out past r = 1.17 m, an
excursion of 0.37 m from the reference. The horizon is 0.44 m of travel. The solver
cannot see far enough around the obstacle to commit to the detour, so it drives into the
boundary of the keep-out zone and the problem goes infeasible. Same root cause as 7b.

Inside the circle, the required detour is 45 mm and the solver finds it immediately.

The general statement, which is the one worth putting in the report: for a hard obstacle
constraint against a moving reference, the required detour has to be smaller than the
distance the horizon covers. That is a design rule, not a tuning tip, and it is not
stated anywhere in the tutorial.

### 7d. A hard constraint on the measured state cannot be recovered from

The finding that changed what ships, and the one most worth explaining out loud.

Task 4 ran correctly at a desk and deadlocked in Gazebo. The robot pinned at
(-0.019, 0.844), which is 0.2248 m from the obstacle centre against a 0.225 m constraint
radius, and every subsequent solve reported `Infeasible_Problem_Detected`. Because the
node publishes zero on failure, the robot never moved again.

The mechanism is the same as task 1b's, with worse consequences. A hard `set_nl_cons` is
enforced at every step of the horizon **including step zero**, and step zero is the
measured state, which is not a decision variable. A measurement that already violates the
constraint makes the problem infeasible for every possible input, permanently.

Why only task 4. Tasks 2 and 3 pass their obstacles with margin, so a millimetre of
overshoot changes nothing. Task 4 tracks a reference that runs *through* the keep-out
zone, so the optimal path rides the constraint boundary for a sustained stretch.
`desk` with zero actuation delay the task 4 circle clears its obstacle by **0.1 mm**.
There is no margin left to absorb anything.

Reproduced at a desk by injecting actuation delay, which is the cheapest stand-in for
what Gazebo does (the command is computed from a pose that is already old):

| Constraint | Actuation delay | Result |
|---|---|---|
| hard | 0 ms | lap completes, clears by 0.1 mm |
| hard | 200 ms | 1.6 mm inside, deadlocks at t = 22.6 s |
| hard | 300 ms | deadlocks at t = 20.4 s |
| soft, penalty 1e4 | 0 ms | identical to hard, to within 1e-3 m |
| soft, penalty 1e4 | 200 ms | lap completes, 4.7 mm inside, mean error 0.016 m |
| soft, penalty 1e4 | 300 ms | lap completes, 26.4 mm inside, mean error 0.062 m |

Velocity error alone does not reproduce it: plus or minus 5 percent on the executed
velocity still completes the lap. It is latency, not gain error.

**What ships, and the justification.** Tasks 2 and 3 keep hard constraints, matching the
tutorial, and both are verified in Gazebo. Task 4 uses a soft constraint with a penalty of
1e4 against a tracking weight of 1.0.

Three reasons that is the right trade rather than a weakening of the requirement:

1. The soft constraint is identical to the hard one whenever the hard one is satisfiable.
   Measured: with no perturbation the two produce the same closest approach to within
   1e-3 m. The solver does not buy clearance cheaply; the penalty is four orders above
   the tracking cost.
2. The slack is spent against the **inflation margin**, not against real clearance. The
   obstacle is inflated by 0.105 m for the robot's footprint, so 4.7 mm of slack still
   leaves about 0.100 m between the robot and the physical object.
3. The alternative is a controller that stops permanently the first time a measurement
   lands a millimetre inside a keep-out zone, which on a real robot is a worse safety
   property, not a better one.

`sim` With the soft constraint, task 4 completes with zero infeasible warnings, mean
tracking error 0.004 m, and a closest approach of 0.224 m against the 0.225 m constraint:
1 mm of slack used, out of a 105 mm inflation margin.

### 7e. Discretisation, the tutorial's own caveat

The obstacle constraint is checked at the sampled instants only, never on the straight
line between two of them. At a large `t_step` a plan can hop across a small obstacle with
every sampled point legally outside it. The tutorial demonstrates this on purpose at
`t_step = 1.0` and fixes it at 0.1. The same limit applies here and is a property of the
method, not of this implementation.

## 8. Task 4, trajectory tracking

A circle of radius 0.8 m about the origin, 60 s per lap, two laps, with one obstacle at
(0.0, 0.62) radius 0.12 m, sitting just inside the circle so its inflated radius of
0.225 m overlaps the reference.

`desk` Full lap: mean tracking error 0.010 m, maximum 0.046 m, closest approach to the
obstacle 0.225 m against a 0.225 m constraint radius, box never left. The robot bulges
outward by about 45 mm to clear the obstacle and rejoins the circle.

`sim` Mean tracking error 0.004 m, and 0.003 m after the first quarter of the run once the
robot is established on the circle. Radius from the origin: mean 0.802 m against a 0.800 m
reference, minimum 0.594 m (the approach from the start point) and maximum 0.845 m (the
bulge around the obstacle). Closest approach 0.224 m against the 0.225 m constraint. Zero
infeasible warnings over the whole run. This requires the soft constraint of section 7d;
with a hard one the same run deadlocks at the top of the lap.

Two numbers worth quoting because they confirm the computed path properties exactly: the
node logs a steady `v 0.084 m/s` and `omega 0.105 rad/s` while on the circle, against
0.084 m/s and 0.105 rad/s computed from the curve.

`untested` Path properties, computed from the curve: 5.027 m per lap, constant curvature
1.25 1/m, constant path speed 0.084 m/s (38 percent of the Burger's limit) and constant
path turn rate 0.105 rad/s (13 percent of the lab's 0.8 rad/s bound). The headroom is
what the controller spends detouring around the obstacle and catching back up.

The design decision worth defending: the reference reaches the controller as a horizon,
not as a point. `trajectory_node` publishes `/new_position` exactly as in Lab 1, and
additionally publishes `nav_msgs/Path` on `/reference_path` holding where the setpoint
will be at each of the next 20 steps. The MPC fills its TVP from that path.

Holding one point constant across the horizon is correct for a fixed setpoint and wrong
for a moving one: it tells the optimizer to plan every future step toward a goal it
already knows will have moved. The notebook's own dynamic-setpoint section does the same
thing, filling `tvp['_tvp', k, 'xdes']` with the reference's value at step `i + k`.

`/new_position` stays the single control entry point, so the goal marker and the rosbag
are unchanged from Lab 1, and a controller that cannot use a horizon simply ignores the
extra topic.

## 9. Design decisions, with the alternative that was rejected

| Decision | Chosen | Rejected, and why |
|---|---|---|
| Setpoint delivery | `_tvp` | A constant in the objective. Changing the goal would mean rebuilding and recompiling the optimizer, which takes seconds. |
| Solve location | Its own timer at `t_step` | Solving inside the `/odom` callback. The solver then runs at odometry rate, and the horizon covers a different span of real time on every tick. |
| Obstacle source | One YAML array per axis, inflated once | Separate lists for the constraint and the plot. The picture and the constraint drift apart silently. |
| Velocity bound | `min(lab bound, robot bound)` | Using the lab's 0.5 m/s directly. The model would predict speeds the robot cannot reach. |
| Constraint hardness | Hard, `soft_constraint=False` | Soft with a penalty. Always solvable and always smooth, but it weakens the thing the lab asks to demonstrate. |
| Obstacle hardness, task 4 | Soft, penalty 1e4 | Hard, as in tasks 2 and 3. Deadlocks permanently in Gazebo. See section 7d. |
| Solver failure | Publish zero, log the status once | Hold the last command. Under a genuinely infeasible problem that drives the robot on a stale command toward a boundary it is forbidden to cross. |
| `n_robust` | 0 | The tutorial's 1. `n_robust` builds a scenario tree over uncertain model parameters, and this model declares none, so it costs solve time and buys nothing. |

## 10. Gazebo results, all four tasks

`sim` Gazebo Sim 8.15.0, world `turtlebot3_dqn_stage1`, Burger at the origin, 2026-09-22.
One bag per run, one trajectory plot per bag, under `ros2_ws/bags/`.

| Run | Outcome | Final error | Closest approach vs constraint | Boundary |
|---|---|---|---|---|
| Task 1, goal (0.8, 0.5) | goal reached | 0.033 m | no obstacles | max abs 0.773, 0.482 of 1.0 |
| Task 1b, goal (1.5, 0.0) | infeasible, holds at boundary | 0.483 m | no obstacles | x reached 1.017 of 1.0 |
| Task 2, goal (1.5, 0.0) | goal reached | 0.028 m | 0.257 m vs 0.255 m, active | max abs 1.473, 0.181 of 2.0 |
| Task 3, goal (1.8, 0.0) | goal reached | 0.038 m | 0.256 and 0.257 m vs 0.255 m, both active | max abs 1.763, 0.111 of 2.0 |
| Task 4, circle | tracked, obstacle cleared | mean 0.004 m | 0.224 m vs 0.225 m | max abs 0.814, 0.845 of 1.5 |

Every run: commanded `v` peaked at exactly 0.220 m/s and `omega` at exactly 0.800 rad/s,
the two bounds that bind. Every `TwistStamped` carried a non-zero header stamp and
`frame_id` `base_link`, checked message by message across all five bags, which is the one
thing Gazebo will not complain about and a real robot will.

`sim` Solve time in the node, with the simulator running: 12.9 to 28.7 ms, against a
100 ms budget. Task 3, with two obstacles, is the slowest at 23 to 29 ms. The first solve
after startup is the outlier at 37.5 ms, which is warm-up rather than steady state.

## 11. A lab-day trap found while recording

`ros2 topic pub --once /new_position ...` publishes and exits immediately. If a bag
recorder subscribed to `/new_position` has not yet matched the brand new publisher,
DDS discovery loses the message, the controller still receives it, and the bag records
nothing. The first task 1 recording had a perfectly good trajectory and zero setpoints in
it, so the trajectory plot had no goal marker on it.

Use `--times 6 --rate 2` instead, or any repeated publish. The fix costs three seconds and
the failure is invisible until the plot is made.

## 12. Verification, and what it does not cover

`desk` The MPC test suite checks the constraint wiring
the constraint wiring rather than solver aesthetics: input bounds respected on every
command, the box never left, obstacle clearance at or above the inflated radius, the
horizon-reference override, and every finding in section 7 pinned down as a test so the
claims here stay true if anything changes underneath them. Three of them run a full 68 s
lap of task 4, because task 4's failure mode does not appear until 22 s in.

`sim` All four tasks launched in Gazebo, bags recorded, plots produced from the bags with
obstacle and boundary overlays drawn from the same parameter files the controller loaded.
Parameters read back off the running node with `ros2 param get`, because a unit test
cannot catch a parameter file that installed empty and only a real launch can. Obstacle
arrays report as `Parameter not set` for task 1, which is how "no obstacles" is expressed
rather than a fault. Zero stray processes after every run.

Not done, and the report must not claim otherwise:

- No physical robot run. Every number is `desk` or `sim`.
- The 0.105 m inflation is an assumption. CONFIRM BY: drive task 2 on turtle4 and measure
  the closest approach from the bag.
- The 17 mm boundary overshoot in task 1b and the 1 mm obstacle slack in task 4 are
  Gazebo's dynamics. Both will differ on hardware, probably in the direction of more
  overshoot, because a real Burger has more latency than the simulator.
- No video capture yet. `scripts/record_demo.py` records a bag and a screen capture of
  RViz together and its topic list covers the Lab 2 topics, but no recording has been
  made for these runs.
