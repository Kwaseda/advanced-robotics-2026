# R7021E -- Submission

Lab work for R7021E on a TurtleBot3 Burger. Each lab is a self-contained subfolder with
its own ROS 2 workspace, so you can download one, build it, and run it without needing
the other.

- **Lab 1** -- closed-loop position control (Near Identity Diffeomorphism), a
  figure-of-eight trajectory generator, closest-wall-point detection from the laser scan,
  and wall following until a loop closes.
- **Lab 2** -- Model Predictive Control: setpoint tracking under input and boundary
  constraints, static obstacle avoidance, and circular trajectory tracking with an
  obstacle.
- **Lab 3** -- autonomous exploration: an RRT* planner written from scratch on the live
  SLAM map, frontier-based goal selection with an information gain, obstacle avoidance by
  map inflation plus a reactive check in the follower, and the loop that explores a maze
  until nothing is left to see. In simulation it explores the full 7.3 m maze completely
  in 11 to 13 minutes.
- **Lab 4** -- grid-based SLAM with a Rao-Blackwellized particle filter (Grid-FastSLAM
  2.0): the odometry motion model, a likelihood-field measurement model, the improved
  proposal built from each particle's own scan match, systematic resampling, and the
  filter step, inside the course's package. Plus a Gazebo maze package and tools to
  replay one recorded drive through every configuration.

## Layout

```
lab1-files/   Everything needed to build and run Lab 1. Use ros2_ws/ inside it
              directly as your workspace.
lab2-files/   The same, for Lab 2. Its own workspace, and different package names, so
              both labs can be sourced at once without shadowing each other.
lab3-files/   The same, for Lab 3. Needs the course's own r7021e_exploration package
              extracted into its src/ as well; lab3-guide.md says where from.
lab4-files/   The same, for Lab 4. Self-contained: the course's r7021e_fast_slam package
              with our modules filled in, and a Gazebo maze package.
documents/    This file, a guide per lab, and the design notes.
```

## Quick start

Lab 1:

```bash
cd lab1-files/ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch r7021e_bringup lab1.launch.py sim:=true rviz:=true
```

Lab 2 needs two pip packages first. Pin casadi exactly; `lab2-guide.md` explains why:

```bash
pip install --user --break-system-packages "casadi==3.7.2" "do-mpc==5.1.1"
cd lab2-files/ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch r7021e_mpc_bringup lab2.launch.py task:=1 sim:=true rviz:=true
```

Lab 3 needs the course's `r7021e_exploration` package and slam-toolbox first:

```bash
sudo apt install ros-jazzy-slam-toolbox
# extract r7021e_exploration from the Canvas zip into lab3-files/ros2_ws/src/
cd lab3-files/ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch r7021e_rrt_bringup lab3.launch.py sim:=true rviz:=true
```

Lab 4, the simulator in one terminal and the filter with RViz in another:

```bash
cd lab4-files/ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch r7021e_fast_slam_sim maze_sim.launch.py
```

```bash
source lab4-files/ros2_ws/install/setup.bash
ros2 launch r7021e_fast_slam r7021e_fast_slam.launch.py use_sim_time:=true
```

## Documents

- [lab1-guide.md](lab1-guide.md) -- every Lab 1 task, the parameter files, the control
  theory, and the lab-day checklist.
- [lab2-guide.md](lab2-guide.md) -- the same for Lab 2, plus the do-mpc setup and a
  troubleshooting section.
- [lab2-report-notes.md](lab2-report-notes.md) -- Lab 2's design decisions with the
  alternative that was rejected in each case, every measurement behind them, and the
  findings worth explaining rather than hiding. The Lab 2 report is written from this.
- [lab3-guide.md](lab3-guide.md) -- the same for Lab 3, plus the simulation mazes, what a
  good run looks like, the recording workflow and a troubleshooting section.
- [lab3-report-notes.md](lab3-report-notes.md) -- Lab 3's design decisions with the
  rejected alternative in each case, the real-time problems worth explaining rather than
  hiding, the four faults that stopped runs short of the whole maze and how each was
  found, and the final results. The Lab 3 report is written from this.
- [lab4-guide.md](lab4-guide.md) -- the same for Lab 4: recording the Task 7 drive by
  teleop, every figure command, the robot session, and a troubleshooting section.
- [lab4-report-notes.md](lab4-report-notes.md) -- Lab 4's design decisions, the tuning
  measurements behind every parameter, the three bugs that only running found, and the
  results. The Lab 4 report is written from this.
- [planning-notes.md](planning-notes.md) -- design decisions and the reasoning behind
  the package structure, across the labs.

## Packages

Lab 1:

- **r7021e_control** -- the four Lab 1 nodes (`controller_node`, `trajectory_node`,
  `scan_monitor_node`, `wall_follower_node`) and the control laws behind them.
- **r7021e_bringup** -- `lab1.launch.py`, the RViz configuration, and the parameter
  files under its own `config/`.

Lab 2:

- **r7021e_mpc** -- `mpc_node`, the MPC control law behind the same `Controller`
  interface Lab 1 defined, plus the trajectory generator and the goal marker.
- **r7021e_mpc_bringup** -- `lab2.launch.py`, the RViz configuration, and the parameter
  files under its own `config/`.

Lab 3:

- **r7021e_rrt** -- `navigation_node`, plus the RRT* planner, the frontier clustering and
  information gain, and the shared occupancy grid representation they both use. The three
  algorithm modules import no ROS and are unit tested at a desk.
- **r7021e_rrt_bringup** -- `lab3.launch.py`, `maze_world.launch.py`, the RViz
  configuration, the parameter files, and the two generated Gazebo mazes.

Lab 4:

- **r7021e_fast_slam** -- the course's package with the five Lab 4 modules implemented
  (`motion_model`, `measurement_model`, `proposal`, `resampling`, `rbpf`), a test suite,
  `params.yaml` for simulation and `params_hardware.yaml` for the robot, and tools to
  replay a bag offline, run the Task 7 sweep, and check a bag's ground truth.
- **r7021e_fast_slam_sim** -- `maze_sim.launch.py`: the course maze in Gazebo, the Burger
  in it, and an optional bridge of the simulator's true pose to `/ground_truth`.

Lab 3 also uses two nodes from the course's own `r7021e_exploration` package, unmodified:
`frontier_detector_node` and `path_follower_node`. That package is not included here; it
is downloaded from Canvas.

## Before your lab session

Run through the lab-day checklist in the guide for the lab you are running. Message type,
domain ID, parameter loading, and stray simulator processes from a previous run. Each of
those has cost a real session before; the checklists exist so they do not cost another
one.
