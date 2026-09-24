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
  map inflation, and the loop that explores a maze until nothing is left to see.

## Layout

```
lab1-files/   Everything needed to build and run Lab 1. Use ros2_ws/ inside it
              directly as your workspace.
lab2-files/   The same, for Lab 2. Its own workspace, and different package names, so
              both labs can be sourced at once without shadowing each other.
lab3-files/   The same, for Lab 3. Needs the course's own r7021e_exploration package
              extracted into its src/ as well; lab3-guide.md says where from.
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

## Documents

- [lab1-guide.md](lab1-guide.md) -- every Lab 1 task, the parameter files, the control
  theory, and the lab-day checklist.
- [lab2-guide.md](lab2-guide.md) -- the same for Lab 2, plus the do-mpc setup and a
  troubleshooting section.
- [lab2-report-notes.md](lab2-report-notes.md) -- Lab 2's design decisions with the
  alternative that was rejected in each case, every measurement behind them, and the
  findings worth explaining rather than hiding. The Lab 2 report is written from this.
- [lab3-guide.md](lab3-guide.md) -- the same for Lab 3, plus the simulation mazes, the
  recording workflow and a troubleshooting section.
- [lab3-report-notes.md](lab3-report-notes.md) -- Lab 3's design decisions with the
  rejected alternative in each case, the real-time problems worth explaining rather than
  hiding, and what six simulation runs measured, including why every one of them stopped
  with frontiers still on the map. The Lab 3 report is written from this.
- [planning-notes.md](planning-notes.md) -- design decisions and the reasoning behind
  the package structure, across all three labs.

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

Lab 3 also uses two nodes from the course's own `r7021e_exploration` package, unmodified:
`frontier_detector_node` and `path_follower_node`. That package is not included here; it
is downloaded from Canvas.

## Before your lab session

Run through the lab-day checklist in the guide for the lab you are running. Message type,
domain ID, parameter loading, and stray simulator processes from a previous run. Each of
those has cost a real session before; the checklists exist so they do not cost another
one.
