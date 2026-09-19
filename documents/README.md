# R7021E Lab 1 -- Submission

Closed-loop position control (Near Identity Diffeomorphism), a figure-of-eight
trajectory generator, closest-wall-point detection from the laser scan, and wall
following until a loop closes, on a TurtleBot3 Burger.

## Layout

```
lab-files/    Everything needed to build and run Lab 1. Copy or clone this whole
              directory into your ROS 2 workspace (or use ros2_ws/ inside it
              directly as your workspace).
documents/    This file, the full lab guide, and the design notes.
```

## Quick start

```bash
cd lab-files/ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch r7021e_bringup lab1.launch.py sim:=true rviz:=true
```

Full instructions for every task, the parameter files, the control theory, and the
lab-day checklist are in [lab1-guide.md](lab1-guide.md). Design decisions and the
reasoning behind the package structure are in
[planning-notes.md](planning-notes.md).

## Packages

- **r7021e_control** -- the four Lab 1 nodes (`controller_node`, `trajectory_node`,
  `scan_monitor_node`, `wall_follower_node`) and the control laws behind them.
- **r7021e_bringup** -- `lab1.launch.py`, the RViz configuration, and the parameter
  files under its own `config/`.

## Before your lab session

Run through the "Critical Lab Day Checklist" section of
[lab1-guide.md](lab1-guide.md) -- message type, domain ID, parameter loading, and
(in simulation) stray processes from a previous run. Each of those has cost a real
session before; the checklist exists so it doesn't cost another one.
