# Lab 4 guide: grid-based SLAM with a Rao-Blackwellized particle filter

A TurtleBot3 Burger builds an occupancy grid of the maze and estimates its own path at the
same time, with Grid-FastSLAM 2.0. Every particle carries a pose hypothesis and its own
map. The improved proposal scan-matches each particle against its own map and samples its
new pose from a Gaussian fitted around the match.

The course supplies the package skeleton, the occupancy grid, the scan matcher, the ROS
node and the plotting code. This folder fills in the five modules the lab asks for
(motion model, measurement likelihood, improved proposal, resampling, the filter step),
tuned in simulation. It also adds a Gazebo package for the maze, offline replay tools,
and a test suite.

Everything in this guide runs in simulation first. The robot is only available during
the session.

## What Lab 4 is

Nine tasks from `R7021E_Lab_4.pdf`. Tasks 1 to 7 are the pre-lab, in simulation. Tasks 8
and 9 are the lab session on hardware.

| Task | What | Where in this folder |
|---|---|---|
| 1 | Occupancy mapping with known poses (odometry only); shipped vs tuned grid parameters | `use_measurement_update:=false`, `config/params_shipped.yaml` |
| 2 | Odometry motion model: sample, and evaluate the density | `motion_model.py` |
| 3 | Likelihood-field measurement model | `measurement_model.py` |
| 4 | Effective sample size, systematic resampling | `resampling.py` |
| 5 | Improved proposal: candidates, weighted Gaussian, eta | `proposal.py` |
| 6 | The filter step | `rbpf.py` |
| 7 | Tune, then four figures: maps, ATE vs N, N_eff, timing | recorded bag + `tools/offline_sweep.sh` + `plot_results` |
| 8 | Transfer to the real robot; two recorded runs | `config/params_hardware.yaml` |
| 9 | Wheels spinning, robot lifted | observation and explanation |

## One-time setup on a new machine

### Simulation packages

Gazebo Sim and the TurtleBot3 models, in whatever workspace you keep them:

```bash
export TURTLEBOT3_MODEL=burger
source ~/turtlebot3_ws/install/setup.bash
```

Add both lines to `~/.bashrc` so every new terminal has them. `turtlebot3_teleop` reads
`TURTLEBOT3_MODEL` and exits without it.

### Python packages

Only numpy and matplotlib, both in a standard ROS 2 Jazzy install. No pip installs.

### For recording (screen capture plus bag in one pass)

```bash
sudo apt install x11-utils gstreamer1.0-tools gstreamer1.0-plugins-base \
                 gstreamer1.0-plugins-good python3-xlib
```

Check it before the lab session, not during it:

```bash
gst-inspect-1.0 ximagesrc && gst-inspect-1.0 vp8enc && gst-inspect-1.0 webmmux
```

## Build and test

```bash
cd ~/advanced-robotics-2026/lab4-files/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

Two packages should build: `r7021e_fast_slam` (the filter) and `r7021e_fast_slam_sim`
(the Gazebo maze).

The tests take about ten seconds and need no simulator:

```bash
cd ~/advanced-robotics-2026/lab4-files/ros2_ws/src/r7021e_fast_slam
python3 -m pytest test -q
```

Expect `100 passed`. Three of them are the lab's three ordering traps: the map is
integrated last, resampling deep-copies every grid, and the logged N_eff is the value
before resampling.

## Every terminal

Each block below starts with the lines a fresh terminal needs, so you can paste a whole
block as it stands. In simulation use your own domain id, so you do not see a
neighbour's robot. On the robot use the robot's (turtle4 is 34).

```bash
export ROS_DOMAIN_ID=30
export TURTLEBOT3_MODEL=burger
source ~/advanced-robotics-2026/lab4-files/ros2_ws/install/setup.bash
```

## Pre-lab in simulation

### Step 1: record the Task 7 drive (one good loop, by teleop)

Every Task 7 figure and both Task 1 maps are replays of **one** recorded drive. Two drives
are not a controlled comparison. Record it once, carefully, and reuse it for everything.

**Terminal 1, the simulator** (the course maze, with the true pose bridged so the bag can
be checked later):

```bash
export ROS_DOMAIN_ID=30
export TURTLEBOT3_MODEL=burger
source ~/advanced-robotics-2026/lab4-files/ros2_ws/install/setup.bash
ros2 launch r7021e_fast_slam_sim maze_sim.launch.py ground_truth:=true
```

Wait for the Gazebo window with the robot in the maze.

**Terminal 2, the recorder.** Start it before you drive:

```bash
export ROS_DOMAIN_ID=30
source ~/advanced-robotics-2026/lab4-files/ros2_ws/install/setup.bash
mkdir -p ~/advanced-robotics-2026/lab4-files/ros2_ws/bags
cd ~/advanced-robotics-2026/lab4-files/ros2_ws/bags
ros2 bag record /scan /odom /tf /tf_static /clock /ground_truth -o maze_drive
```

**Terminal 3, teleop:**

```bash
export ROS_DOMAIN_ID=30
export TURTLEBOT3_MODEL=burger
source ~/advanced-robotics-2026/lab4-files/ros2_ws/install/setup.bash
ros2 run turtlebot3_teleop teleop_keyboard
```

Keep this terminal focused while you drive. The keys:

| Key | Effect |
|---|---|
| `w` / `x` | speed up / slow down, 0.01 m/s per press |
| `a` / `d` | turn left / right faster, 0.1 rad/s per press |
| `s` or space | stop |

**How to drive it**, because the drive decides whether the figures mean anything:

- **Slow and smooth.** About 15 presses of `w` (0.15 m/s). Turn with 2 to 5 presses of
  `a` or `d` **while moving**, so the robot drives arcs.
- **Never spin fast in place.** Gazebo's wheels slip during fast spins, its odometry then
  drifts from where the robot really is, and that odometry is what the node logs as ground
  truth (lab section 4.1). Exploration runs with fast spins had "ground truth" up to 1.6 m
  wrong. If you must turn around in a dead end, stop, press `a` three times (0.3 rad/s),
  and let it turn slowly.
- **Keep off the walls.** A scraping robot slips too.
- **Drive a loop through a good part of the maze and come back to the start.** Going out
  and back along the same corridors is fine; revisiting places is what tests the filter.
  Five or six minutes is plenty.

**Stop, in this order:** `s` in terminal 3, then **one** Ctrl-C in terminal 2 and **wait**
until the recorder has finished writing (it can take a minute). A second Ctrl-C truncates
the bag. Then Ctrl-C terminal 1.

Check the bag:

```bash
export ROS_DOMAIN_ID=30
source ~/advanced-robotics-2026/lab4-files/ros2_ws/install/setup.bash
cd ~/advanced-robotics-2026/lab4-files/ros2_ws/bags
ros2 bag info maze_drive
python3 ../src/r7021e_fast_slam/tools/check_ground_truth.py maze_drive
```

The second command compares Gazebo's clean odometry with the true pose over the whole
drive. It should end with `verdict: OK` (under 0.10 m). A reference loop of 46 m gave
0.048 m. If it prints `TOO FAR`, the drive slipped: record it again, more gently.

If `ros2 bag info` says it cannot find metadata, the recorder was killed rather than
stopped. Regenerate the metadata:

```bash
ros2 bag reindex -s mcap maze_drive
```

Then check for leftovers. `gz sim`'s server and GUI have not died on Ctrl-C on any run
tested:

```bash
ps aux | grep -E "[g]z sim|[g]zserver|[g]zclient|[r]viz2|[p]arameter_bridge" | grep -v grep
```

Kill anything it lists before the next launch, by PID. If you use `pkill -f` instead, keep
the brackets in the pattern (`pkill -f "[g]z sim"`): without them the pattern matches the
command line of the terminal running it, and kills that terminal.

### Step 2: watch the filter run live (Task 6)

With the simulator running (terminal 1 as above, `ground_truth:=true` optional), start the
filter and RViz in a second terminal:

```bash
export ROS_DOMAIN_ID=30
source ~/advanced-robotics-2026/lab4-files/ros2_ws/install/setup.bash
ros2 launch r7021e_fast_slam r7021e_fast_slam.launch.py use_sim_time:=true run_name:=live_demo
```

Then teleop as in step 1. RViz shows `/map` (the best particle's grid), `/particles` (the
pose cloud), `/slam_path` (the best particle's trajectory) and the `map -> odom`
correction. Ctrl-C writes `runs/live_demo.npz`.

To replay the recorded drive through the live node instead of driving:

```bash
# terminal A
export ROS_DOMAIN_ID=30
source ~/advanced-robotics-2026/lab4-files/ros2_ws/install/setup.bash
ros2 launch r7021e_fast_slam r7021e_fast_slam.launch.py use_sim_time:=true run_name:=replay_demo

# terminal B, once RViz is up
export ROS_DOMAIN_ID=30
source ~/advanced-robotics-2026/lab4-files/ros2_ws/install/setup.bash
ros2 bag play ~/advanced-robotics-2026/lab4-files/ros2_ws/bags/maze_drive --clock
```

`use_sim_time:=true` only with `--clock`. Without a clock publisher the node's clock never
moves and it publishes nothing (it warns after 5 s).

### Step 3: Task 1 maps (shipped and tuned grid parameters)

Task 1 maps with odometry only (`use_measurement_update:=false`), once with the course's
parameters exactly as shipped and once with ours. Both runs replay the same bag through
the same node, offline. The shipped grid is 5000 × 5000 cells at 1 cm, which is far too
large to publish live to RViz ten times a second.

```bash
export ROS_DOMAIN_ID=30
source ~/advanced-robotics-2026/lab4-files/ros2_ws/install/setup.bash
cd ~/advanced-robotics-2026/lab4-files/ros2_ws/src/r7021e_fast_slam
BAG=~/advanced-robotics-2026/lab4-files/ros2_ws/bags/maze_drive

python3 tools/offline_replay.py $BAG mapping_shipped --params-file config/params_shipped.yaml \
    -p use_measurement_update:=false
python3 tools/offline_replay.py $BAG mapping_tuned \
    -p use_measurement_update:=false -p "odom_noise_sigma:=[0.0,0.0,0.0]"
python3 tools/offline_replay.py $BAG mapping_noisy -p use_measurement_update:=false

python3 -m r7021e_fast_slam.plot_results --figure maps \
    --runs runs/mapping_shipped.npz runs/mapping_tuned.npz \
    --labels "as shipped" "tuned" --out figures/task1
```

The shipped run takes about two and a half minutes. It produces a **blank** map, which is
the point: `log_odds_limit: 0.0` with clamping on pins every cell at p = 0.5.
`mapping_noisy` is the odometry-only map with the simulated odometry noise on; Task 7
figure 1 sets it against the filter.

### Step 4: the Task 7 sweep (figure 2 and figure 4)

The lab's `tools/run_sweep.sh` replays the bag in real time. The node keeps only the
newest scan, so once one filter step takes longer than the 0.2 s between scans (from
about N = 20 on a desktop, sooner under load) it skips scans. A test replay under load
showed FS2 at N = 5 processing 877 steps against FS1's 1258 from the same bag. Different
input, so not a fair comparison.

`tools/offline_sweep.sh` runs the same combinations with the same run names through
`tools/offline_replay.py`, which feeds every scan through the same node code:

```bash
export ROS_DOMAIN_ID=30
source ~/advanced-robotics-2026/lab4-files/ros2_ws/install/setup.bash
cd ~/advanced-robotics-2026/lab4-files/ros2_ws/src/r7021e_fast_slam
tools/offline_sweep.sh ~/advanced-robotics-2026/lab4-files/ros2_ws/bags/maze_drive "1 5 10 20 50" "1 2 3" 4
```

The last argument is how many runs execute in parallel. Thirty runs; the N = 50 ones take
longest (an hour or more each on a busy machine). Leave it running.

If you want the lab's real-time script anyway:

```bash
tools/run_sweep.sh ~/advanced-robotics-2026/lab4-files/ros2_ws/bags/maze_drive "1 5 10 20" "1 2 3"
```

### Step 5: figure 3 (the two resampling policies)

Same bag, same N, same seed, resampling selectively (the default) and at every step:

```bash
export ROS_DOMAIN_ID=30
source ~/advanced-robotics-2026/lab4-files/ros2_ws/install/setup.bash
cd ~/advanced-robotics-2026/lab4-files/ros2_ws/src/r7021e_fast_slam
BAG=~/advanced-robotics-2026/lab4-files/ros2_ws/bags/maze_drive
python3 tools/offline_replay.py $BAG neff_selective -p num_particles:=10 -p seed:=1
python3 tools/offline_replay.py $BAG neff_every -p num_particles:=10 -p seed:=1 -p resample_every_step:=true
```

### Step 6: the four figures

```bash
export ROS_DOMAIN_ID=30
source ~/advanced-robotics-2026/lab4-files/ros2_ws/install/setup.bash
cd ~/advanced-robotics-2026/lab4-files/ros2_ws/src/r7021e_fast_slam

# 1: odometry-only map (with the simulated noise) next to the filter's map
python3 -m r7021e_fast_slam.plot_results --figure maps \
    --runs runs/mapping_noisy.npz runs/fs2_n10_s1.npz \
    --labels "odometry only" "Grid-FastSLAM 2.0, N=10" --out figures

# 2: ATE against N, both proposals, all seeds
python3 -m r7021e_fast_slam.plot_results --figure ate --runs runs/fs*.npz --out figures

# 3: the two resampling policies
python3 -m r7021e_fast_slam.plot_results --figure neff \
    --runs runs/neff_selective.npz runs/neff_every.npz \
    --labels selective "every step" --out figures

# 4: time per stage and map memory against N (FS2, seed 1)
python3 -m r7021e_fast_slam.plot_results --figure timing --runs runs/fs2_n*_s1.npz --out figures
```

The figures land in `figures/` inside the package: `maps.png`, `ate_vs_particles.png`,
`neff.png`, `timing.png`.

## Lab session on the robot

### Before you go

- Build clean, tests pass (above).
- The recording packages are installed (the `gst-inspect` line above).
- You know the robot's number and domain id.

### Bring the robot up

Terminal 1, on the robot over ssh:

```bash
ssh turtle@192.168.50.40
export ROS_DOMAIN_ID=34
ros2 launch turtlebot3_bringup robot.launch.py
```

Replace `40` and `34` with your robot's address and domain id; the password is the one
the lab hands out. Leave it running.

Terminal 2, on your laptop, check the robot is visible before anything else:

```bash
export ROS_DOMAIN_ID=34
ros2 topic hz /scan
ros2 topic hz /odom
```

`/scan` should be about 5 Hz. If either shows nothing, fix that first: nothing downstream
can work without them.

### Run the filter on the robot

```bash
export ROS_DOMAIN_ID=34
source ~/advanced-robotics-2026/lab4-files/ros2_ws/install/setup.bash
ros2 launch r7021e_fast_slam r7021e_fast_slam.launch.py use_sim_time:=false \
    params_file:=$HOME/advanced-robotics-2026/lab4-files/ros2_ws/src/r7021e_fast_slam/config/params_hardware.yaml \
    run_name:=hw_baseline
```

`params_hardware.yaml` differs from the simulation file in three lines: no injected
odometry noise (real odometry is noisy enough), wider `odometry_sigmas`
([0.03, 0.04, 0.03], real wheels slip more), and the run name. `use_sim_time:=false`:
on the robot the clock is real.

Check the parameters really loaded, in another terminal:

```bash
export ROS_DOMAIN_ID=34
ros2 param get /grid_slam_node odom_noise_sigma
ros2 param get /grid_slam_node num_particles
```

Expect `[0.0, 0.0, 0.0]` and `10`.

Drive with teleop (terminal 3, same `ROS_DOMAIN_ID=34`, `ros2 run turtlebot3_teleop
teleop_keyboard`). Drive gently, as in simulation.

### Record every run (bag and RViz video together)

Start the filter with RViz (as above), then in its own terminal:

```bash
export ROS_DOMAIN_ID=34
source ~/advanced-robotics-2026/lab4-files/ros2_ws/install/setup.bash
cd ~/advanced-robotics-2026/lab4-files
python3 scripts/record_demo.py ros2_ws/bags/hw_baseline
```

It records the bag to `ros2_ws/bags/hw_baseline/` and the RViz window to
`ros2_ws/bags/hw_baseline.webm`, and stops both on **one** Ctrl-C. Wait for it to print
both file sizes. Check the bag right after:

```bash
ros2 bag info ~/advanced-robotics-2026/lab4-files/ros2_ws/bags/hw_baseline
```

### The runs Task 8 asks for

1. **Baseline**: the full maze with the best configuration, returning to the start.
   `run_name:=hw_baseline`, recorded as `hw_baseline`.
2. **Two loops**: the maze twice, returning to the start. `run_name:=hw_two_loops`,
   recorded as `hw_two_loops`.

Hand in the bags, the RViz videos, and `config/params_hardware.yaml` as it was for these
runs. If you change a parameter on the robot, write down which one, from what to what,
and the symptom that made you change it. That is a result for the report.

### Tuning on the robot, if needed

| Symptom | Change | Why |
|---|---|---|
| Spurious walls, speckle in free space | `p_occ` 0.7 → 0.57 to 0.60 | one hit then no longer marks a cell occupied; it takes two |
| Particle cloud stays wide long after the map is good | `odometry_sigmas` down, e.g. [0.02, 0.02, 0.02] | the motion model is wider than the real error |
| Estimate cannot follow a turn, map tears at corners | `odometry_sigmas` up, e.g. [0.04, 0.05, 0.04] | the truth falls outside the motion prior |
| The node falls behind (RViz lags, few steps) | `num_particles` 10 → 5 | a step at N = 10 takes about 115 ms of the 200 ms budget on a desktop |

To change one without editing the file, copy it first:

```bash
cp ~/advanced-robotics-2026/lab4-files/ros2_ws/src/r7021e_fast_slam/config/params_hardware.yaml \
   ~/advanced-robotics-2026/lab4-files/ros2_ws/src/r7021e_fast_slam/config/params_hardware_session.yaml
```

edit the copy, and pass it as `params_file:=`.

### Task 9: lifted wheels

Put the robot on a support so the wheels turn freely without it moving. Run the filter
and record, exactly as above, `run_name:=hw_lifted`. Drive forward gently with teleop for
10 to 20 s, then stop. Watch the robot icon, the particle cloud and the map in RViz.

What to expect (from a simulation of the same situation): the estimate **creeps** forward
much more slowly than the wheels suggest, about a third of the claimed speed. The particle
cloud stays tight, and walls start to thicken or double. The prediction step believes the
wheels. Scan matching and the improved proposal pull the pose back, because the laser
keeps seeing the same walls. The pull-back only works down to one map cell, and each step
draws the frozen scan into the map at the crept pose, so the map drifts along with it.

## Topics

| Topic | Type | From |
|---|---|---|
| `/scan` | LaserScan | robot / Gazebo |
| `/odom` | Odometry | robot / Gazebo |
| `/map` | OccupancyGrid | the best particle's grid, 10 Hz |
| `/particles` | PoseArray | every particle's pose |
| `/best_particle` | Odometry | the best particle, with the cloud's covariance |
| `/slam_path` | Path | the best particle's trajectory |
| `/tf` `map -> odom` | | the filter's correction to odometry |
| `/ground_truth` | TFMessage | simulation only, `ground_truth:=true` |

## Parameters worth knowing

All in `config/params.yaml` (simulation) and `config/params_hardware.yaml` (robot).

| Parameter | Value | What it does |
|---|---|---|
| `num_particles` | 10 | largest N that keeps up with the 5 Hz scanner |
| `use_improved_proposal` | true | false runs FastSLAM 1.0, the baseline |
| `use_measurement_update` | true | false maps with odometry only (Task 1) |
| `odometry_sigmas` | [0.02, 0.02, 0.02] sim | the filter's belief about odometry noise per step; also sets the scan matcher's pull toward odometry |
| `odom_noise_sigma` | [0.003]×3 sim, zeros on the robot | noise the node adds to Gazebo's odometry, per message |
| `p_free`, `p_occ` | 0.4, 0.7 | evidence per beam: -0.405 and +0.847 in log-odds |
| `log_odds_limit` | 5.0 | clamp; must exceed logit(occ_thresh) = 0.405 |
| `hit_window_cells` | 2 | thickness of the occupied band at a beam's end |
| `map_resolution`, `map_size_m` | 0.05, 20 | 400 × 400 cells, 640 KB per particle, centred on the start |
| `sigma_hit`, `max_dist` | 0.05, 0.15 | likelihood sharpness (one cell) and clip (3 sigma) |
| `scan_match_window_xy/theta` | 0.20, 0.10 | matcher searches ±0.20 m and ±0.10 rad |
| `match_score_min` | 0.55 | an unexplored map scores 0.5 everywhere; below this the particle takes a FastSLAM 1.0 step |
| `num_candidates` | 27 | proposal lattice 3 × 3 × 3 |
| `proposal_window_xy/theta` | 0.10, 0.05 | FULL width: the lattice spans ±0.05 m and ±0.025 rad |
| `resample_threshold` | 0.5 | resample when N_eff < 0.5 N |
| `update_max_rate` | 10 | at the scan rate (5), stamp jitter makes the gate reject most scans |

The filter refuses to start on values that would run but produce a blank map or identical
particles (for example `p_occ <= 0.5`, `log_odds_limit` too small for the threshold, a
non-cube `num_candidates`), and prints why.

## If something goes wrong

| Symptom | Cause | Fix |
|---|---|---|
| RViz shows nothing, node warns "nothing publishes /clock" | `use_sim_time:=true` without a clock | play the bag with `--clock`, or launch with `use_sim_time:=false` on the robot |
| `ValueError: params.yaml: ...` at startup | a startup check caught a bad value | read the message; it names the value and why it cannot work |
| Map blank | clamp or evidence values wrong | `log_odds_limit > 0.405`, `p_occ > 0.5`, `hit_window_cells > 0` |
| `ros2 bag info`: no metadata | recorder killed, not stopped | `ros2 bag reindex -s mcap <bag>` |
| Next launch fails, or two robots appear | a previous `gz sim` still running | the `ps aux` line above, then kill what it lists |
| ATE figure looks wrong for every configuration | the drive slipped, so the ground truth is off | `tools/check_ground_truth.py <bag>`; re-record gently if `TOO FAR` |
| Offline replay prints `bag has no ['/scan']` | wrong bag directory | pass the directory that holds `metadata.yaml` |
