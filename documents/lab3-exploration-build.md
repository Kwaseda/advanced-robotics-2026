# 2026-09-23, Lab 3: the exploration stack built, and the thirteen things only running it found

Follows `2026-09-23-lab2-recording-verified-and-lab3-handoff.md`. This is the Lab 3 build
session: the skeletons filled in, the node written, the launch and simulation stood up,
and the run-it-do-not-reason-about-it rule earning its place a dozen times over.

Append-only. Nothing here is edited later; corrections go in the next entry.

## What was decided before any code

Grilled first, as agreed. Six questions, each with a recommendation attached. The answers:

1. **I(p) is the reduced LiDAR range form**, frontier cells within `r_info` of the
   candidate goal counted across the whole frontier grid, with plain cluster cell count
   kept behind a YAML switch so the report carries a measured comparison rather than an
   assertion. This settles the inconsistency the handoff flagged between the plan draft
   and the skeleton, in favour of the plan draft and of the lab instructions' own tip.
2. **Unknown cells are traversable, and not inflated**, bounded by `unknown_lookahead`.
   Every frontier goal sits on the boundary of unknown space, so a planner that refuses
   unknown cells cannot reach a single goal the gain ever picks.
3. **A 1 Hz timer**, replanning on arrival, stall or staleness. Not the map callback.
4. **The provided path follower, unmodified**, and one publisher on `cmd_vel`. The first
   answer to this was "publish zero cmd_vel ourselves", which was reconsidered on the
   grounds that the follower publishes on its own 10 Hz timer forever and the two would
   interleave rather than one winning. Termination is a park path at the robot's pose.
5. **0.105 m inflation**, matching Lab 2's footprint number, plus a sweep for the report.
6. **Frontier-based, not next-best-view.** Reopened mid-session by a transcript from
   another group who chose next-best-view, and closed again: the Canvas plan commits to
   frontier-based in writing and that group's choice does not bind this one. One idea was
   taken from it, `max_empty_cycles`, terminating after several empty cycles rather than
   the first.

## What was built

| File | What it is |
|---|---|
| `r7021e_rrt/grid.py` | The one grid representation. `OccupancyMap` plus `PlanningGrid`, which holds the inflation. New: the skeletons assumed a representation and did not define one. |
| `r7021e_rrt/rrt_star.py` | The planner. Skeleton method names kept. |
| `r7021e_rrt/exploration_gain.py` | Clustering, goal points, I(p), H, selection. Skeleton function names kept. |
| `r7021e_rrt/navigation_node.py` | The loop. The only file importing rclpy. |
| `r7021e_bringup/launch/lab3.launch.py` | One launch file, arguments for everything that changes between a desk and a bench. |
| `r7021e_bringup/launch/maze_world.launch.py` | Gazebo side, separable so teardown is checkable. |
| `config/lab3.yaml`, `config/slam_lab3.yaml` | Every number, with the reason next to it. |
| `scripts/plot_exploration.py` | New. `plot_trajectory.py` has no concept of an occupancy grid. |

89 tests, all passing, none of them importing ROS.

Three deliberate deviations from the skeleton, each written into the module docstring:
the tree is an object rather than a bare list so the exhaustive scans run in numpy
(measured: 0.9 s per plan in a Python loop against 0.06 s, and six candidates per second
does not fit in the first); nodes cache their cost and their children, because rewiring
asks for both in the inner loop; and `plan()` keeps growing after the first branch reaches
the goal, per slide 35, rather than returning it.

## The thirteen things that only running it found

Ordered as they were found. Every one of them passed the unit tests.

### 1. A QoS mismatch, silent by design

`slam_toolbox` publishes `/map` TRANSIENT_LOCAL. The course's `frontier_detector_node`
creates its publisher with a bare `QoSProfile(depth=1)`, which is VOLATILE. Subscribing to
both with the map's profile means the frontier subscription matches no publisher and
receives nothing; DDS does not fall back. Two profiles now, and the comment in
`navigation_node.py` quotes the exact warning text both sides log, since recognising that
wording is what makes this a one-run bug instead of an evening.

Evidence tier: `sim`, lab3_maze_small, 2026-09-23.

### 2. Termination burning its fuse during startup

`max_empty_cycles: 5` at 1 Hz is a five second fuse, and the node starts before SLAM has
published anything. First Gazebo run: "exploration complete" twenty seconds after launch,
two goals visited. Termination is now gated on the node having chosen a goal at least
once, and the bound is 10 rather than 5.

### 3. The frontier count dips and recovers

Measured with a probe node subscribing alongside: on lab3_maze_small the frontier cell
count fell from 206 to 27 and recovered within a few seconds as SLAM redrew the map. 27
scattered cells contain no run of five connected ones, so zero clusters is a legitimate
mid-run reading. This is why the bound moved to 10 rather than staying at 5 with only the
startup gate added.

### 4. Diagonal frontiers are invisible to 4-connected clustering

The expensive one. The second full run ended with about ten frontier cells plainly visible
in RViz, stepping diagonally across the lower left, and the node correctly reported zero
clusters: under 4-connectivity a staircase is ten clusters of one cell, all below the
5-cell noise floor. Coverage at the moment it declared success was 65 percent of the
mapped area with the entire right half of the maze never visited.

The clustering is now 8-connected, and the reasoning is worth keeping: the extractor's own
4-neighbourhood answers "is this free cell adjacent to unknown", a question about one
cell. Grouping frontier cells into boundaries is a different question, and the boundary of
what a rotating LiDAR has seen is a curve, which on a grid is a staircase. Diagonal
frontiers are the normal case, not an edge case.

This is the bug that looks most like correct behaviour in a log, and the only thing that
caught it was rendering the map.

### 5. Arrive, reselect, arrive

With clustering fixed, the robot reached a frontier it could not clear from where it
stood, then chose it again: thirty-one cycles of moving a few centimetres back and forth,
every one logged `[held]`. Two causes. The commitment hysteresis was surviving arrival,
which it has no business doing, since it exists to stop thrash *while driving*. And with
the commitment released the same goal still won on merit, because H = sum(d) - w I
contains nothing that knows where the robot has been.

Fixed with a retirement list: a goal arrived at, or stalled on, is never offered again,
nor is any candidate within 0.30 m of it. The next run retired 9 goals and covered the
whole maze extent rather than half of it.

### 6. A wedged robot corrupts the map, and the chain is longer than it looks

The run after that produced a map spanning -3.2 to +4.5 m for a 4 m maze, with diagonal
smears. The chain: the planner returns waypoints 0.30 m apart, the follower drops a
waypoint as soon as it is within its 0.20 m look-ahead and steers at the next one, so
across a corner it aims through the corner and drives the chord rather than the checked
segments. In a 0.7 m corridor that chord is enough to wedge the robot. A wedged robot
spins its wheels, odometry runs away from the truth, and `correlation_search_space_
dimension` is 0.5 m, so once the odom error exceeds that the scan match fails and the map
smears. A corrupted map is not a recoverable state for an exploration run.

Fixed on our side, not by touching the follower: the published path is resampled to 0.10 m,
half the look-ahead, so there is always a waypoint inside the look-ahead that lies on the
path the planner actually verified.

### 7. Two publishers on /clock, which I added myself

Densification removed the stalls (sixteen to zero) and the map was still smeared, spanning
-3.2 to +4.5 m for a 4 m maze. The cause was in my own launch file: `maze_world.launch.py`
started a `parameter_bridge` for `/clock`, with a comment asserting that bridging it twice
was harmless because "the bridge deduplicates by topic". It does not.
`spawn_turtlebot3.launch.py` already bridges clock via `turtlebot3_burger_bridge.yaml`, so
there were two publishers, each with its own view of simulator time, and a node on sim
time saw the clock step backwards whenever the later message came from the other one. tf
said so plainly, eight times per run: "Detected jump back in time. Clearing TF buffer."

Removing my bridge took the count to zero jumps and the map from sprawling to roughly
maze-sized.

### 8. The scan interval against the turn rate

With the clock fixed the map was still wrong, but wrong in a legible way: its first half
clean and rectilinear, its second half the same maze rotated about twenty degrees and
smeared. That is angular divergence, and the arithmetic is a straight comparison of two
numbers already in the configuration.

The path follower turns at up to `max_w` = 1.0 rad/s. slam_toolbox's
`minimum_time_interval` was 0.5 s, so the robot could rotate 0.5 rad between two accepted
scans. Its coarse angular search is `coarse_search_angle_offset` = 0.349 rad. The search
window was smaller than the rotation it had to recover, so a fast turn simply failed to
match and the pose fell back on odometry.

At 0.1 s the limit becomes the scan rate itself, about 5 Hz, so the worst case is 0.2 rad
between scans. The map came out rectilinear and recognisably the maze, 8463 cells against
22348 for the same maze before, and the run terminated with 4 clusters outstanding rather
than 17.

### 9. Speckle, inflated

The 7.2 m maze run terminated after 24 cycles having explored 10.4 m2 of roughly 40, and
its log said the right things: 7 clusters found, 6 unreachable, ten empty cycles, stop.
The map figure said something else, with four red unreachable markers sitting in open
white space.

Offline against the final map from the bag, with the planner given 20000 iterations and
2 s instead of 1500 and 0.15 s, still 0 of 6. So not a budget problem. A flood fill from
the robot's own cell reached exactly one cell: its own.

Printing the raw occupancy around the robot explained it. The raw map there is almost
entirely free with a scatter of isolated occupied cells through it. Inflated by a 3-cell
disc, each of those becomes a 7 by 7 blocked blob, and a region with them two cells apart
has no traversable cell left in it at all. The planner was individually correct about
every answer it gave.

`PlanningGrid` now despeckles before inflating: an occupied cell with no occupied
neighbour at all in the 8-neighbourhood is dropped as a spurious return. Walls are
continuous, so a real wall cell always has one, and that threshold is the most
conservative version of this that does anything.

### 10. Despeckling was not enough, and the reason is worth stating

With despeckling on, the flood fill from the robot still reached one cell. The scatter
there is not isolated singles but a diffuse cloud about two cells apart, so the pairs
survive and the collar still seals the region. At every inflation tried down to 0.05 m,
the robot's free component was 9 to 163 cells against 9000 to 11000 traversable cells on
the same map.

Two things came out of chasing that.

`PlanningGrid.nearest_unblocked()` and rooting the tree away from the robot's pose. A
start that is not traversable is not rare and not an error: a 0.7 m maze corridor is
narrower than twice the 0.15 m collar plus the robot, so driving down the middle of one
already means standing inside the collar. Where the existing escape-prefix mechanism
cannot reach free space, the tree is now rooted at the nearest genuinely traversable point
and the robot's own pose is put back on the front of the path.

And a bug in my own first version of that, worth recording because it is the same shape as
several others here: `_crosses_occupied` sampled from the robot's own cell outward, and on
a noisy map the robot's own cell is sometimes marked occupied. Every candidate was
therefore rejected and the search returned nothing while reporting no error. The robot has
demonstrably survived being where it is; the sample at the start point is skipped.

### 11. An unstick manoeuvre, because a stationary robot cannot map its way out

Even rooted elsewhere, the robot's local free pocket is disconnected from the rest of the
map, so nothing outside it is reachable. No planner change repairs that, because the
phantom walls only disappear when new scans contradict them, and a robot that is not
moving produces no new scans from anywhere else.

So `navigation_node` now recognises the state rather than concluding from it: clusters are
being found, none of them is reachable, and the robot is standing still. That is not "the
maze is explored", and it now publishes a short path to the nearest genuinely traversable
point instead of counting toward termination. Bounded at four consecutive attempts, reset
by any successful cycle, because a robot that truly cannot move should stop rather than
twitch.

### 12. A test that was not a desk test

`test_navigation_helpers.py` imported `navigation_node` to reach `_densify`, which pulled
in rclpy and quietly made the suite depend on a sourced ROS installation, against the one
architectural property this package claims. `densify_path` moved to `rrt_star.py`, where
it belongs anyway as an operation on paths, and the test file imports no ROS.

### 13. The recovery that recovered nothing

The unstick manoeuvre worked once and then degenerated, and the log says it plainly:

```
Unsticking 0.55 m to (0.80, 0.56), attempt 1/4
Unsticking 0.15 m to (0.90, 1.01), attempt 1/4
Unsticking 0.08 m to (0.90, 1.01), attempt 2/4
Unsticking 0.03 m to (0.90, 1.01), attempt 3/4
Unsticking 0.01 m to (0.90, 1.01), attempt 4/4
```

It drove to the *nearest* traversable point, which is right for rooting a plan and wrong
for this. Having arrived beside that point, the next attempt is the distance from where it
now stands to the same point, so the manoeuvre shrinks to nothing and all four attempts
are spent without the robot going anywhere.

`farthest_unblocked()` is the counterpart, and `unstick_min_distance` is the floor below
which there is nothing useful to move to and the cycle counts toward termination instead.
Two selectors over one candidate scan, and the docstring on each says which job it is for,
because the difference is not obvious from the names alone.

## Two mistakes of my own, recorded because they cost runs

**Sampling inside a box around start and goal.** A standard RRT refinement and exactly
wrong for a maze, where the path is the opposite of direct. The desk demo at the bottom of
`rrt_star.py` caught it: a room whose only gap sits at x = 3.0, with start and goal both
at x = 0.5, returns no path at all under a 1.5 m margin. Sampling is over the whole map,
and the argument survives only as an optional parameter.

**Plotting /odom over a map drawn in the map frame.** `/odom` lives in the odom frame, and
slam_toolbox exists because that frame drifts. Drawing it over the map subtracts nothing
and plots the drift as motion: the first maze figure showed a trajectory leaving a sealed
maze and wandering two metres into open ground, which reads as a robot driving through a
wall and is not. `plot_exploration.py` now composes map to base_link from `/tf`.

And one piece of reasoning that was simply wrong, reverted with the reasoning corrected in
place: `max_laser_range` was lowered from the course's 20.0 to the Burger's real 3.5 m on
the argument that trusting beams to 20 m marks space free on measurements that do not
exist. That is backwards. The parameter clips how far a ray is traced when rastering; at
20 m with a 3.5 m sensor it is a no-op, while setting it exactly at the sensor maximum
lets no-return beams raster as though they struck something. Back to 20.0.

## Environment notes worth not rediscovering

- The teardown script's `pkill -f "[n]avigation_node"` killed an unrelated Python process
  that happened to have that string in its own command line, mid-edit. The bracket trick
  stops the pattern matching the shell running it; it does nothing about a different
  process that legitimately contains the name. Scope the pattern, or check what it matches
  before sending a signal.
- `lab3.launch.py` sets `ROS_DOMAIN_ID` for the nodes it starts, so a `ros2 topic echo` or
  a probe script in another shell sees nothing until it exports the same domain. Half an
  hour went into "the simulator is not producing scans" before that was the answer.
- `ros2 bag record` writes `metadata.yaml` on clean shutdown only. A terminated recorder
  leaves an `.mcap` that rosbag2 refuses to open. `ros2 bag reindex -s mcap <bag>`
  regenerates it, and the run script now does that automatically.
- `ros2 bag record --use-sim-time` waits for `/clock` before starting and, in these runs,
  never started at all: an 8 MB file with no messages. Recording without the flag works.
- The maze generator writes to a hardcoded `~/ros2_ws/src/turtlebot3_simulations/...`
  path. The two worlds here were generated with that line pointed at an environment
  variable instead; the commands are in `maze_world.launch.py`'s docstring.
- Both generated mazes put a free cell at the world origin, deliberately. The first 9 by 9
  attempt used an origin of -3.2 and put a wall through the spawn point.
- The course package is installed at `~/ros2_ws/src/r7021e_exploration`, copied from the
  Canvas zip exactly as the lab instructions say, and not into this repository's
  `ros2_ws/src`. `reference/NOTICE.md` still holds.

## The final six runs, and the question they answered

Run 2026-09-24 on the build that came out of everything above: three small-maze runs at
360 s with the shipped reduced-range gain, two more with `gain.mode: cluster_size`, one
full-maze run at 720 s. Numbers are in section 7 of the report source and are not repeated
here. Two things are worth keeping in the log rather than the report.

**The gain comparison did not produce a result, and the reason is arithmetic.** Reduced
range reached 9.58, 10.44 and 12.52 m2; cluster size reached 9.92 and 9.84. The means
differ by 0.97 m2 and the reduced-range spread alone is 2.94 m2. The earlier inflation
sweep had already shown why this was likely: 0.105 m and 0.13 m both `ceil()` to 3 cells,
so those two runs enforced an identical collar and differed only by seed, and they came
out 9.80 and 8.40 m2. Run-to-run variance in this setup is about 1.5 m2 and no comparison
at n of 3 can see past it. Designing the sweep before measuring the variance was the
mistake; the number of runs needed follows from the variance, not from what fits in an
evening.

**The unreachable clusters are genuinely unreachable, and there are two separate causes.**
This was the open question at the end of the last entry. The test: take each run's final
map, rebuild the planning grid at several inflation radii, and flood fill from the parking
pose under the planner's own rules, which means carrying the consecutive-unknown distance
and capping it at `unknown_lookahead`. The first version of this test did not carry the
run length, connected the robot to the whole outside of the maze through one doorway of
unknown cells, and reported every frontier reachable. A connectivity test on a planner that
does not allow free connectivity answers a question nobody asked.

With the cap in place: in four of the six runs no remaining frontier was reachable even
with the collar removed entirely, the reachable set at zero inflation being 98, 152, 475
and 2136 cells. Those runs were sealed by cells slam_toolbox marked occupied, and no
inflation setting would have changed them. In the other two the map was open and the collar
closed it: full-a had two of four goals reachable at zero collar, one at 0.10 m, none at
the shipped 0.15 m, where the robot's reachable set fell from 2025 cells to 122.

So the despeckling filter in entry 9 was necessary and is not sufficient, for the reason
entry 10 already gave: it removes isolated cells and what closes a corridor is a band of
adjacent ones, which it correctly leaves alone. And the unstick manoeuvre of entry 11 was
being asked to escape a 13-cell pocket by driving to the furthest traversable point within
0.6 m, which inside that pocket is a few centimetres. Entry 13 recorded that the recovery
recovered nothing; this says why.

## A third analysis error, found by looking at the picture again

Entry 13 and the section above both trusted a trajectory that was partly fiction, and the
thing that caught it was someone saying the orange line looked wrong rather than any test.

The track is composed correctly, map to base_link from the two tf transforms, and that part
was already fixed once. What was still missing is that slam_toolbox rewrites map to odom
every time the scan matcher closes a loop, and the composed pose then steps to a new place
without the robot having moved. Joined into one polyline those steps draw as straight lines
across the map, through walls, between two points the robot was never between. One of them
was 1.74 m long.

Separating them needs no heuristic. A Burger tops out at 0.22 m/s and base_footprint
arrives at 50 Hz, so real motion cannot exceed about 4.4 mm per sample, and ordinary driving
in these runs sits near 3 mm. Every composed step above three times that bound coincided
with a map to odom change on the same sample, 43 of 43 in small-c and 28 of 28 in full-a.
Not one was motion. `robot_track` now returns segments broken at those steps, the figures
draw each segment separately so a correction leaves a visible gap, and distance driven is
summed inside segments only.

The numbers moved and the report's table was wrong until this was fixed: small-c dropped
from 26.73 m to 18.07 m, full-a from 33.68 to 29.89, and the cluster-size efficiency
comparison from 0.29 against 0.44 m2 per metre to 0.41 against 0.62. The direction of that
comparison survived, which is luck rather than vindication.

Two lessons. Summing consecutive samples of a pose estimate measures how far the estimate
moved, and only equals distance travelled when the frame it is expressed in holds still,
which under SLAM is exactly what it does not do. And every previous analysis error in this
lab was caught by drawing the thing and looking at it, which is now three for three.

## Two things that recurred

**The domain trap, a second time.** `param_check.sh` exported `ROS_DOMAIN_ID=77` and then
launched, and `ros2 node list` saw nothing for forty seconds. The launch file sets
`ROS_DOMAIN_ID` from its own `domain_id` argument, default 34, for every node it starts, so
exporting a domain around a launch does not put the nodes on it. The environment notes
below already recorded this and the script still did it. Passing `domain_id:=77` as a
launch argument is the fix, and the general form is that an environment variable a launch
file sets is not one a caller can set from outside.

**A traceback that was not a crash.** The same failure printed
`rclpy.executors.ExternalShutdownException` from all three nodes, which reads like three
processes dying and is what SIGINT during `rclpy.spin` looks like. It was my own shutdown,
arriving after the read loop timed out. The course's own two nodes print the identical
traceback on Ctrl-C, so this is stock rclpy behaviour rather than anything in this code.
Worth knowing before spending time on it: the thing to check first was the one line above
the traceback, which said the node had come up and had read its parameters correctly.

## Verifying a run instead of looking at one

The trajectory figure was questioned on the grounds that the robot appeared to drive
through walls, which is the right question to ask of it and was not answerable from the
figure. Three things can put an orange line on top of a black cell and only one of them is
a fault in the robot.

The first is the frame, which was the previous entry's bug and is fixed. The second is that
the figure draws the final map while the route was recorded against the map as it stood at
each moment, and slam_toolbox rewrites cells behind the robot all run. Measured on full-a:
7.3 percent of pose samples sit in an occupied cell of the final map, 0.7 percent did in
the contemporaneous map, 0.4 percent in both. Nine out of ten apparent crossings are walls
that appeared afterwards. Re-expressing the track in the final map's frame does not help,
4.5 percent for the last map to odom and 4.0 percent for raw odometry, which is the
evidence that this is not a frame problem at all.

The third is a real collision, and here I got it wrong first. I checked the raw scans, saw
a minimum return of 0.120 m against a 0.113 m robot radius, and wrote that the robot had
never touched anything. `range_min` on that scanner is 0.12 m. The sensor cannot report
anything closer than the floor it reports, so the measurement was vacuous, and 378 of 3597
scans in that run sit exactly on it. A minimum that equals a documented limit is a reading
of the limit, not of the world.

The measurement that works is a distance transform of the contemporaneous occupied set,
giving clearance from the robot's centre to the nearest obstacle the map already knew
about. Full maze: median 0.200 m, 3.1 percent of samples inside the robot radius. Small
maze: median 0.158 m, 15.4 percent. That is a real defect. It is also bunched rather than
spread, none in the first 120 s and 16.3 percent in the next 120, while cross-track error
stays at 0.038 to 0.075 m against a 0.10 m waypoint spacing. The follower is tracking
accurately down a corridor that closes around it.

The output of all this is a four panel verification sheet per run rather than a map with a
line on it: the map with the collar drawn explicitly, the route with every footprint
violation marked, coverage, and clearance against time with the robot radius and the collar
on it. A figure that cannot be wrong in three different ways at once is worth the extra
panels.

## The turn rate hypothesis, and three harness bugs on the way to testing it

The follower turns at up to 1.0 rad/s against a 5 Hz scanner, so the robot rotates about 11
degrees between scans, most of slam_toolbox's 20 degree coarse search window. That is a
clean hypothesis for the map degrading where the robot turns, and `max_w` is a declared
parameter on the course's follower, so it is testable without touching their code. A launch
argument now sets it, defaulting to their 1.0.

Halving it to 0.5 rad/s made everything worse. Free area 7.99 m2 against 9.58, 10.44 and
12.52; clearance violations 75.3 percent against 6.7 and 15.4; median clearance zero, with
the robot's own cell marked occupied from 106 s to the end. The probable reason is that the
follower's angular command is proportional to heading error and clipped at `max_w`, so a
lower clip does not turn more gently, it fails to correct at a corner. Hypothesis tested,
refuted, argument left at the course default.

Getting to that one number cost three bugs of my own, all in the run harness and all worth
recording because they share a shape.

`pkill -f "[g]z sim"` in the teardown script killed the tool call that invoked it. The
brackets stop the pattern matching its own command line, which is the trap I already knew
about, but the command that ran teardown also contained a verification `grep -E "gz sim"`
with the name spelled plainly, and that is what matched. The bracket protects one process,
not the shell that happens to mention the same string.

`setsid cmd &` sets `$!` to the setsid wrapper, not to the process that ends up leading the
new process group. The group id read back from it belonged to something already gone, the
SIGINT went nowhere, and a 300 s run left a zero byte bag while an orphaned recorder held
300 s of data in memory. Both `ros2 launch` and `ros2 bag record` handle SIGINT on their own
pid, so plain background jobs and a direct kill are simpler and correct.

The harness sourced `/opt/ros/jazzy` and `ros2_ws` but not `turtlebot3_ws`, so
`turtlebot3_gazebo` was unfindable, the launch died before Gazebo started, and the next run
recorded nothing again. The interactive environment comes from `.bashrc` and a script that
needs the same environment has to reproduce all of it, `TURTLEBOT3_MODEL` and
`GZ_SIM_RESOURCE_PATH` included.

All three produced the same symptom, an empty or missing bag, and none of them was visible
without reading the launch log. The harness now waits for `/map` to appear instead of
sleeping a guessed interval, and says so when the stack never comes up.

## Verification

- `colcon build --symlink-install` clean, four packages.
- 89 desk tests, no ROS imports in any of them. Same 89 in the team repository.
- `ros2 launch` with the full stack, parameters read back off the running node.
- Clearance measured against the contemporaneous map, per run, rather than inferred
  from the figure or from a laser minimum that turned out to be the sensor's floor.
- The plotting script's corrected track checked against the physical speed bound,
  and the team repository's copy verified to print identical numbers on the same bag.
- The team repository built from scratch and all 23 node parameters read back off
  its own install with `ros2 param get`, on 2026-09-24.
- Stray process check after every run, with bracketed patterns. `gz sim` never dies on
  Ctrl-C; the teardown script kills and then verifies.

## Still open

- The hardware session. Everything above is `sim` tier on a generated maze.
- Maze dimensions from the TA, so the worlds can be regenerated at the Concept Lab's real
  cell size.
- Recovery from a locally corrupted map. Answered above as a cause, not as a fix: the
  robot needs to turn in place and let fresh scans contradict the phantom cells, and the
  current manoeuvre cannot do that from inside the pocket.
- Separating the robot-radius margin from the map-error margin, which is where the
  collar measurement in section 7 points.
- `prd-03-exploration-and-the-information-gain-seam.md` written, build gate closed.
