# 2026-09-24, Lab 3: the experiment log for why exploration stalls

Kept separate from the build worklog because this is a different kind of record. The build
log says what was made and what broke while making it. This one is a sequence of
hypotheses about why finished, working code still does not explore the whole maze, each
with the theory behind it, what was changed, and what the numbers did. Refuted hypotheses
are kept, with the reasoning that made them plausible, because the next person to have the
same idea should be able to find out it was already tried.

Evidence tier for everything here is `sim`, Gazebo, `lab3_maze_small.world` (4.0 m) and
`lab3_maze.world` (7.2 m), 2026-09-24.

## The denominator, which should have been established first

Every coverage number before this entry was an area with nothing to compare it to. "9.58
m2 explored" cannot be read as success or failure without knowing how much there is to
explore. Rasterising the world files gives it:

| world | extent | wall area | free area inside the maze |
|---|---|---|---|
| `lab3_maze_small` | 5.2 by 5.2 m | 2.88 m2 | **13.93 m2** |
| `lab3_maze` | 8.4 by 8.4 m | 8.00 m2 | **45.29 m2** |

Against that, the runs behind the report read very differently from how they were written up:

| run | free area | fraction of the maze |
|---|---|---|
| small-a | 9.58 m2 | 68.8% |
| small-b | 10.44 m2 | 74.9% |
| small-c | 12.52 m2 | **89.9%** |
| small-cluster-a | 9.92 m2 | 71.2% |
| small-cluster-b | 9.84 m2 | 70.6% |
| full-a | 15.14 m2 | **33.4%** |

So the small maze is nearly solved on a good run and the variance between runs is the
story there, while the full maze is the real failure at a third of the map. The
experiments below target the full maze, because that is where the headroom is.

Getting this wrong for as long as I did has a specific cause worth naming: the report
argued at length that percentages are not comparable between runs, which is true of
percentages of the *SLAM grid* because that grid is sized to the pose graph. It does not
apply to a percentage of the world's own geometry, which is fixed and knowable, and I let
the first argument stand in for the second.

## What was already ruled out, before this log starts

Recorded so the list of live hypotheses stays honest. Detail is in the build worklog.

- **The trajectory drawing.** Loop closures were being drawn as motion. Fixed, and the
  distances it inflated were corrected.
- **Frame composition.** 7.3% of poses sit in an occupied cell of the final map, 0.7% in
  the map as it stood at the time. Re-expressing the track in the final map frame scores
  4.5% and raw odometry 4.0%, so the disagreement is between two versions of the map
  rather than between two frames.
- **Follower tracking.** Cross-track error is 0.038 to 0.075 m against a 0.10 m waypoint
  spacing, through the parts of the run that work. The follower is not the problem.
- **Follower turn rate (H0).** See below; tested and refuted.

## The mechanism this log is trying to break

Stated once, because every hypothesis below is an attack on one link of it.

1. The robot drives into a region where the map is locally wrong, or makes it wrong.
2. Phantom occupied cells appear at or around the robot's own position.
3. The 0.105 m inflation radius stamps a 0.15 m collar around each of them.
4. The traversable set around the robot collapses. Measured at parking: 122 cells in
   full-a, 13 in small-a.
5. Every frontier candidate becomes unreachable, because no path exists out of the pocket.
6. Ten consecutive cycles of that and the node terminates, correctly by its own rule, with
   frontiers still on the map.

The node's existing recovery drives to the furthest traversable point within 0.6 m. Inside
a 13-cell pocket that is a few centimetres of translation, which is why it recovers
nothing.

## H0: the follower's turn rate is starving the scan matcher. REFUTED

**Theory.** The follower turns at up to 1.0 rad/s and the LDS-01 produces 5 scans per
second, so the robot rotates about 11 degrees between consecutive scans. slam_toolbox's
coarse angular search window is `coarse_search_angle_offset: 0.349` rad, 20 degrees, so
each match starts more than half a window away from the last. Slower rotation should mean
smaller angular deltas and better matches.

**Change.** `max_w` is a declared parameter on the course's follower, so no edit to their
code was needed. Added a `follower_max_w` launch argument, default 1.0, and ran the small
maze at 0.5.

**Result.** Worse on every measure.

| | baseline (3 runs) | `max_w` 0.5 |
|---|---|---|
| free area | 9.58, 10.44, 12.52 m2 | 7.99 m2 |
| fraction of maze | 68.8 to 89.9% | 57.4% |
| clearance violations | 6.7%, 15.4% | 75.3% |
| median clearance | 0.180, 0.158 m | 0.000 m |

From 106 s to the end of the run the robot's own cell was marked occupied and it sat
parked inside a phantom wall.

**Why it failed, and what that teaches.** Reading the follower's control law rather than
its parameter list: angular velocity is `dif_ang * kp_yaw` clipped at `max_w`, and linear
velocity is forced to zero whenever `|dif_ang| > 0.3` rad. So it is a turn-in-place then
drive-straight controller, not a curve follower. Lowering the clip does not make it turn
more gently along a path. It makes it slower to finish the in-place turns it was already
doing, and unable to correct heading at a corner, so it leaves the corridor. The
hypothesis mistook a saturation limit for a tracking gain.

The launch argument stays, defaulted to the course's 1.0, because the next person will have
the same idea.

## What the map actually looks like against the world it was built in

Before testing any more hypotheses about exploration, a check that should have come first.
The robot spawns at the world origin and slam_toolbox anchors its map frame there, so the
SLAM map and a rasterisation of the world file are directly comparable with no
registration step.

![SLAM map against the true walls](../reports/figures/lab3-map-vs-truth.png)

The result reframes the whole problem. The real small maze is 4 by 4 m. The map sprawls
from x = -3 to x = +4.5 and y = -2 to y = +3, roughly twice the area of the world it is
supposed to represent. West of the origin the mapped walls follow the real ones with an
offset; east of about x = 0.4 the map is a fan of diagonals that corresponds to nothing in
the world at all.

Numerically, against the region each run explored:

| run | mapped wall area | true wall area there | on a real wall | phantom |
|---|---|---|---|---|
| small-a | 4.78 m2 | 1.91 m2 | 0.83 m2 | 82.6% |
| small-c | 6.36 m2 | 2.16 m2 | 0.73 m2 | 88.6% |
| full-a | 5.39 m2 | 2.97 m2 | 0.98 m2 | 81.8% |
| slowturn | 3.91 m2 | 1.53 m2 | 0.48 m2 | 87.9% |

Most of what the planner has been treating as obstacles does not exist. This is the root
cause, and every symptom in the report is downstream of it: the phantom walls are what the
collar seals the robot into, and the "unreachable frontiers" are unreachable through a
fictional maze.

It also invalidates a number I published an hour earlier. Coverage as a fraction of the
world's free area assumed the mapped free area corresponds to real free area. It does not,
so "small-c reached 89.9% of the maze" is not a claim the data supports. What it reached is
12.52 m2 of free area in a distorted map.

## Two hypotheses killed without running anything

**Occupancy thresholding.** `occupied_threshold: 65` implies the map carries graded
probabilities that could be thresholded more strictly to reject weakly supported phantom
cells. It does not. The published grid contains exactly three values, -1, 0 and 100, so
every threshold between 1 and 100 produces an identical planning grid. The parameter is
inert and the hypothesis is void.

**`max_laser_range: 20.0` against a 3.5 m sensor.** The worry was that a beam which hits
nothing comes back at the sensor maximum and, being below a 20 m threshold, is accepted by
Karto as a genuine hit, painting a ring of phantom wall at 3.5 m. Checked against the
recorded scans: no-return beams arrive as `inf`, 5.7% of all beams, and exactly 4 beams in
647280 sit at `range_max`. Non-finite readings are discarded before matching, so the
setting is genuinely inert here. The reasoning in `slam_lab3.yaml` stands.

## Re-ordering the remaining hypotheses

The map-versus-truth check changes what is worth testing. Recovery behaviour, goal
selection and inflation all operate on the map, and the map is mostly fiction. Fixing how
the robot reacts to phantom walls is treating a symptom; the phantom walls are the disease.

So the order is: find out why the map is wrong, fix that, and only then re-measure whether
the recovery and termination logic still need changing.

### H1: the recovery manoeuvre cannot clear a phantom wall. IMPLEMENTED, NOT YET TESTED

**Theory.** slam_toolbox builds occupancy by ray tracing every scan held in the pose graph:
cells a beam passes through accumulate evidence of free, the cell it terminates in
accumulates evidence of occupied. A phantom wall is a cell that collected occupied evidence
from a scan taken at a bad pose. The only thing that removes it is later beams passing
through it, which means observing that cell from a bearing it can be seen through. New
angles clear phantom walls; new positions barely help if there is nowhere to go.

The existing recovery drives to the furthest traversable point within 0.6 m. In the state
that actually occurs, a pocket of 13 to 122 free cells, that is a few centimetres of
translation and almost no change of bearing. It was the wrong manoeuvre.

**Change.** A rotation recovery, tried before the translation one. It exploits a property
of the supplied follower rather than adding a second publisher on `cmd_vel`: the follower
forces linear velocity to zero whenever the heading error to its target exceeds 0.3 rad,
and otherwise turns at the error times a gain, clipped. So a waypoint held at a fixed
2.0 rad off the robot's own nose, recomputed against the current heading every cycle, is a
pure rotation command. The error never falls below 2.0 rad, the linear term stays clamped
at zero, and the robot turns on the spot while the 360 degree LiDAR sweeps every bearing.
The waypoint sits 0.05 m away so that if the node stops republishing mid-turn, the worst
the follower can do is creep five centimetres.

`look_around_target` lives in `rrt_star.py` rather than the node, for the same reason
`densify_path` does: it is geometry, it needs no ROS, and four desk tests cover it.

Parameters: `max_look_around_cycles: 8`, `look_around_radius: 0.05`,
`look_around_step: 2.0`.

**Status.** Built and unit tested, 93 desk tests passing. Not yet run, and deliberately not
yet run, because the hypothesis below is upstream of it.

### H2: false loop closures are warping the pose graph. NEXT

**Theory.** This maze is made of identical 0.7 m corridors on a 0.8 m pitch. Every junction
looks like every other junction to a 360 degree planar scan, which is the textbook
condition for perceptual aliasing. slam_toolbox will happily match a scan taken in one
corridor against a graph node from a different corridor, and a single accepted false loop
closure drags a whole section of the pose graph to a wrong place. The fan of diagonals in
the eastern half of the overlay figure is what a warped pose graph looks like when its
scans are rendered.

The configured thresholds are permissive for such a world:
`loop_match_minimum_response_coarse: 0.35`, `loop_match_minimum_response_fine: 0.45`,
`loop_match_minimum_chain_size: 10`, `loop_search_maximum_distance: 3.0`.

There is already indirect evidence. map to odom is the correction slam applies to
odometry, and it moved 3.98 m net over one 360 s run in a 4 m maze, with single jumps up
to 1.74 m. A correction the size of the world is not a scan matcher refining a pose, it is
a scan matcher changing its mind about where the robot is.

**What has to be measured first.** A large correction means odometry and the scan matcher
disagree. On its own that does not say which is wrong, and the two have opposite fixes: if
dead reckoning drifted then the matcher is doing its job and the problem is upstream in how
the robot drives, and if dead reckoning was fine then the matcher is dragging a good
estimate away. The simulator's own pose settles it, so the experiment harness now bridges
`/world/maze_world/dynamic_pose/info` to a `/ground_truth` topic and records it.

That bridge stays in the harness and not in the launch file the teammates get. There is no
such topic on a real robot, and a bringup that depends on one is a bringup that only works
in simulation.

## A harness bug worth not repeating: editing a running shell script

The 600 s full-maze baseline died at the very end with
`run_one.sh: line 53: ch: command not found`, having produced a perfectly good bag and
then failed to stop the recorder, reindex it or print the summary.

The cause is that bash reads a script incrementally, by byte offset, rather than parsing it
all up front. The ground truth bridge was added to `run_one.sh` while that run was still
executing it, which shifted every byte after the insertion point, and the already-running
interpreter resumed at an offset that now landed in the middle of a word. `ch` was the
tail of something else.

Nothing warns about this and the failure appears minutes after the edit, at a place with no
obvious connection to it. Experiment runs now launch from a frozen copy of the script, so
editing the original between runs cannot reach a run already in flight.

The bag was recoverable: the recorder was still holding the run in memory, so a SIGTERM
flushed it and wrote `metadata.yaml`.

## exp-00: full maze baseline, current build

The comparison point for everything that follows. 600 s, `lab3_maze`, defaults.

| | value |
|---|---|
| free area | 12.17 m2 of 45.29, **26.9%** |
| clearance violations | **75.2%** of samples inside the robot radius |
| median clearance | 0.071 m |
| distance driven | 30.50 m |
| loop closure jumps in tf | **92** |

Set against `full-a` from the report, which was the same configuration on the same world:
15.14 m2 and 3.1% violations with 29 jumps. Same build, same settings, wildly different
outcome. Run-to-run variance is not a nuisance term here, it is the same size as every
effect being measured, and the jump count tracks it: 92 pose graph corrections against 29.

### H3: `occupancy_threshold: 0.1` publishes any cell with a single hit as a wall. STRONGEST

Found by reading the slam configuration again after the map-versus-truth result, rather
than by running anything.

**Theory.** Karto, under slam_toolbox, decides each published cell by a hit ratio. It
counts how many beams terminated in the cell (hits) and how many passed through or
terminated in it (passes), and publishes the cell as occupied when
`hits / passes > occupancy_threshold`, provided `passes >= min_pass_through`. The
configured values are `occupancy_threshold: 0.1` and `min_pass_through: 2`.

At 0.1, a cell that was hit once and passed through eight times is still published as a
wall. Real walls are hit on nearly every observation and sit near a ratio of 1.0. A cell
that received one spurious hit, from a beam taken at a slightly wrong pose or from the
0.01 m range noise in the model, never recovers no matter how many later beams pass
cleanly through it. That is exactly the signature measured: mapped wall area two to three
times the true wall area, and 82 to 89 percent of it standing in genuinely free space.

**The reasoning error in the current config, which is mine.** The comment beside the
parameter says it is left at the course's 0.1 deliberately, because "navigation_node does
its own thresholding at occupied_threshold 65 on the published grid, so lowering this would
move the decision into SLAM where the report cannot see it."

That is wrong twice. First, the direction: the concern was about lowering it, when the
problem is that 0.1 is already close to the floor. Second and worse, the premise. The
published grid contains only -1, 0 and 100, so by the time the node applies its threshold
of 65 every occupied cell is already 100 and every free cell is 0. The node's threshold
cannot change a single cell. The entire decision was already inside SLAM, at 0.1, and the
comment asserted the opposite while the code proved it.

Two separate measurements had already shown this and neither was connected to this
parameter at the time: the grid only ever holds three values, and the phantom rate is near
90 percent.

**Change to test.** `occupancy_threshold` from 0.1 to 0.5, so a cell is published as a wall
only when the majority of beams that reached it stopped there. Nothing else altered.

**Prediction, written before running.** Mapped wall area should fall from two to three
times the truth toward roughly one, the phantom fraction should fall well below 50 percent,
and the traversable set around the robot should stop collapsing. If the pose estimate is
also bad, which H2 is about, the map will improve but the walls will still sit in the wrong
places, so the phantom fraction will fall without reaching zero. That difference is the
thing to look for: it separates a thresholding problem from a localisation problem.

## exp-01: the root cause, found with the simulator's own pose

The ground truth channel settles it, and the answer is not in slam_toolbox at all.

### Validating the channel first

`/world/maze_world/dynamic_pose/info` bridges to a `TFMessage` whose frame names are all
empty, so the robot cannot be picked out by name. It is `transforms[0]`, the model pose,
with the remaining entries being links relative to it. Two checks before trusting it:

- Heading derived from the truth path's own direction of travel agrees with the heading
  from its quaternion to a median of 0.8 degrees, 92% within 20 degrees. A differential
  drive robot cannot move sideways, so that had to hold.
- The Burger spawns at (0.020, 0.155) with a yaw of **84.3 degrees**, not at the origin
  facing east. The first version of this comparison subtracted nothing and reported yaw
  errors near 180 degrees next to sub-metre position errors, a pair that cannot both be
  true. Everything is now expressed in the robot's own starting frame, which is also the
  frame slam_toolbox anchors its map to.

### The measurement

| | truth | odometry | ratio |
|---|---|---|---|
| distance driven | 13.62 m | 9.87 m | 0.725 |
| total rotation | 5606 deg | 3538 deg | 0.631 |

Odometry under-reports rotation by 37%, which looks like a calibration error until the
error is plotted against time rather than totalled.

| t | true rotation | odom rotation | yaw error |
|---|---|---|---|
| 16 s | 96 deg | 92 deg | -4.6 deg |
| 56 s | 245 deg | 253 deg | +8.1 deg |
| **77 s** | 310 deg | 206 deg | **-105 deg** |
| 117 s | 1909 deg | 1113 deg | -796 deg |
| 218 s | 4096 deg | 2095 deg | -2002 deg |
| 319 s | 4694 deg | 2693 deg | -2000 deg |

Not a ramp. Odometry is accurate for the first 56 seconds, the error explodes between 77
and 218 seconds, and then goes flat: after 218 s both channels gain the same 594 degrees,
so dead reckoning is working perfectly again. Whatever happened was an event, not a
mis-calibration, and it is confined to one interval.

### What the event was

The truth channel carries height and attitude as well. The base link sits at z = 0.010 m
when the robot is upright.

At **t = 75.7 s** the robot is at (+1.90, -0.38), **0.040 m from a real wall**. Its
circumscribing radius is 0.113 m, so its footprint is already 73 mm inside that wall. Over
the next second z climbs from 0.056 to 0.098 m and pitch reaches -29 degrees.

The robot drove into a wall and climbed it. 455 samples carry roll or pitch beyond 25
degrees. While it is up on the wall the wheels have no traction, the drive torque spins the
body, and the robot physically turns eleven revolutions while the wheel encoders account
for five and a half.

### The chain, end to end

1. The robot's footprint reaches a real wall and rides up it.
2. Traction is lost, the body spins, and dead reckoning misses 2000 degrees of rotation.
3. slam_toolbox's motion prior is wrong by many revolutions. Its coarse angular search
   window is 20 degrees, so the scan matcher has no chance of recovering.
4. Scans are inserted at poses that are wildly wrong, and the map becomes the fiction
   measured earlier: 82 to 89% phantom walls, twice the true area.
5. The 0.15 m collar stamps those phantom walls solid, the traversable set around the robot
   collapses to tens of cells, every frontier becomes unreachable, and the node terminates
   after ten empty cycles exactly as designed.

Every symptom in the report is step 5. The cause is step 1, and it is a collision.

### Why it hits the wall, which is arithmetic that was available all along

The corridors are 0.7 m wide, so a centred robot has 0.35 m to either wall. The
circumscribing radius is 0.113 m. The planner keeps its path outside a 0.15 m collar, so a
perfectly tracked path leaves

    0.35 - 0.15 - 0.113 = 0.037 m

of margin. The follower's measured cross-track error is 0.038 to 0.075 m. **The margin is
smaller than the tracking error**, by a factor of one to two, so contact is not a risk the
system runs, it is a guarantee given enough corners. The robot was measured at 0.040 m from
the wall when it tipped, which is the margin, spent.

This also explains why H0 made everything worse. Halving `max_w` did not change the margin;
it made the robot worse at correcting heading at exactly the corners where the margin is
spent.

### H4: the inflation margin must exceed the follower's tracking error. TESTING

**Change.** `inflation` from 0.105 to 0.16 m. With `ceil` at 0.05 m resolution that is a
4 cell, 0.20 m collar, leaving

    0.35 - 0.20 - 0.113 = 0.037 m ... no:  0.35 - 0.20 = 0.15 m of path offset,
    so clearance at the path is 0.35 - 0.15 - 0.113 = 0.087 m

of margin, which is above the measured tracking error rather than below it. The corridor
still admits a path: a 0.20 m collar on each side of a 0.7 m corridor leaves a 0.30 m free
channel.

**Prediction, written before the run.** If the collision is the root cause, this run should
show no tipping event, odometry tracking truth for the whole run, and a map whose phantom
fraction is far below 88%. If tipping still occurs, the margin is not the mechanism and the
corner-cutting geometry of the follower is the next suspect.

## The results table, all runs on `lab3_maze_small` with ground truth

Free area is against the world's true 13.93 m2. "Tip" counts pose samples with roll or
pitch beyond 25 degrees, which is the robot up on a wall. "Contact" is the fraction of
samples whose footprint overlaps a real wall. "True clearance" is the median distance from
the robot's centre to the nearest real wall, so it measures the driving rather than the map.

| run | change | free area | tip | first contact | contact | true clearance | phantom |
|---|---|---|---|---|---|---|---|
| exp-01 | baseline | 6.15 m2 | 455 | 69.4 s | 2.6% | 0.180 m | 84.8% |
| exp-02 | inflation 0.16 | 8.88 m2 | 3789 | 69.8 s | 13.3% | 0.160 m | 88.3% |
| exp-03 | unknown_lookahead 0.0 | 6.75 m2 | 1119 | 66.8 s | 21.3% | 0.180 m | 86.6% |
| exp-04 | reactive stop, 0.6 rad | 9.03 m2 | 32 | 77.8 s | 5.6% | 0.220 m | 84.3% |
| exp-05 | stop 1.6 rad + retreat | 4.51 m2 | 13908 | 83 s | - | - | 57.1% |
| exp-06 | stop 0.6 rad + retreat | 8.10 m2 | 10104 | 96 s | - | - | 87.2% |
| exp-07 | exp-04 repeated | 6.83 m2 | 584 | 67.8 s | 1.8% | 0.220 m | 86.6% |
| exp-08 | + goal clearance | 9.69 m2 | 331 | 90.3 s | 3.2% | 0.200 m | 86.7% |

**Every single run makes first contact within a few centimetres of the same place**,
(+1.84 to +1.91, -0.13 to -0.38), driving east into the outer wall of the maze. The
corridor there runs north to south with its centre at x = 1.60 and the wall face at
x = 1.95. The robot ends up 0.25 to 0.30 m east of the corridor centre, which puts its
0.113 m footprint through the wall.

### H4: raise the inflation so the margin exceeds tracking error. REFUTED

`inflation` 0.105 to 0.16, a 4 cell 0.20 m collar. Tipping went **up**, 455 to 3789, and
median true clearance went **down**, 0.180 to 0.160 m.

The arithmetic that motivated it was right and irrelevant. The collar is stamped around the
walls **in the map**, and where the robot crashes the map is either missing the wall or has
put it somewhere else. Inflating a wall that is drawn in the wrong place moves the forbidden
region to the wrong place too. A larger collar also shrinks the free channel, so the planner
routes closer to whatever it does believe in. Widening a margin measured against a bad
reference does not widen the real margin.

### H5: unknown space is traversable, so the planner routes through unmapped walls. REFUTED

`rrt.unknown_lookahead` 0.50 to 0.0. The reasoning was sound: a wall is 0.1 m thick, a
0.50 m allowance crosses one trivially, and `cluster_goal_point` only ever returns a
**known-free** point, so no goal actually requires unknown traversal.

It made no difference to the crash. First contact at 66.8 s, same place, and the contact
fraction rose to 21.3%. The robot is not crossing unknown space into a wall. It is driving
through space the map calls free.

### H6: nothing in the system consults the live laser. PARTIALLY CONFIRMED, KEPT

**Theory.** Every check in the node reads the map, and at the frontier the map is by
construction the least trustworthy thing available: it is being built from where the robot
has not been yet. The laser is in the robot's own frame and is right even when the map, the
pose and tf are all wrong. Nothing was using it. The supplied follower drives at waypoints
with no range check of any kind.

**Change.** A reactive stop in the navigation node: if the closest return within a sector
ahead falls below `safety_stop_distance` (0.18 m, above the 0.113 m robot radius and well
below the 0.35 m corridor half width), stop by publishing a park path, retire the goal, and
let the next cycle plan afresh. No second publisher on `cmd_vel`; the stop uses the same
path interface as everything else.

**Result.** The consistent effect is on the driving: median true clearance rises from
0.180 m to 0.200 to 0.220 m across every run that has it, and contact falls from 2.6% to
1.8 and 3.2%. The effect on the catastrophic tipping is not reliable: exp-04 got 32
samples, and exp-07, the identical configuration, got 584. One run is not evidence at this
variance.

Kept, because it improves the measured quantity it targets in every run, and because a
robot with a laser that never reads it is wrong regardless of what the statistics say.

### H6b: back away from the obstacle instead of stopping. REFUTED, twice

Stopping leaves the robot close to whatever triggered the stop. Backing off sounds strictly
better. Two variants, both far worse:

- 1.6 rad sector with a 0.25 m retreat: 13908 tipped samples, 259 stops in one run, 4.51 m2
  explored. The wide sector sees the corridor's own side walls, which the robot passes at
  0.15 to 0.22 m, so the stop fires continuously and the robot paralyses itself.
- 0.6 rad sector with the same retreat: 10104 tipped samples.

The retreat point sits behind the robot. The follower turns on the spot to face it, because
the heading error exceeds its own 0.3 rad threshold, and a 180 degree turn taken while
already close to a wall is precisely the manoeuvre that scrapes. Stopping puts the robot
nowhere new, and that turns out to be the point.

### H7: the frontier goal is placed against the wall. IMPLEMENTED, KEPT

Prompted by re-reading the lab's own wording for Task 4: "For Frontier-based methods, design
some algorithm to extract a set of key points to evaluate for information gain." The key
point extraction was the least considered part of the design and it is where the robot is
being sent.

`cluster_goal_point` took the cluster centroid and, when that was not known-free, walked
toward the robot and returned **the first free point it found**. That point is by
construction the one closest to whatever blocked the centroid, so it sits against a wall
with only the collar for clearance. And the collar is computed from the map, which at a
frontier is exactly where it is least reliable.

The walk now continues 0.35 m past the first free point and returns the roomiest point
found, using a distance transform of the blocked set added to `PlanningGrid`.

**Result.** exp-08 explored 9.69 m2, the most of any run, and first contact was delayed
from 67 to 90 s. Both are single runs against a variance that spans 6.15 to 9.69 m2, so
this is a direction rather than a demonstration.

## Where this stands, honestly

The root cause is established and is not where the report said it was. It is not gain
selection, not inflation, not unknown handling, not the turn rate, and not slam_toolbox's
tuning. **The robot drives into a real wall at one particular place, rides up it, loses
traction, and the resulting 2000 degrees of unmeasured rotation destroys the pose estimate
and with it the map.** Everything the report described as an exploration failure is the
wreckage of that one event.

Not solved. Contact is delayed and made less frequent, median clearance is measurably
better, and coverage is up on the runs that have both changes, but no configuration yet
avoids the crash, and one crash is enough to end a run.

The next thing to try, and the reason it has not been tried yet, is that it needs a run
budget rather than an idea: the three changes worth keeping were each measured once against
a metric whose run-to-run spread is larger than the effect. Every number in the table above
is n of 1. Before anything else is added, the kept configuration needs five runs and so
does the baseline.

After that, the specific unanswered question is why the planner routes 0.25 m off the
corridor centre into the east wall when the map there is not obviously wrong yet. The
instrumentation to answer it now exists: ground truth, the true-path plot, and per-cycle
map state at the contact point.

## What the lab instructions say, having finally read all of them

Three things in `docs/labs/Lab_Instructions_Exploration.pdf` bear directly on the failure,
and none of them had been used.

**Task 3 names the fix first.** "Integrate some kind of collision avoidance system based on
your knowledge from this course. **Reactive (APF)**, map inflation, risk heuristic, or
other. As long as the robot is ensured not to drive into walls due to the RRT only
considering the robot as a point."

Reactive is listed before inflation, and the requirement is stated as an outcome: the robot
is ensured not to drive into walls. We implemented inflation alone, reported Task 3 as
answered, and the measured outcome is a robot that drives into walls. The task was not met
on its own terms, and the instructions had already named the method that addresses it.

**The course allows modifying the supplied nodes.** "It is recommended that you take some
time to study the code in the package. If you like, you are allowed to modify the nodes as
you feel is needed." The path follower's control law, which turns on the spot whenever
heading error exceeds 0.3 rad and otherwise drives straight at a waypoint 0.2 m ahead, is
implicated in the corner cutting. It was treated as immovable throughout this lab because
of a design decision taken early, not because the course required it. That decision is
worth revisiting, and it is Dominic's to take.

**The competition metric is A plus B**, time to complete exploration plus time to 90
percent, which is why both are already in the summary output.

## H8: the maze is narrower than the course's own. CONFIRMED, LARGEST SINGLE EFFECT

Found by reading `reference/R7021E_MazeGen-master`, which is the generator these worlds came
from. Its walls are emitted as `<size>{cell - 0.1} 0.1 1.5</size>`, so the wall segment
length gives the cell pitch away.

| | wall segment | cell pitch | corridor | half width | slack past a 0.113 m robot |
|---|---|---|---|---|---|
| our worlds | 0.700 m | 0.8 m | 0.70 m | 0.350 m | 0.237 m |
| the course's committed `maze.world` | 0.900 m | 1.0 m | 0.90 m | 0.450 m | 0.337 m |

Our maze is built at `-l 0.8`. The generator's default is `-l 1`, and the reference world
the course ships is built at that default. We made the problem 22 percent harder than the
environment the course provides, and then spent a day tuning against the difference.

Regenerating at the course's own cell size, same 5 by 5 layout, 72 wall segments, 22.41 m2
of free space:

| | exp-08, 0.7 m corridors | exp-09, 0.9 m corridors |
|---|---|---|
| phantom walls | 86.7% | **63.8%** |
| first contact | 90.3 s | 72.8 s |
| stuck after contact | 11% of the next 20 s | 12%, and it got out |
| minimum true clearance | 0.040 m | 0.072 m |
| free area | 9.69 m2 | 13.58 m2 |

The phantom rate is the number that matters, and it fell by a quarter from a single
environment change. The robot also recovered from its first contact instead of being pinned
by it, which is the first time that has happened in any run.

This does not excuse anything. The collision mechanism is real and would still be real in a
0.9 m corridor. But every measurement taken before this one was taken in a maze narrower
than the one the lab will actually be run in, and that has to be said plainly in the report.

## H9: reactive path centring, the APF half of Task 3. IMPLEMENTED, MIXED

**Theory.** Inflation is a hard constraint. It forbids the collar and is then completely
indifferent between a path that hugs the collar boundary and one down the middle of the
corridor, so RRT* takes the hugging one because it is shorter. That is why the robot's
median distance to a real wall was 0.16 to 0.22 m in corridors whose half width is 0.35 to
0.45 m: it drove nearer the walls than the centre for entire runs, with no margin left for
the first thing to go wrong. A potential field is the standard answer and the one Task 3
names.

**Change.** `centre_path` in `rrt_star.py`, ROS free and covered by five desk tests. Each
interior waypoint climbs the gradient of the clearance field, estimated by sampling the
distance transform either side of it, by up to `path_centring_shift` (0.12 m). Endpoints
are pinned, steps that land blocked are rejected, and steps that do not increase clearance
are rejected, so it can only return a path the planner would also have accepted. Applied
before densifying, so the interpolation follows the moved path rather than bending it.

**Result, on the course-width maze.**

| | exp-09, no centring | exp-10, centring |
|---|---|---|
| median true clearance | 0.160 m | **0.200 m** |
| contact fraction | 6.1% | **3.2%** |
| first contact | 72.8 s | **123.9 s** |
| tipped samples | 202 | 10664 |
| free area | 13.58 m2 | 9.91 m2 |

It does precisely what it was designed to do: the robot drives further from walls, touches
them half as often, and survives 50 seconds longer. And the run still ended worse, because
when it did make contact at 123.9 s it stuck, and one stick is the whole run.

That pattern is now consistent across every change tried. The measures that track the
driving all improve. The measure that decides the run, whether any single contact turns
into a wedge, does not. Reducing the frequency of an event that is individually fatal does
not help much until it reaches zero.

## Status and the one thing to do next

Established, and none of it was in the report before today: the failure is a physical
collision, the collision destroys odometry through wheel slip while the robot is up on a
wall, and the destroyed odometry is what makes the map fiction. The maze was narrower than
the course's own. Task 3's reactive option was never implemented and is now in.

Still not solved. No configuration yet reaches zero wedges, and nothing less will do.

The next step is not another hypothesis. Nine of the eleven runs in this log are n of 1
against a spread that exceeds every effect measured, and two configurations that differed
only in random seed produced 32 and 584 tipped samples. The kept set, course-width maze
plus reactive stop plus goal clearance plus path centring, needs five runs against five
baselines before anything else is added or removed. Everything needed to do that exists:
the harness, ground truth, and a verdict table that reads tipping, odometry error, phantom
fraction and coverage off a bag in one command.

## A measurement correction: "free area explored" was never coverage

Every coverage number above this line, in this log and in the report, came from counting
free cells in the SLAM occupancy grid at the end of a run. That was wrong, and it was
wrong in a direction that flattered exactly the runs that failed worst.

The grid those cells were counted in is between 64 and 89 percent phantom. A run whose
map doubles the size of the real maze reports more free area than a run whose map is
roughly right, because the fiction is made of free cells too. So "free area explored"
was measuring how much wrong map got built, and reporting it as progress.

The replacement is `true_coverage.py`, and it contains no SLAM at all. It rasterises the
world file into true occupied and true free space, takes the flood fill that excludes the
apron outside the outer wall, and then, for every ground truth pose at 2 Hz, casts 360
beams at one degree out to the LDS-01's own 3.5 m, stopping each beam at the first
occupied cell. What the beams reach is what the robot saw. Ground truth pose, real
geometry, real sensor field of view, and nothing that can drift.

The correction is large enough to reverse a conclusion:

| run | SLAM "free area" | true coverage | phantom |
|---|---|---|---|
| exp-09, wide maze | 13.58 m2 | 14.03 m2 of 22.41 (62.6%) | 63.8% |
| exp-10, plus centring | 9.91 m2 | 15.61 m2 of 22.41 (**69.6%**) | 73.4% |
| exp-11, our follower | 16.92 m2 | 14.83 m2 of 22.41 (66.2%) | 88.2% |

Read off the SLAM grid, exp-11 explored 71 percent more than exp-10. Measured against the
maze that actually exists, it explored slightly less. The 16.92 m2 was phantom.

Every coverage claim in the report has to be restated from this metric before the report
goes anywhere.

## H10: replace the supplied path follower. KEPT, for the metric it targets

The lab instructions say "you are allowed to modify the nodes as you feel is needed". An
earlier decision in this build treated the supplied follower as fixed, and that decision
came from us, not from the course.

**Why it was suspected.** Two numbers read off the exp-09 and exp-10 bags:

- 45 to 62 percent of every velocity command in a run is a pure rotation, zero linear
  velocity, robot turning on the spot and covering no ground.
- The law flips between turning and driving about 14 times a minute. Median turn burst
  0.9 s, median drive burst 1.5 to 2.5 s.

That is the supplied law working as written. It zeroes linear velocity whenever the
heading error to its target exceeds 0.3 rad, and the waypoints are 0.1 m apart on a path
that bends, so the robot stops, turns, creeps, and stops again. The competition is scored
on time.

**What was checked first and found innocent.** The supplied node returns early on an empty
path without publishing anything, and the Gazebo differential drive plugin has no command
timeout, so a stale velocity would stand indefinitely. Measured: zero gaps longer than
0.5 s in either bag, and zero metres of body motion during gaps. The navigation node's
park manoeuvre keeps the follower publishing, so the hazard exists in the code and has
never fired. Recorded here because it is the kind of thing that fires on hardware.

**Change.** `path_follower.py`, the law with no ROS in it, and `path_follower_node.py`
around it. Same topics, same message types, same five parameter names. Three differences:

1. Arrival is detected and the robot stops. The supplied version pops waypoints only
   `while len(path) > 1`, so its list never empties and at the end of a path it chases
   `atan2` of a vanishing vector and spins. This also turns the navigation node's park
   manoeuvre from "rotate to face world east and settle" into "stop", with no change to
   the navigation node.
2. Speed tapers with `cos(heading error)` instead of switching off at 0.3 rad.
3. The turn-in-place threshold moves to 1.2 rad, where `cos` has fallen to 0.36 anyway.

`follower:=course` still runs the supplied one, so the comparison stays available instead
of being asserted.

**Result, exp-10 against exp-11, one variable, the control law.**

| | exp-10, course | exp-11, ours |
|---|---|---|
| pure rotation commands | 45.2% | **8.4%** |
| true body rotation / commanded | **3.73** | **0.90** |
| flips per minute | 14.8 | 10.6 |
| median drive burst | 1.5 s | 3.0 s |
| time to 50% of the maze | 86 s | **46 s** |
| time to 66% of the maze | **103 s** | 164 s |
| time to 75% of the maze | never | never |
| true coverage | 69.6% | 66.2% |
| first contact | 123.9 s | 66.9 s |
| contact fraction | 3.2% | 60.2% |
| phantom | 73.4% | 88.2% |

The second row is the one that matters most and is easy to skim past. In exp-10 the body
turned 3.73 times further than it was ever commanded to turn: 27192 degrees of real
rotation against 7288 commanded. That gap is the spin-out, the wheels slipping while the
robot is up against something, and it is the mechanism this log has been chasing since the
ground truth analysis. In exp-11 the ratio is 0.90. The body did what it was told.

A note on how those coverage times are counted, because the first version of this table
got it wrong. Measuring "time to 90 percent of what this run achieved" rewards a run for
giving up early: a run that reaches 60 percent and stops hits 90 percent of its own total
sooner than a run that goes on to finish. On that measure exp-11 looked like a halving,
95.9 s against 51.0 s. Against a fixed denominator, the maze, it reaches half the maze in
half the time and then falls behind, taking 164 s to reach two thirds where exp-10 took
103 s, because it started hitting walls at 66.9 s instead of 123.9 s. The competition
scores time to explore the maze, so the maze is the denominator.

**Verdict: kept, but not on its own merits.** The control signal improves decisively and
the spin-out confound is gone from every future measurement, which is worth having by
itself. On the scored metric alone it is a wash: faster early, slower to two thirds,
never past it. What it did was remove the thing that was masking the real fault, and the
real fault is the next section. This is n of 1 and stays n of 1 until the run budget is
spent.

**A coupling this exposed.** The look-around recovery commands a turn by publishing a
waypoint off the robot's nose, which only works because the supplied law refuses to
translate above 0.3 rad of heading error. Against a follower that stops on arrival, a
waypoint 0.05 m away against a 0.05 m arrival tolerance is a stop, and exp-11 spent eight
consecutive recovery cycles publishing one. Fixed by making the two invariants explicit in
`lab3.yaml` and checking them in a desk test: the radius must exceed the follower's
arrival tolerance, and the step must exceed its turn-in-place threshold plus what the robot
can turn between republications. Now 0.25 m and 2.6 rad.

## H11: the reactive check was running fifty times too slowly

**Where it came from.** `onset.py` prints the twenty seconds before first contact: true
pose, true clearance, commanded velocity, once a second. For exp-11 it reads

```
  63.9   -0.03    1.96   yaw 156   clear 0.450   v 0.150
  64.9   -0.11    2.07   yaw 114   clear 0.400   v 0.097
  65.9   -0.17    2.21   yaw 109   clear 0.250   v 0.149
  66.9   -0.21    2.35   yaw 108   clear 0.100   v 0.150
```

The robot drove into the outer north wall head on, at full commanded speed, with no
turning to speak of. Not a corner clipped while arcing, not a path through a phantom wall.
A straight approach to a wall that was really there.

The safety stop did not fire, and the reason is arithmetic. It lives in the navigation
node's `_tick`, and `replan_period` is 1.0 s. At 0.15 m/s the robot covers 0.15 m between
consecutive checks, and `safety_stop_distance` is 0.18 m. The check therefore gets roughly
one sample inside the band it is supposed to catch, and if that sample lands late the robot
is already against the wall. The 0.25 to 0.10 m step in the trace above is one tick.

This also explains why the supplied follower survived longer: stop-turn-go moves more
slowly on average, so a 1 Hz check catches it. It was never safer, only slower.

**Change.** The reactive layer moves into the follower, which runs at 10 Hz against a 5 Hz
scan. `reactive_limit` caps forward speed on the nearest return within `sector` of straight
ahead: full speed above `slow_distance` (0.30 m), tapering linearly to zero at
`stop_distance` (0.18 m). Angular velocity is untouched, because a robot that cannot turn
away from a wall it has stopped in front of has not stopped, it has parked against it. A
scan older than `scan_timeout` counts as blocked rather than clear, and says so in the log,
because a follower that silently refuses to drive is worse than one that refuses loudly.

The margin, written out: braking from 0.15 m/s at the drive plugin's 1.0 m/s2 takes
0.011 m, plus 0.015 m travelled before the next cycle sees anything. 0.026 m against the
0.12 m between `stop_distance` and the LiDAR's own `range_min` floor.

The planner's own safety stop stays. It does a different job: it retires the goal, which is
a planning decision. This one only decides speed.

This is the second half of Task 3's "Reactive (APF)", and it is the half that can actually
run at sensor rate. Inflation is a statement about the map, and the map is wrong exactly
when it matters.

**Result, exp-12. This is the run the log has been trying to produce since the ground
truth analysis.**

| | exp-10, course follower | exp-11, ours | exp-12, ours plus reactive |
|---|---|---|---|
| footprint overlapping a real wall | 3.2% | 60.2% | **0.0%, zero samples** |
| minimum true clearance | 0.040 m | 0.040 m | **0.152 m** |
| tipped samples | 10664 | 13097 | **0** |
| final odometry yaw error | 768 d | 2136 d | **1 d** |
| true coverage | 69.6% | 66.2% | **100.0%** |
| time to 50% of the maze | 86 s | 46 s | **22 s** |
| time to 75% of the maze | never | never | **119 s** |
| time to 90% of the maze, the B metric | never | never | **140 s** |
| time to the whole maze, the A metric | never | never | **231 s** |
| distance driven | 24.06 m | 26.30 m | 38.08 m |
| safety stops in the planner | 15 | 17 | **0** |
| how it ended | wedged | wedged | terminated normally, 10/10 empty cycles |

Zero contact samples out of 20000. The closest the robot's centre came to a real wall all
run was 0.152 m against a 0.113 m circumscribing radius, so 0.039 m of air at the worst
moment. It never tilted past 25 degrees, not once. Odometry finished one degree out after
340 seconds and 38 metres. It covered the whole maze and stopped because there was nothing
left, which is the terminating condition the node was designed around and which no
previous run had reached honestly.

Against the competition's own scoring this is the difference between a score and no score.
A is time to full exploration and B is time to 90 percent, and exp-12 is the only run in
this log that reaches either: 231 s and 140 s. Every other run, including both halves of
the H10 comparison, never gets past two thirds of the maze at all.

The reactive layer costs nothing in throughput, which is worth stating because a speed
limiter that fires constantly would. Mean forward speed is 0.140 m/s in both exp-11 and
exp-12, identical. What changes is how much of the run is spent moving forward at all:
79.8 percent of commands against 55.7 percent, and 38.08 m driven against 26.30 m. The
time was never being lost to the limiter. It was being lost to crashing.

The planner's own safety stop fired zero times, because nothing ever got close enough for
it to have an opinion. It stays in anyway: it does a different job, retiring a goal, which
is a planning decision rather than a velocity one.

Task 3's requirement is written as an outcome, "as long as the robot is ensured not to
drive into walls". This is the first run in the series that meets it as written.

**What was actually wrong, in one sentence.** The reactive check existed, was correct, and
ran fifty times too slowly to act, because it had been put in the planner's loop instead of
the sensor's.

## A second measurement correction: "phantom" was mostly wall thickening

exp-12 finished with one degree of odometry error, zero wall contact and a map it explored
to completion, and the phantom fraction called that map 84.9 percent fictional. A number
that says that about that run is not describing the map.

Pulling it apart, the measure was summing three unrelated things:

- **registration.** The map is a few centimetres off, so a correctly mapped wall lands one
  cell outside a truth mask compared by exact cell equality. Dilating the truth by one cell
  takes exp-12 from 84.9 to 71.9 percent, which is registration alone.
- **thickening.** slam_toolbox marks the hit cell of every beam endpoint. Beams arrive at
  both faces of a wall, with angular spread, over hundreds of scans, so a 0.1 m wall comes
  out as a band several cells wide. exp-12 maps 6.51 m2 of wall against 2.58 m2 of truth,
  2.52 times. That is ordinary behaviour for an occupancy grid, not a fault.
- **fiction.** A mapped wall with no real wall near it. This is the one that seals the
  robot in and the only one the word phantom should have been used for.

`map_quality.py` reports them separately. Fiction is the fraction of mapped wall cells more
than 0.30 m from any true wall, which is wider than a wall plus any registration error seen
in these runs.

| run | mapped | true | thickening | median offset | fiction |
|---|---|---|---|---|---|
| exp-09 | 3.30 m2 | 2.63 m2 | 1.25x | 0.05 m | 19.6% |
| exp-10 | 4.12 m2 | 3.04 m2 | 1.36x | 0.10 m | 19.4% |
| exp-11 | 5.63 m2 | 2.53 m2 | 2.23x | 0.25 m | 48.0% |
| exp-12 | 6.51 m2 | 2.58 m2 | 2.52x | 0.20 m | 40.4% |

Note what this does to the reading of exp-09 and exp-10: they score better here only
because they died early and stopped inserting scans. A run that maps a tenth of the maze
accurately and then stops has less to be wrong about. The comparison is not like for like
and this table should not be read as exp-09 having built a better map.

Two corrections in one day, both in my own instrumentation, both flattering the runs that
failed. The general lesson is the same one as the loop-closure lines in the trajectory
plot: a derived metric with no independent check will agree with whatever it is compared
against. Ground truth is the only thing in this project that has not needed correcting,
because it is the only thing not computed from something the robot believed.

**What the earlier claims in this log become.** Every use of "phantom" above this section
means thickening plus registration, not fiction. The H8 headline, phantom 86.7 percent
falling to 63.8 percent on the course-width maze, was a real improvement in map quality and
was not an improvement in fictional walls. The wide maze result stands; its description
does not.

## Still open, after the collision problem is closed

The map is thick. exp-12 draws walls 2.52 times their true area and 40 percent of its
mapped wall cells sit more than 0.30 m from any real wall, on a run whose pose was accurate
to one degree throughout. That is a mapping question, not a driving one, and it is now the
largest remaining defect.

The obvious candidate is the warning slam_toolbox prints on every run, that
`max_laser_range` is 20.0 m against a 3.5 m LiDAR. That one is already closed: it was
tried at 3.5 on 2026-09-23 and reverted, because the parameter clips how far a ray is
rastered rather than which rays are trusted, and clipping it at exactly the sensor maximum
makes no-return beams raster as though they had struck something. The reasoning is written
out in `slam_lab3.yaml` beside the value. Noting it here so it does not get retried a
third time.

The candidate that is still open is `occupancy_threshold`, listed in this log as H3 and
still untested. It sits at the course's 0.1, meaning a cell is published occupied once one
ray in ten has ended in it. With `min_pass_through` at 2, a cell taking one grazing
endpoint and nine clean passes is published as wall. That is a precise description of the
mechanism that puts a band of occupied cells either side of a real one.

The note beside it in `slam_lab3.yaml` says it was left low deliberately because
`navigation_node` thresholds the published grid at 65 anyway. That reasoning does not hold
and this log already contains the measurement that breaks it: the published grid is binary,
only -1, 0 and 100, so a threshold at 65 discriminates nothing and the decision is made
entirely inside SLAM by this parameter. Raising it to 0.5 asks for a majority of rays
before a cell becomes a wall.

And exp-12 is one run. The same configuration is repeating twice, with one run of the
supplied follower on the same settings as the paired baseline, before any of this is
written into the report as settled.
