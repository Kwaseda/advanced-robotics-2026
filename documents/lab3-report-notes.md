# Lab 3 design notes: autonomous exploration

The decisions behind the Lab 3 implementation, the alternative that was rejected in each
case, and the findings worth explaining rather than hiding. The Lab 3 report is written
from this.

Everything measured here is from simulation on generated mazes, unless a line says
otherwise. Hardware results replace these numbers, they do not sit alongside them.

---

## 1. RRT* rather than RRT

**Chosen:** RRT*, with the cheapest-parent choice and rewiring, and with the tree growing
past the first solution so the shortest goal-connected branch wins.

**Rejected:** plain RRT, which would have been faster to write.

**Why:** the lab instructions state that computation time is not a priority, which removes
the only real argument for plain RRT. A plain-RRT path is usually jagged, and "why does
the robot take that route" is a question with no good answer when the route is whatever
the first random branch happened to find.

**Cost:** rewiring is the part that is easy to get subtly wrong. Re-parenting a node
without pushing the corrected cost down its subtree leaves the tree looking fine from
outside while later comparisons mix current and stale costs, and the tree quietly stops
converging. There is a unit test that walks the whole tree and checks every node's cost
against its parent, which is the only way to see it.

## 2. Collision avoidance by map inflation

**Chosen:** inflate every occupied cell by the robot's radius before the planner sees the
map, and let the RRT keep treating the robot as a point.

**Rejected:** a reactive artificial potential field, and a risk term inside the
exploration heuristic. Both are allowed by the instructions.

**Why:** the margin becomes geometry rather than behaviour. A potential field is a second
controller fighting the path follower for `cmd_vel`; a risk term in the heuristic makes
obstacle distance trade against exploration gain, so a large enough frontier can buy its
way through a wall.

**The number:** 0.105 m. The Burger's footprint is 178 by 138 mm, so the circumscribing
radius is 0.113 m, and 0.105 m is that with a small allowance because the circumscribing
radius is the worst-case diagonal rather than the typical clearance. It is the same number
Lab 2 used, so one footprint decision covers both labs.

**The detail that matters:** at the map's 0.05 m resolution, 0.105 m is 2.1 cells and the
stamp rounds **up** to 3, so the enforced collar is 0.15 m. Rounding down would enforce
less margin than the configuration file claims, which is worse than either number.

**Unknown cells are not inflated**, and that is deliberate. Inflating them puts a blocked
collar around every frontier, which defeats the method entirely.

**Speckle is removed before the inflation**, and this turned out to matter more than the
radius does. A live SLAM map carries isolated occupied cells in the middle of free space.
One is harmless; inflated by a 3-cell disc each becomes a 7 by 7 blocked blob, and a
region with a scatter of them two cells apart has no traversable cell left in it at all
while its raw map is almost entirely free. An occupied cell with no occupied neighbour
anywhere in its 8-neighbourhood is dropped first. Walls are continuous, so a real wall
cell always has one, and the threshold of 1 is the most conservative version of this that
does anything at all.

**The robot's own pose is often not traversable**, and that is normal rather than an
error: a 0.7 m maze corridor is narrower than twice the 0.15 m collar plus the robot.
Refusing to plan from a blocked start stops the run, so two mechanisms cover it. Edges
leaving the root may begin with a bounded blocked prefix, sized so it can leave a collar
and cannot cross a wall. Where that is not enough, the tree is rooted at the nearest
genuinely traversable point and the robot's own pose is put back on the front of the
returned path.

## 3. Unknown space is traversable

**Chosen:** unknown cells are traversable, bounded by `rrt.unknown_lookahead` at 0.50 m.

**Rejected:** treating unknown as blocked, with an exemption near the goal.

**Why:** a frontier goal sits on the boundary of unknown space by definition. A planner
that refuses unknown cells cannot reach a single goal the exploration gain ever picks, so
the robot declares the maze explored while standing in a corridor.

**The bound:** without it, RRT* will happily plan a route across half an unmapped maze on
no evidence, and then score that fictional path length into H. The budget is carried on
the tree node rather than checked per edge, because three short legal steps through
unknown space otherwise add up to one illegal one.

## 4. Frontier-based rather than next-best-view

**Chosen:** frontier-based, following Yamauchi (1997).

**Rejected:** next-best-view scoring the RRT's own leaf nodes.

**Why:** the course supplies the frontier extractor, which does the hard part.
Next-best-view needs the gain scoring to reach into the planner's internal tree, which
couples two pieces that are otherwise separately testable. Frontier-based also has a real
citation behind it rather than an invented justification.

**Honest note:** next-best-view is cheaper per cycle, because it scores leaves of one tree
rather than planning a separate path per candidate. The expensive option was chosen
deliberately, on the grounds that computation time is not graded and the real path length
is a much better distance term than a straight line.

## 5. The information gain

**Chosen:** `I(p)` is the number of frontier cells within 0.75 m of the candidate goal,
counted across the whole frontier grid rather than within the candidate's own cluster.

**Rejected:** plain cluster cell count, which is simpler and needs no radius.

**Why the reduced range:** it is the lab instructions' own tip, which exists because
without it every frontier in a small maze scores nearly the same and the robot has no
reason to prefer any of them. 0.75 m is a fifth of the Burger's real 3.5 m LiDAR.

**Why across the whole grid:** two clusters 20 cm apart each score their own size under
cluster count, and neither knows the other is there, so the robot commits to a spot that a
single visit would have cleared anyway. Counting every frontier cell within the radius
prices that correctly.

**What it is not:** a sensor model. It does not trace rays, so it over-counts frontier
cells hidden around a corner. Ray casting per candidate per cycle costs far more than it
buys at a radius this short, and the approximation is stated rather than hidden.

Both forms are implemented and selected by `gain_mode`, so the two can be run on the same
maze and compared rather than argued about.

## 6. The heuristic needs a unit conversion, and saying so matters

`H(p) = sum(d(p)) - w I(p)`, minimised.

The lecture's form is `H(p) = sum(d(p)) - I(p)`, which subtracts a cell count from a
distance. Those are not the same unit, so without a conversion factor the expression
cannot be evaluated at all. `w = 0.10` metres per frontier cell is that factor.

It is also the greedy-versus-complete knob the optional competition asks about. At `w = 0`
the robot always drives to the nearest frontier. As `w` grows it will cross the maze for a
large unexplored region. At 0.10 and a 0.75 m radius, a well-populated frontier scores 30
to 60 cells, so `I` is worth 3 to 6 m of driving.

`sum(d(p))` is the length of the RRT* path actually planned to that candidate, not a
straight-line estimate. That is the more expensive option from the slides and it matters:
a frontier just behind a wall is a few centimetres away in a straight line and several
metres away by road.

## 7. Two pieces of memory around a memoryless heuristic

`H` depends only on the current map and the current pose. Two things have to be remembered
outside it, and both were added because of a specific failure.

**The commitment.** As the robot drives toward cluster A, `d` shrinks for A, but the map
grows behind it and `I` can flip the ranking to B and back. That is frontier oscillation,
and the fix is to keep the current goal unless a rival beats it by more than
`commit.switch_margin`. The commitment is released on arrival: it exists to prevent thrash
while driving, not to re-choose the same place after reaching it.

**The retirement list.** A goal the robot has arrived at, or stalled trying to reach, is
never offered again, nor is any candidate within `goal.exhaust_radius` of it. Without it,
a frontier that survives the robot standing next to it keeps winning on distance forever,
because nothing in `H` knows where the robot has been. The observed failure is the robot
arriving, finding the frontier still there, replanning 0.3 m to the same goal, and
arriving again, indefinitely.

Retiring on arrival is safe rather than aggressive. A frontier the robot actually cleared
disappears from the frontier grid on the next map update and would never be proposed
again; one that survives is one that standing there again will not clear.

## 8. Termination

`select_best_frontier` returning nothing is the "explored enough" signal, but one empty
result is not enough to act on.

The frontier cell count dips and recovers during a healthy run, from a couple of hundred
cells to a couple of dozen, as SLAM redraws the map behind the robot, and a couple of
dozen scattered cells contain no run of five connected ones. `max_empty_cycles` is 10 at a
1 Hz replan period against a 1 Hz map update, so it is ten independent looks at the world.

The counter does not run until the node has chosen a goal at least once. The node starts
before SLAM has published anything, and a short fuse burning during startup produces a run
that announces "exploration complete" within seconds of launch.

One state is deliberately not treated as termination: clusters are still being found,
every one of them is unreachable, and the robot is standing still. That is not an explored
maze. It means the robot's own surroundings have closed around it in the map, and a
stationary robot cannot map its way out, because phantom walls only disappear when new
scans contradict them. The node publishes a short path to the furthest traversable point
within 0.6 m instead, up to four times, and any successful cycle resets the count.

The furthest, not the nearest, and the difference is not cosmetic. Driving to the nearest
free point leaves the robot standing beside it, so the next attempt is the distance from
there to the same point: 0.15 m, then 0.08, then 0.03, then 0.01, four attempts spent
without the robot going anywhere.

Stopping the robot is its own problem, because the supplied follower has no message that
stops it: it never empties its own waypoint list and returns early on an empty path
without publishing zero. Termination therefore publishes a path holding a single waypoint
at the robot's own pose, which drives the follower's distance term to zero. The robot then
rotates to face world east, because `atan2(0.0, 0.0)` is 0.0, and parks. That rotation is
cosmetic and is stated rather than hidden.

## 9. Clustering is 8-connected, and the extractor is 4-connected

These answer different questions and it is worth being explicit about which is which.

`frontier_detector_node` uses a 4-neighbourhood to ask whether one free cell is adjacent
to unknown space. That is a statement about a single cell.

Grouping frontier cells into boundaries is a second question, and a boundary that runs
diagonally is still one boundary. The boundary of what a rotating LiDAR has seen is a
curve, and a curve on a grid is a staircase. Under 4-connectivity a diagonal frontier is
not a cluster at all: it is N clusters of one cell, all below the noise floor, all
discarded. The robot then reports zero clusters and declares the maze explored while
looking straight at a frontier in RViz.

The observed cost of getting this wrong was a run that stopped at 65 percent of the mapped
area with the entire right half of the maze never visited, and a log that read as complete
success.

The price of 8-connectivity is that two frontiers touching at one corner merge into a
cluster whose centroid sits between them. The centroid fallback walk handles the case
where that lands somewhere unusable, and the size cap splits anything long enough for the
merge to matter.

## 10. Real-time problems, and what each one cost

Worth reading before the lab session, because four of these are not visible in unit tests.

| Problem | Symptom | Fix |
|---|---|---|
| QoS mismatch on `/frontiers` | Subscription receives nothing, silently. Both sides log "incompatible QoS" | `/map` is transient-local, `/frontiers` is volatile. Two profiles |
| Topic name in the course template | The template's frontier subscription can never fire | It declares `frontier`; the publisher uses `frontiers` |
| Termination during startup | "Exploration complete" seconds after launch | Gate the counter on having chosen a goal once |
| Diagonal frontiers | Zero clusters with frontiers plainly visible | 8-connected clustering |
| Arrive, reselect, arrive | Robot moves a few centimetres back and forth indefinitely | Release the commitment on arrival, retire the goal |
| Follower cutting corners | Robot wedged against a wall, then a smeared SLAM map | Publish waypoints at 0.10 m, half the look-ahead |
| Two `/clock` publishers in sim | "Detected jump back in time", doubled walls in the map | One clock bridge. `spawn_turtlebot3.launch.py` already provides it |
| SLAM scan interval vs turn rate | Map's second half rotated and smeared | `minimum_time_interval` 0.1, so rotation between scans stays inside the matcher's angular search window |
| Speckle, inflated | Frontier goals in open space reported unreachable; a flood fill from the robot reaches one cell | Despeckle before inflating; root the tree at the nearest traversable point |
| Robot sealed into a map pocket | Clusters found, none reachable, robot stationary, run "complete" | Move to the furthest traversable point and let SLAM re-observe |
| Recovery that recovers nothing | Unstick distances of 0.08, 0.03, 0.01 m | Target the furthest reachable point, with a minimum useful distance |
| A test that needed ROS | Suite fails without a sourced ROS installation | `densify_path` moved into the planner module, where it belongs |

The last one has arithmetic behind it worth repeating: the follower turns at up to
1.0 rad/s, so a 0.5 s minimum interval allows 0.5 rad between accepted scans, against a
`coarse_search_angle_offset` of 0.349 rad. The search window was smaller than the rotation
it had to recover, so fast turns fell back on odometry.

## 11. What the chain of failures teaches

Most of the problems above are one causal chain rather than a list of separate bugs: a
path the follower does not track exactly, which wedges the robot, which slips the wheels,
which runs odometry away from truth, which exceeds the scan matcher's search window, which
corrupts the map, which puts spurious occupied cells into free space, which the inflation
turns into solid regions, which makes every frontier unreachable, which ends the run with
a log that says the maze is explored.

None of that is visible from the planner's own outputs, and the unit tests pass throughout.
Every single stage of it is individually correct: the follower tracks its waypoints, the
scan matcher reports its best estimate, the inflation stamps the radius it was given, the
planner correctly finds no path, and the exploration logic correctly concludes there is
nothing reachable left. The only thing that surfaced any of it was rendering the map, the
frontier cells and the trajectory together and looking at the picture.

## 12. What six runs actually measured

Six runs in simulation: five on the small maze for 360 s, three of them with the shipped
reduced-range gain and two with `gain.mode: cluster_size`, and one on the full maze for
720 s. Everything here is simulation only.

Coverage is quoted as free area in square metres rather than as a percentage. The
percentage is not comparable between runs, because a slam_toolbox grid is sized to whatever
the pose graph covers, so a run whose map drifted outward has a larger denominator and
scores lower for having explored more. These six runs ended with grids of 13774 to 17380
cells for the same maze.

Reduced range reached 9.58, 10.44 and 12.52 m2. Cluster size reached 9.92 and 9.84. The
means differ by 0.97 m2 and the spread inside the reduced-range set alone is 2.94 m2, so
the two cannot be told apart on this evidence and the notes do not claim they can. Anyone
repeating this should measure the run-to-run variance first and pick the number of runs
from it. Three per configuration is not enough here.

The more useful result is why every run stopped. All six parked with frontier clusters
still on the map, 10, 15, 17, 14, 10 and 4 of them, and in all six the parking cycle had
candidates scored and planned and every plan failed. Rebuilding each run's final planning
grid at different inflation radii and flood filling from the parking pose, under the
planner's own rule that unknown space may only be crossed in runs shorter than
`rrt.unknown_lookahead`, splits them into two cases.

In four runs no remaining frontier was reachable even with the collar removed completely.
The reachable sets at zero inflation were 98, 152, 475 and 2136 cells. Those runs were
closed in by cells slam_toolbox had marked occupied, and no inflation setting would have
changed them. The despeckling filter removes isolated spurious cells and what closes a
corridor is a band of adjacent ones, which it correctly leaves alone.

In the other two the map was open and the collar closed it. The full-maze run had two of
its four remaining goals reachable with no collar, one with a 0.10 m collar, and none with
the shipped 0.15 m one, where the robot's reachable set fell from 2025 cells to 122.

That is the cost of the answer in section 2, and it is a tradeoff rather than a defect. A
0.105 m radius is right for a 0.105 m robot and `ceil()` makes it a 0.15 m collar, and a
thinner collar is thinner than the robot. What it does say is that the margin is covering
the real footprint and the map error with one number, and separating those two is where
the improvement is.

## 13. Distance driven is not the length of the pose estimate's path

Worth knowing before anyone plots a run or quotes a distance from one.

`plot_exploration.py` composes the robot's path in the map frame from tf, rather than
reading /odom, because /odom is in the odom frame and that frame drifts. That much is
necessary and not sufficient. slam_toolbox rewrites the map to odom transform every time
the scan matcher closes a loop, and the composed pose then steps to a new place without the
robot having moved. The largest such step measured here was 1.74 m in a single sample.

Drawn as one polyline those steps become straight lines across the map, through walls,
joining two points the robot was never between. Summed as distance they are counted as
driving. In one 360 s run that was 8.66 m of an apparent 26.73 m.

The bound that separates them is physical rather than tuned. A Burger tops out at 0.22 m/s
and base_footprint arrives at about 50 Hz, so real motion cannot exceed roughly 4.4 mm per
sample, and ordinary driving in these runs sits near 3 mm. Checked across six runs, every
composed step above three times that bound coincided with a map to odom change on the same
sample, 43 of 43 in one run and 28 of 28 in another. None of them was motion.

So `robot_track` returns the track as segments, broken wherever a step exceeds that bound.
The figures draw each segment separately, which leaves a visible gap at every correction,
and that gap is the honest thing to show: the position estimate really did jump, and
nothing observed where the robot went in between. `track_length` sums inside segments only.

The general form, for any plot made from a pose estimate under SLAM: summing consecutive
samples measures how far the estimate moved, which equals distance travelled only when the
frame it is expressed in holds still.

## 14. How to tell whether a run was actually safe

A map figure with a route on it cannot answer this, and it is the question a demonstrator
will ask. Three different things put the route on top of a black cell.

The frame, which section 13 covers. The map moving under the route, because the figure
shows the final map while the route was recorded against the map as it stood at each
moment, and slam_toolbox rewrites cells behind the robot all run. And a real collision.

Measured on the full maze run: 7.3 percent of pose samples sit in an occupied cell of the
final map, but only 0.7 percent did in the map as it was at the time, and 0.4 percent in
both. So most apparent wall crossings are cells that became walls after the robot had
passed. Re-expressing the route in the final map's frame does not improve it, which is how
you know it is not a frame problem.

The raw scan looks like the way to settle it, since it is in the robot's frame and cares
about neither the map nor the pose estimate. It is not. The closest return in these runs is
0.120 m against a 0.113 m robot radius, which looks conclusive until you read `range_min`
in the message: 0.12 m. The scanner cannot report anything closer than that, so a minimum
sitting on it is a reading of the sensor limit and not of the world.

What does settle it is clearance from the robot's centre to the nearest occupied cell of
the map as it was at the time, from a distance transform. Full maze: median 0.200 m, with
3.1 percent of samples closer than the robot's own radius. Small maze: median 0.158 m and
15.4 percent. Those violations bunch into short windows rather than spreading through the
run, and cross-track error against the active path stays near the 0.10 m waypoint spacing
throughout, so the follower is tracking accurately down a corridor that the map is closing
around it.

Worth producing per run, rather than a map with a line on it: the map with the inflation
collar drawn explicitly, the route with every footprint violation marked, coverage against
time, and clearance against time with the robot radius and the collar drawn on it.

## 15. The follower's turn rate is not a free parameter

The follower turns at up to 1.0 rad/s and the scanner runs at 5 Hz, so the robot rotates
about 11 degrees between consecutive scans, which is most of slam_toolbox's 20 degree
coarse angular search window. That is a reasonable hypothesis for why the map degrades
where the robot turns most, and `max_w` is a declared parameter on the follower, so it can
be tested without editing that node. The launch file exposes it as `follower_max_w` and
defaults to the course's 1.0.

Halving it to 0.5 rad/s made everything worse, not better. Free area fell to 7.99 m2 from a
9.58 to 12.52 m2 baseline, clearance violations rose to 75.3 percent from 6.7 and 15.4, and
median clearance fell to zero, with the robot's own cell marked occupied from 106 s to the
end of the run.

The probable reason is in the follower's control law: angular velocity is proportional to
heading error and then clipped at `max_w`. Lowering the clip does not make the robot turn
more gently along its path. It makes the robot unable to correct heading fast enough at a
corner, so it leaves the corridor. Anyone tempted by this parameter should know it has been
tried once and measured, and that lowering it is not the improvement it looks like.
