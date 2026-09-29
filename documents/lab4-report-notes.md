# Lab 4 design notes: Grid-FastSLAM 2.0

Notes for the report and for the individual oral assessment. Each section is one
decision or one result, with the reason and the evidence. All results are from Gazebo
replays of one recorded drive through the course maze (46 m, 372 s, a gentle teleop-style
loop out and back), unless stated otherwise.

## 1. What is ours and what is the course's

The course package supplies the ROS node (`grid_slam_node.py`), the occupancy grid and
inverse sensor model (`grid_map.py`), the scan matcher, the configuration dataclass, the
run log and the plotting code. The five modules the lab asks for are ours:
`motion_model.py`, `measurement_model.py`, `proposal.py`, `resampling.py` and the filter
step in `rbpf.py`. We changed one given file, `grid_slam_node.py`, in one place (section
10). `r7021e_fast_slam_sim` (the Gazebo maze), the three tools
(`offline_replay.py`, `offline_sweep.sh`, `check_ground_truth.py`) and the tests are
additions.

## 2. Motion model: one noise model in two forms

The increment is (rot1, trans, rot2), with independent Gaussian noise per component.
`sample_motion_model_odometry` perturbs the increment and replays it from each particle's
pose. `motion_model_log_pdf` inverts the replay for a candidate pose and scores the
residuals with the same Gaussians. Both floor the sigmas identically, both wrap angular
residuals, and the density mirrors `odometry_increment`'s near-zero-translation branch.
If the two forms disagreed, the proposal would be fitted to a different distribution than
the weights assume, with nothing visible to show it. Two tests enforce the agreement: the
density recovers the sampler's own noise to 1e-9, and it tracks a two-million-sample
histogram bin by bin.

The density is that of the increment (Probabilistic Robotics Table 5.5). The density of
the pose would add a 1/trans Jacobian; the textbook model and the lab omit it.

## 3. Measurement model: likelihood field, off-map beams penalised

One distance transform per call, then a lookup per beam, vectorised over all K candidate
poses. Each beam scores z_hit·exp(-d²/2σ²) + z_rand/z_max, summed in log space. Off-map
endpoints score as max_dist, the worst case, rather than being dropped, so a pose cannot
gain by throwing beams off the map. Max-range returns are removed before scoring: they
map free space but say nothing about where a surface is.

We use sigma_hit 0.05 (one cell) and max_dist 0.15 (3 sigma) instead of the shipped 0.10
and 0.10, which clipped the Gaussian at one sigma. Five seeds each, N = 10:

| sigma_hit / max_dist | FS2 ATE | FS1 ATE |
|---|---|---|
| 0.05 / 0.15 (ours) | 0.19 ± 0.05 m | 0.36 ± 0.15 m |
| 0.10 / 0.10 (shipped) | 0.71 ± 0.09 m | 0.64 ± 0.23 m |
| 0.10 / 0.20 | 0.47 ± 0.27 m | 0.88 ± 0.13 m |

A sharper likelihood gives the improved proposal a sharper peak to fit.

## 4. Improved proposal

- **Lattice.** K = num_candidates floored to a perfect cube, counted with integers
  rather than by rounding a cube root. The window parameters are the full width (the
  lattice spans ±w/2). This differs from the scan matcher, whose windows are half-widths.
- **tau** = measurement log-likelihood + motion log-density, per candidate.
- **log eta** = logsumexp over all candidates: the K-point approximation of
  ∫ p(z|x,m) p(x|x_prev,u) dx, which is what the importance weight of the optimal
  proposal is. It is not tau at the sampled pose.
- **mu**: the weighted mean, with a circular mean for the heading.
- **Sigma**: the weighted scatter with the heading residual wrapped, plus a small diagonal,
  symmetrised, and eigenvalues floored at 1e-6. The regulariser matters: with a sharp
  likelihood, tau often sits almost entirely on one candidate, and the scatter matrix is
  then singular.

## 5. Resampling

N_eff = 1/Σw². Systematic resampling: one uniform draw, N pointers spaced 1/N apart, and
`searchsorted` with `side='right'` so a zero-weight particle can never be selected. It
returns ancestor indices only; the given `_maybe_resample` rebuilds the set with a deep
copy of every grid.

## 6. The filter step and its three traps

Per particle: predict, scan-match against the particle's own map, fit the proposal and
sample from it, add log eta to the weight, and only then integrate the scan. If the match
score is below `match_score_min` (an unexplored map scores 0.5 everywhere), the particle
takes a FastSLAM 1.0 step instead: keep the odometry sample, weight by the likelihood.

The three ways to get a convincing map from a broken filter each have a test that fails
if the ordering is wrong:

1. **Map integrated last.** Every likelihood and scan-match call must see the pre-step
   grid, bit for bit.
2. **Resampling deep-copies.** Survivors of one ancestor own distinct grids; changing one
   leaves the others untouched.
3. **Pre-resample N_eff.** The logged value is the one that made the decision, not N.
   `best_index` is also read before resampling.

Thirteen deliberately planted bugs (for example log eta as a max, an arithmetic heading
mean, integrating first, sharing grids) were each caught by the suite.

The fallback branch weights a particle by p(z | x_bar) instead of eta. The two are on
different scales (eta sums 27 terms that include the motion density). The fallback is
part of the course's own sketch of the step. It is FastSLAM 1.0 for that particle, so its
weight is the right one for the proposal it actually used. In practice the branches
almost never mix: all particles fall back together on the empty first map, and on about
0.1% of particle-steps afterwards.

## 7. Configuration checks

Values that run without error but produce a blank map or identical particles are refused
at startup, with the reason: p_occ ≤ 0.5, p_free ≥ 0.5, a clamp below logit(occ_thresh),
a non-cube or too small num_candidates, zero windows, zero odometry sigmas,
max_dist < 2·sigma_hit, and range_max ≤ 0.

## 8. Task 1: the grid parameters, one at a time

Mapping with clean odometry, one change at a time from the tuned set:

| Change | Occupied cells | Free cells |
|---|---|---|
| all shipped values | 0 | 0 |
| tuned | 3045 | 9128 |
| `log_odds_limit: 0` | 0 | 0 |
| `p_occ: 0.5` | 0 | 10787 |
| `hit_window_cells: 0` | 0 | 11509 |
| `hit_window_cells: 6` | 6084 | 7438 |
| `use_clamping: false` | 3399 | 8815 (largest cell magnitude 5935) |
| `map_resolution: 0.10` | 1321 | 1970 |

The shipped set gives a blank map: `log_odds_limit: 0` with clamping on pins every cell at
0.5. `hit_window_cells: 0` gives the occupied band no width, so no wall is ever marked.
Without clamping, cells become practically irreversible.

## 9. The drive decides whether ATE means anything

The node logs Gazebo's clean /odom as ground truth. On exploration runs with fast spins
in place, Gazebo's wheels slipped and clean /odom was up to 1.6 m from the true pose, as
large as every error being measured. On a gentle teleop-style loop it stayed within
0.048 m over 46 m. `tools/check_ground_truth.py` checks a recorded bag, and the guide
explains how to drive so it passes.

## 10. Separate random generator for the injected noise

The node used one generator for both the odometry corruption and the filter. The
corrupted odometry therefore depended on the filter's own draws, and FS1 and FS2 replaying
the same bag at the same seed saw different inputs. The corruption now has its own
generator (node parameter `odom_noise_seed`, default 0). A test shows FS1 at N = 7 and
FS2 at N = 3 now see identical corrupted odometry.

## 11. Three parameters that only running could set

- **`update_max_rate` stays at 10.** At the scan rate (5), scan stamps jitter just under
  0.2 s and the gate rejects them: only 33% of steps were 0.2 s apart and ATE rose from
  0.238 to 0.351 m.
- **`odom_noise_sigma` 0.003 per message.** The noise is applied per /odom message
  (50 Hz), about 10 per filter step. 0.01 made the per-step error as large as the motion.
- **`num_particles` 10.** One step takes 64 ms at N = 5, 114 ms at N = 10 and 244 ms at
  N = 20 (idle desktop, single process). N = 20 cannot keep up with the 5 Hz scanner.

## 12. Tuning the Task 7 parameters

Every parameter in Task 7's list, moved in both directions, five seeds each, FS2, N = 10:

| Setting | ATE |
|---|---|
| final configuration | 0.19 ± 0.05 m |
| odometry_sigmas 0.015 / 0.03 | 0.24 / 0.51 m |
| scan-match window 0.10/0.05 / 0.30/0.15 | 0.37 / 0.71 m |
| num_candidates 125 | 0.22 m |
| proposal window 0.05/0.02 / 0.20/0.10 | 0.33 / 0.29 m |
| resample_threshold 0.3 | 0.62 m |
| noisy odometry, no filter | 1.47 m |

Nothing beat the starting configuration. A wider scan-match window finds look-alike
corridors. A wider motion belief spreads the particles and weakens the prior that stops
the matcher jumping. A lower resample threshold lets good particles drift longer before
they are copied.

The spread between seeds was often as large as the difference between settings; single
runs repeatedly pointed the wrong way.

## 13. Weight degeneracy

Selective resampling fires on most steps (about 90%), with N_eff/N around 0.3. About 300
beams scored as independent make the likelihood extremely peaked: particles a few
centimetres apart differ by tens of nats. That is a property of the model the lab
specifies. Thinning or tempering the beams would reduce it; neither was applied, since
both change the model.

## 14. Task 7 results

Both proposals, N in {1, 5, 10, 20, 50}, three seeds each, one bag, the lab's ATE:

| N | FastSLAM 1.0 | Grid-FastSLAM 2.0 |
|---|---|---|
| 1 | 2.58 ± 0.13 m | 1.41 ± 0.14 m |
| 5 | 0.63 ± 0.19 m | 0.49 ± 0.10 m |
| 10 | 0.30 ± 0.16 m | 0.17 ± 0.04 m |
| 20 | 0.18 ± 0.09 m | 0.20 ± 0.06 m |
| 50 | 0.10 ± 0.04 m | 0.60 ± 0.40 m |

Noisy odometry alone: 1.47 m.

- **Particle-scarce regime (N ≤ 10): FS2 wins.** FS2 at N = 10 matches FS1 at N = 20,
  with a third of the spread. With one particle, FS1 is worse than odometry alone: it adds
  sampling noise, and nothing selects against it.
- **Above N = 10, with K = 27: FS2 stops improving, and FS1 keeps improving.** The
  exact weight factor p(z | x_prev, m, u) depends only on a particle's history, so copies
  of the same ancestor should get the same eta. Their weight increments differed with a
  median std of 3.1 nats (1267 families, N = 20). That is the error of the 27-point sum:
  the lattice is one cell apart, and it is centred wherever each copy's own scan match
  converged. With resampling on nearly every step, more particles select harder on that
  error.
- **The test.** With K = 125 (spread 2.5 nats), FS2 reached 0.085 ± 0.018 m at N = 20 and
  0.093 ± 0.018 m at N = 50, matching or beating FS1 at every N. At N = 10 it bought
  nothing (0.22 vs 0.19 m over five seeds) and would cost 4.6 times the likelihood time,
  so the robot configuration keeps K = 27.
- **Figure 3.** Selective resampling fired on 90% of steps, every-step on 100%, with
  N_eff/N medians of 0.30 and 0.32. The likelihood is too peaked for the policies to
  differ much. At the same seed, every-step resampling gave 0.48 m against 0.17 m.
- **Figure 4.** Time and memory are linear in N. Scan matching and the likelihood take
  about half each, and integration about 6%. Memory is 0.64 MB per particle.

## 15. Task 9, simulated

Replay the drive for 60 s, then feed 10 s of odometry claiming 0.15 m/s forward (1.5 m in
total) while every scan is the unchanging view of a stationary robot. Hardware
parameters, three seeds:

| Filter | Estimate moved | Particle spread |
|---|---|---|
| FS2 | 0.40 to 0.45 m | about 0 |
| FS1 | 0.23 to 0.36 m | about 0 |

The estimate creeps forward at about a third of the claimed speed, and the creep does not
stop. Prediction pushes forward by the encoder motion. Scan matching and the proposal pull
back, but only in steps of about one 5 cm cell, while odometry pushes 3 cm per step. Map
integration then draws the frozen scan at the crept pose, so the map drifts with the
estimate. The update gate does not stop it, because it watches odometry. FS1 resists
slightly better here: its wider sampling sometimes lands near the unmoved pose, and
selection keeps those particles.

## 16. Hardware configuration

`config/params_hardware.yaml` changes three lines from the simulation file: injected
noise to zero, `odometry_sigmas` [0.03, 0.04, 0.03], and the run name. Everything else is
expected to transfer. Any other change needed on the robot is a result worth reporting,
with the symptom that caused it.
