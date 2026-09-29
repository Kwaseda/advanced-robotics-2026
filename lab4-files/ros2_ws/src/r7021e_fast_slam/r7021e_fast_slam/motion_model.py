"""Odometry motion model: sampling from it, and evaluating its density.

The three-parameter model of hello-slam's `4_grid_based_slam.ipynb`: the
odometry increment is decomposed into (rot1, trans, rot2) and each component
gets its own Gaussian, with the standard deviation given directly.

    sigmas = [sd_rot1 (rad), sd_trans (m), sd_rot2 (rad)]

The decomposition itself, `odometry_increment`, is plain SE(2) bookkeeping and
lives in `utils.py`. What belongs here is the modelling: putting noise on those
three numbers, and writing down the density that noise implies.

FastSLAM 1.0 only ever samples from this model. Grid-FastSLAM 2.0 also needs
to *evaluate* it at poses that were not drawn from it, because the improved
proposal weights each candidate by likelihood x motion density.
"""
import numpy as np

from .utils import TRANS_EPS, wrap_angle

## Never divide by a zero standard deviation in the density.
_SIGMA_FLOOR = 1e-4


def sample_motion_model_odometry(u, poses, sigmas, rng) -> np.ndarray:
    """Draw one perturbed pose per input pose.
    We need to sample from the odometry motion model for each particle.

    Args:
        u:      (3,) [d_rot1, d_trans, d_rot2]
        poses:  (N, 3) current particle poses
        sigmas: (3,) [sd_rot1, sd_trans, sd_rot2]
        rng:    numpy Generator

    Returns:
        (N, 3) perturbed particle poses

    Noise goes on the increment, not the pose, which gives the banana-shaped
    cloud. Sigmas are floored exactly as in motion_model_log_pdf.
    """
    u = np.asarray(u, dtype=float)
    poses = np.atleast_2d(np.asarray(poses, dtype=float))
    s = np.maximum(np.asarray(sigmas, dtype=float), _SIGMA_FLOOR)
    n = poses.shape[0]

    rot1 = u[0] + rng.normal(0.0, s[0], n)
    trans = u[1] + rng.normal(0.0, s[1], n)
    rot2 = u[2] + rng.normal(0.0, s[2], n)

    heading = poses[:, 2] + rot1
    out = np.empty_like(poses)
    out[:, 0] = poses[:, 0] + trans * np.cos(heading)
    out[:, 1] = poses[:, 1] + trans * np.sin(heading)
    out[:, 2] = wrap_angle(heading + rot2)
    return out


def motion_model_log_pdf(x_cand, x_prev, u, sigmas) -> np.ndarray:
    """log p(x_cand | x_prev, u) for a batch of candidate poses.

    Args:
        x_cand: (N, 3) candidate poses
        x_prev: (3,) previous odometry pose [x, y, theta]
        u:      (3,) [d_rot1, d_trans, d_rot2]
        sigmas: (3,) [sd_rot1, sd_trans, sd_rot2]

    Returns:
        (N,) log probabilities of the candidate poses

    Inverts the composition with the same TRANS_EPS branch as
    utils.odometry_increment, and wraps both angular residuals. This is the
    density of the increment (Probabilistic Robotics Table 5.5); the pose
    density would add a 1/trans Jacobian.
    """
    x_cand = np.atleast_2d(np.asarray(x_cand, dtype=float))
    x_prev = np.asarray(x_prev, dtype=float)
    u = np.asarray(u, dtype=float)
    s = np.maximum(np.asarray(sigmas, dtype=float), _SIGMA_FLOOR)

    dx = x_cand[:, 0] - x_prev[0]
    dy = x_cand[:, 1] - x_prev[1]
    trans_hat = np.hypot(dx, dy)
    rot1_hat = np.where(trans_hat < TRANS_EPS, 0.0,
                        wrap_angle(np.arctan2(dy, dx) - x_prev[2]))
    rot2_hat = wrap_angle(x_cand[:, 2] - x_prev[2] - rot1_hat)

    e1 = wrap_angle(rot1_hat - u[0])
    e2 = trans_hat - u[1]
    e3 = wrap_angle(rot2_hat - u[2])

    log_norm = float(np.sum(np.log(s * np.sqrt(2.0 * np.pi))))
    return (-0.5 * ((e1 / s[0]) ** 2 + (e2 / s[1]) ** 2 + (e3 / s[2]) ** 2)
            - log_norm)
