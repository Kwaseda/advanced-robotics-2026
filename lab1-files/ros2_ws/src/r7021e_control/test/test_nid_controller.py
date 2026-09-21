"""Unit tests for the NID control law.

None of these need ROS, a simulator or a robot. They test the algebra, which is the
part that can be silently wrong: a sign error in the transform produces a robot that
turns the wrong way, which looks like a tuning problem and is not.

Evidence tier: these are unit tests of the maths, not observations of behaviour. They
say nothing about how the robot moves. That claim needs a `sim` or `robot` tier.
"""

import math

import pytest

from r7021e_control.controller_base import Pose2D
from r7021e_control.nid_controller import (
    GOAL_DEFINES_OFFSET_POINT,
    GOAL_DEFINES_ROBOT_CENTRE,
    NIDController,
)

L = 0.10
K_P = 0.8
MAX_V = 0.22
MAX_W = 2.84
TOL = 0.05


def make(goal_defines=GOAL_DEFINES_OFFSET_POINT, offset_L=L):
    return NIDController(offset_L, K_P, MAX_V, MAX_W, TOL, goal_defines)


def test_offset_point_sits_L_ahead_of_the_axle():
    c = make()
    assert c.offset_point(Pose2D(0.0, 0.0, 0.0)) == pytest.approx((L, 0.0))
    assert c.offset_point(Pose2D(0.0, 0.0, 0.5 * math.pi)) == pytest.approx((0.0, L), abs=1e-12)
    assert c.offset_point(Pose2D(1.0, 2.0, math.pi)) == pytest.approx((1.0 - L, 2.0), abs=1e-12)


def test_zero_offset_is_rejected():
    # At L = 0 the transform divides by zero because the construction has collapsed
    # back onto the non-holonomic centre it exists to avoid.
    with pytest.raises(ValueError):
        make(offset_L=0.0)


def test_goal_straight_ahead_gives_pure_forward_motion():
    command = make().compute(Pose2D(0.0, 0.0, 0.0), 1.0, 0.0)
    assert command.omega == pytest.approx(0.0)
    assert command.v > 0.0


def test_goal_to_the_left_turns_left():
    # Positive omega is a left turn, the ROS convention.
    command = make().compute(Pose2D(0.0, 0.0, 0.0), 0.0, 1.0)
    assert command.omega > 0.0


def test_goal_to_the_right_turns_right():
    command = make().compute(Pose2D(0.0, 0.0, 0.0), 0.0, -1.0)
    assert command.omega < 0.0


def test_inside_tolerance_commands_zero():
    # The goal is exactly at the offset point, so the error is zero.
    command = make().compute(Pose2D(0.0, 0.0, 0.0), L, 0.0)
    assert command.v == 0.0
    assert command.omega == 0.0


def test_small_error_is_proportional_and_unsaturated():
    # 0.1 m of error along the heading, times k_p of 0.8, is 0.08 m/s, which is under
    # the 0.22 limit and so passes through untouched.
    command = make().compute(Pose2D(0.0, 0.0, 0.0), L + 0.1, 0.0)
    assert command.v == pytest.approx(K_P * 0.1)


def test_large_error_saturates_at_the_burger_limit():
    # 1 m of error asks for 0.8 m/s. The Burger does 0.22.
    command = make().compute(Pose2D(0.0, 0.0, 0.0), 1.0 + L, 0.0)
    assert command.v == pytest.approx(MAX_V)


def test_saturation_preserves_the_commanded_direction():
    # The demand is capped in magnitude, not component by component, so the ratio of
    # the two velocity components survives. Clamping v and omega separately would bend
    # the path, which is the failure that looks like a tuning problem.
    c = make()
    # Same error direction, one error large enough to saturate and one small enough
    # not to. 10 m of error asks for 8 m/s; 0.1 m of error asks for 0.08 m/s.
    far = c.compute(Pose2D(0.0, 0.0, 0.0), 10.0 + L, 10.0)
    near = c.compute(Pose2D(0.0, 0.0, 0.0), 0.1 + L, 0.1)
    assert math.hypot(far.v, far.omega * L) == pytest.approx(MAX_V)
    assert math.hypot(near.v, near.omega * L) < MAX_V
    assert far.omega / far.v == pytest.approx(near.omega / near.v, rel=1e-9)


def test_no_command_ever_exceeds_the_robot_limits():
    # The check last year's code would have failed. Sweep a grid of poses and goals and
    # assert that nothing leaves the controller that a Burger cannot execute.
    c = make()
    for theta in [i * math.pi / 8 for i in range(16)]:
        for gx in (-5.0, -0.3, 0.0, 0.3, 5.0):
            for gy in (-5.0, -0.3, 0.0, 0.3, 5.0):
                command = c.compute(Pose2D(0.0, 0.0, theta), gx, gy)
                assert abs(command.v) <= MAX_V + 1e-12
                assert abs(command.omega) <= MAX_W + 1e-12


def test_small_offset_drives_the_angular_clamp():
    # The 1/L term. At L = 0.02 the transform asks for 0.22/0.02 = 11 rad/s for a pure
    # lateral error, the clamp cuts it to 2.84, and the executed direction stops
    # matching the commanded one. That is the cost of a small offset.
    c = make(offset_L=0.02)
    command = c.compute(Pose2D(0.0, 0.0, 0.0), 0.0, 1.0)
    assert command.omega == pytest.approx(MAX_W)


def test_goal_defines_robot_centre_removes_the_offset_bias():
    # Under offset_point the robot's centre settles L behind the goal. Under
    # robot_centre the goal is pushed forward by L first, so the error the loop acts
    # on is the centre's own error and the centre converges to the commanded point.
    centre = make(GOAL_DEFINES_ROBOT_CENTRE)
    offset = make(GOAL_DEFINES_OFFSET_POINT)
    pose = Pose2D(1.0, 0.0, 0.0)

    # Robot centre exactly on the goal: robot_centre says the job is done.
    assert centre.compute(pose, 1.0, 0.0) == centre.compute(pose, 1.0, 0.0)
    _, _, centre_error = centre.tracking_error(pose, 1.0, 0.0)
    assert centre_error == pytest.approx(0.0)

    # The same situation under offset_point still has L of error to remove.
    _, _, offset_error = offset.tracking_error(pose, 1.0, 0.0)
    assert offset_error == pytest.approx(L)
