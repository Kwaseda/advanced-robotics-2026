"""Unit tests for the wall following law and the loop closure test.

The signs are the thing being tested. A sign error here produces a robot that steers
away from the wall it is meant to follow, which on a lab day looks like a broken lidar
and is not.
"""

import math

import pytest

from r7021e_control.wall_following import (
    LoopCloseDetector,
    wall_geometry,
    WallFollowLaw,
)

SETPOINT = 0.5
K_P = 1.5
SPEED = 0.15
FRONT_STOP = 0.35
CORNER_RATE = 1.0
MAX_ERROR = 0.5
ACQUIRE_GAIN = 1.0
K_HEADING = 1.0
MAX_V = 0.22
MAX_W = 2.84

# A wall 0.5 m off the right side, inside the proportional band. Passed to every
# test that means to exercise the following state rather than the acquiring one.
NEAR = (0.5, -0.5 * math.pi)


def make(side='right'):
    return WallFollowLaw(side, SETPOINT, K_P, K_HEADING, SPEED, FRONT_STOP, CORNER_RATE,
                         MAX_ERROR, ACQUIRE_GAIN, MAX_V, MAX_W)


def test_unknown_side_is_rejected():
    with pytest.raises(ValueError):
        make('starboard')


def test_on_the_setpoint_drives_straight():
    command, state = make().compute(SETPOINT, 2.0, NEAR)
    assert state == 'following'
    assert command.omega == pytest.approx(0.0)
    assert command.v == pytest.approx(SPEED)


def test_too_far_from_a_right_wall_turns_right():
    # Negative omega is a right turn. Too far from the wall has to steer toward it.
    command, _ = make('right').compute(SETPOINT + 0.2, 2.0, NEAR)
    assert command.omega < 0.0
    assert command.omega == pytest.approx(-K_P * 0.2)


def test_too_close_to_a_right_wall_turns_left():
    command, _ = make('right').compute(SETPOINT - 0.2, 2.0, NEAR)
    assert command.omega > 0.0


def test_the_left_side_mirrors_the_right():
    right, _ = make('right').compute(SETPOINT + 0.2, 2.0, NEAR)
    left, _ = make('left').compute(SETPOINT + 0.2, 2.0, NEAR)
    assert left.omega == pytest.approx(-right.omega)
    assert left.v == pytest.approx(right.v)


def test_an_obstacle_in_front_stops_and_pivots_away_from_the_wall():
    command, state = make('right').compute(SETPOINT, FRONT_STOP - 0.01, NEAR)
    assert state == 'corner'
    assert command.v == 0.0
    # Following the right wall, a corner is turned to the left.
    assert command.omega == pytest.approx(CORNER_RATE)

    command, _ = make('left').compute(SETPOINT, FRONT_STOP - 0.01, NEAR)
    assert command.omega == pytest.approx(-CORNER_RATE)


def test_a_lost_wall_arcs_back_toward_it_rather_than_driving_on():
    # At an outside corner the wall genuinely leaves the side sector for a second or
    # two. Treating that as the largest allowed error turns the robot back toward it.
    command, state = make('right').compute(None, 2.0, NEAR)
    assert state == 'following'
    assert command.omega == pytest.approx(-K_P * MAX_ERROR)
    assert command.v == pytest.approx(SPEED)


def test_a_distant_wall_does_not_produce_a_spin():
    # The error is capped, so a wall 3 m away asks for the same turn as one 1 m away
    # and never for the 2.84 rad/s the actuator bound would otherwise allow.
    command, _ = make().compute(SETPOINT + 3.0, 4.0, NEAR)
    assert abs(command.omega) == pytest.approx(K_P * MAX_ERROR)


def test_no_command_ever_exceeds_the_robot_limits():
    law = make()
    for side in (None, 0.0, 0.1, 0.5, 1.0, 10.0):
        for front in (None, 0.0, 0.2, 0.35, 5.0):
            command, _ = law.compute(side, front, NEAR)
            assert abs(command.v) <= MAX_V + 1e-12
            assert abs(command.omega) <= MAX_W + 1e-12


def test_the_follow_speed_cannot_exceed_the_robot_limit():
    # A YAML file asking for 0.5 m/s on a Burger gets 0.22, not 0.5.
    law = WallFollowLaw('right', SETPOINT, K_P, K_HEADING, 0.5, FRONT_STOP, CORNER_RATE,
                        MAX_ERROR, ACQUIRE_GAIN, MAX_V, MAX_W)
    command, _ = law.compute(SETPOINT, 2.0, NEAR)
    assert command.v == pytest.approx(MAX_V)


def test_loop_does_not_close_on_the_first_sample():
    # The robot starts within any tolerance of its own start position.
    detector = LoopCloseDetector(0.2, 2.0, math.radians(45))
    assert not detector.update(0.0, 0.0, 0.0)
    assert not detector.update(0.01, 0.0, 0.0)


def test_loop_closes_after_a_full_circle():
    detector = LoopCloseDetector(0.2, 2.0, math.radians(45))
    closed_at = None
    for i in range(80):
        angle = i * 0.1
        x, y = math.cos(angle) - 1.0, math.sin(angle)
        # Heading is the tangent to the circle, so it comes back round to the start.
        if detector.update(x, y, angle + 0.5 * math.pi) and closed_at is None:
            closed_at = i
    assert closed_at is not None
    # A unit circle is 6.28 m round, so the 2 m minimum was cleared long before.
    assert detector.travelled > 2.0


def test_a_short_out_and_back_does_not_count_as_a_loop():
    # Driving 0.5 m out and back returns to the start, but has not gone round
    # anything. The minimum travel is what tells the difference.
    detector = LoopCloseDetector(0.2, 2.0, math.radians(45))
    out = [(x * 0.05, 0.0, 0.0) for x in range(11)]
    back = [(x * 0.05, 0.0, math.pi) for x in range(10, -1, -1)]
    assert not any(detector.update(x, y, t) for x, y, t in out + back)


def test_error_to_start_is_the_acceptance_number():
    detector = LoopCloseDetector(0.2, 2.0, math.radians(45))
    detector.update(1.0, 1.0, 0.0)
    assert detector.error_to_start(1.0, 1.15) == pytest.approx(0.15)


def test_from_the_middle_of_a_room_it_drives_at_the_nearest_wall():
    # The defect this state exists for, measured in Gazebo on 2026-09-08. Every wall is
    # beyond the proportional band, the capped error commands a constant turn at a
    # constant speed, and that is a circle. The robot drove a 0.2 m circle for 200 s
    # while every log line said "following".
    law = make('right')
    far_wall_to_the_left = (2.4, 0.5 * math.pi)
    command, state = law.compute(2.4, 2.4, far_wall_to_the_left)
    assert state == 'acquiring'
    # Turning left, toward the wall it can see, rather than right at the capped error.
    assert command.omega > 0.0
    assert command.v == pytest.approx(SPEED)


def test_acquiring_stops_once_a_wall_is_inside_the_band():
    law = make('right')
    assert law.acquire_distance == pytest.approx(SETPOINT + MAX_ERROR)
    just_outside = (law.acquire_distance + 0.01, 0.0)
    just_inside = (law.acquire_distance - 0.01, 0.0)
    assert law.compute(None, 2.0, just_outside)[1] == 'acquiring'
    assert law.compute(None, 2.0, just_inside)[1] == 'following'


def test_acquiring_turn_is_capped_at_the_cornering_rate():
    # A wall directly behind is pi radians of error, which at a gain of 1.0 would ask
    # for 3.14 rad/s. The cap keeps the pivot controlled and inside the actuator bound.
    law = make('right')
    command, state = law.compute(None, None, (3.0, math.pi))
    assert state == 'acquiring'
    assert abs(command.omega) == pytest.approx(CORNER_RATE)


def test_a_corner_still_wins_over_acquiring():
    # Something inside the front stop is a wall to turn away from, whatever the rest of
    # the scan says. Order of the checks, not a coincidence.
    law = make('right')
    command, state = law.compute(2.4, FRONT_STOP - 0.01, (2.4, 0.0))
    assert state == 'corner'
    assert command.v == 0.0


def beam_range(theta, distance, psi):
    """Range of a beam at bearing theta against a straight wall. The ground truth."""
    return distance / math.sin(psi - theta)


def test_wall_geometry_recovers_distance_and_angle_exactly():
    # Generate the two beams from a known wall, then check the fit returns it.
    delta = math.radians(40)
    perpendicular = -0.5 * math.pi
    for distance in (0.4, 0.5, 1.0, 2.0):
        for psi_deg in (-45, -30, -10, 0, 10, 30, 45):
            psi = math.radians(psi_deg)
            a = beam_range(perpendicular, distance, psi)
            b = beam_range(perpendicular + delta, distance, psi)
            d_fit, psi_fit = wall_geometry(a, b, delta)
            assert d_fit == pytest.approx(distance, abs=1e-9)
            assert psi_fit == pytest.approx(psi, abs=1e-9)


def test_heading_into_a_right_wall_turns_away_before_the_distance_says_to():
    # At the setpoint distance the distance term is zero, so anything the law does here
    # comes from the angle. Heading into the wall must turn left.
    law = make('right')
    into_the_wall = (SETPOINT, math.radians(20))
    command, state = law.compute(SETPOINT, 2.0, NEAR, into_the_wall)
    assert state == 'following'
    assert command.omega > 0.0
    assert command.omega == pytest.approx(K_HEADING * math.radians(20))


def test_heading_away_from_a_right_wall_turns_back_toward_it():
    law = make('right')
    away = (SETPOINT, math.radians(-20))
    command, _ = law.compute(SETPOINT, 2.0, NEAR, away)
    assert command.omega < 0.0


def test_the_heading_term_mirrors_on_the_left():
    # The same physical situation, heading into the wall being followed. Both fits
    # report it as a positive psi, because the caller picks the forward beam on the
    # side it follows and the left geometry is already the mirror of the right one.
    # Each side must then turn away from its own wall, which is opposite signs.
    into_the_wall = (SETPOINT, math.radians(20))
    right, _ = make('right').compute(SETPOINT, 2.0, NEAR, into_the_wall)
    left, _ = make('left').compute(SETPOINT, 2.0, NEAR, into_the_wall)
    assert right.omega > 0.0
    assert left.omega == pytest.approx(-right.omega)


def test_the_two_terms_can_cancel():
    # Too far from the wall, but already turning back toward it hard enough. The law
    # should not keep turning, and without the heading term it always would.
    law = make('right')
    distance_error = 0.2
    # Too far from the wall, so the distance term turns toward it; already heading
    # into the wall by just enough that the angle term turns back by the same amount.
    psi = K_P * distance_error / K_HEADING
    command, _ = law.compute(None, 2.0, NEAR, (SETPOINT + distance_error, psi))
    assert command.omega == pytest.approx(0.0, abs=1e-12)


def test_missing_geometry_falls_back_to_distance_only():
    # A dropped beam must not stop the robot steering. It steers worse, for a tick.
    law = make('right')
    command, state = law.compute(SETPOINT + 0.2, 2.0, NEAR, None)
    assert state == 'following'
    assert command.omega == pytest.approx(-K_P * 0.2)


def test_an_out_and_back_along_one_wall_is_not_a_loop():
    # The false positive measured in Gazebo on 2026-09-08. The robot drove out along a
    # wall and came back down the same wall, arriving 4 mm from where it started and
    # pointing the opposite way, and the run stopped after 4.48 m of a 15 m lap.
    detector = LoopCloseDetector(0.2, 2.0, math.radians(45))
    for i in range(41):
        assert not detector.update(i * 0.05, 0.0, 0.0)
    for i in range(40, -1, -1):
        assert not detector.update(i * 0.05, 0.0, math.pi)
    assert detector.travelled > 2.0


def test_the_same_path_closes_if_the_robot_turns_around_at_the_end():
    # Same positions, but arriving on the original heading. That is a loop as far as
    # anything without a map can tell, and the detector has to accept it.
    detector = LoopCloseDetector(0.2, 2.0, math.radians(45))
    closed = False
    for i in range(41):
        detector.update(i * 0.05, 0.0, 0.0)
    for i in range(40, -1, -1):
        closed = detector.update(i * 0.05, 0.0, 0.0) or closed
    assert closed
