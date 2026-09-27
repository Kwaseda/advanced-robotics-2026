"""Desk tests for the following law.

The three that matter are the three the supplied law gets wrong: arrival, the
speed profile through a bend, and what happens with no path at all.
"""

import math

from r7021e_rrt.path_follower import (Command, FollowerLimits, advance_index,
                                      angle_difference, follow,
                                      forward_clearance, reactive_limit)

LIMITS = FollowerLimits()


def test_empty_path_stops_and_reports_arrival():
    assert follow([], (0.0, 0.0), 0.0, LIMITS) == Command(0.0, 0.0, 0, True)


def test_single_waypoint_under_the_robot_is_a_stop():
    # This is the navigation node's park manoeuvre. Against the supplied law it
    # is a rotation to world east; against this one it is a stop.
    command = follow([(0.0, 0.0)], (0.0, 0.0), 1.3, LIMITS)
    assert command.arrived
    assert (command.v, command.w) == (0.0, 0.0)


def test_park_stays_stopped_at_any_heading():
    for yaw in (-3.0, -1.0, 0.0, 0.7, 3.1):
        command = follow([(0.05, -0.02)], (0.06, -0.01), yaw, LIMITS)
        assert (command.v, command.w) == (0.0, 0.0), yaw


def test_aligned_robot_drives_at_the_limit():
    command = follow([(1.0, 0.0), (2.0, 0.0)], (0.0, 0.0), 0.0, LIMITS)
    assert command.v == LIMITS.max_v
    assert command.w == 0.0


def test_heading_error_tapers_speed_rather_than_killing_it():
    # The supplied law zeroes linear velocity above 0.3 rad. At 0.5 rad this one
    # keeps cos(0.5), which is most of it, and drives an arc.
    command = follow([(1.0, 0.0), (2.0, 0.0)], (0.0, 0.0), -0.5, LIMITS)
    assert command.v == LIMITS.max_v * math.cos(0.5)
    assert command.w > 0.0


def test_large_error_still_turns_in_place():
    command = follow([(0.0, 1.0), (0.0, 2.0)], (0.0, 0.0), -math.pi / 2, LIMITS)
    assert command.v == 0.0
    assert command.w == LIMITS.max_w


def test_angular_velocity_is_clamped_both_ways():
    left = follow([(0.0, 1.0), (0.0, 2.0)], (0.0, 0.0), 0.0, LIMITS)
    right = follow([(0.0, -1.0), (0.0, -2.0)], (0.0, 0.0), 0.0, LIMITS)
    assert left.w == LIMITS.max_w
    assert right.w == -LIMITS.max_w


def test_final_approach_slows_down():
    near = follow([(0.09, 0.0)], (0.0, 0.0), 0.0, LIMITS)
    assert 0.0 < near.v < LIMITS.max_v
    assert not near.arrived


def test_index_advances_past_consumed_waypoints_and_never_retreats():
    path = [(0.0, 0.0), (0.1, 0.0), (0.5, 0.0), (1.0, 0.0)]
    assert advance_index(path, (0.0, 0.0), 0, 0.2) == 2
    # Standing next to an early waypoint again does not rewind the index.
    assert advance_index(path, (0.0, 0.0), 2, 0.2) == 2


def test_index_stops_at_the_last_waypoint():
    path = [(0.0, 0.0), (0.01, 0.0)]
    assert advance_index(path, (0.0, 0.0), 0, 0.2) == 1


def test_a_path_that_doubles_back_keeps_its_tail():
    # Out and back along the same line. Passing the far end on the way out must
    # not consume the return leg, which popping from the front would do.
    path = [(0.0, 0.0), (0.5, 0.0), (1.0, 0.0), (0.5, 0.0), (0.0, 0.0)]
    assert advance_index(path, (0.0, 0.0), 0, 0.2) == 1


def test_angle_difference_takes_the_short_way_round():
    assert math.isclose(angle_difference(3.0, -3.0), 2 * math.pi - 6.0)
    assert math.isclose(angle_difference(-3.0, 3.0), 6.0 - 2 * math.pi)
    assert math.isclose(angle_difference(0.1, 0.4), 0.3)


def test_speed_never_goes_negative_at_the_threshold():
    for error in (1.19, 1.2, 1.5, 3.0):
        command = follow([(math.cos(error), math.sin(error)), (2.0, 0.0)],
                         (0.0, 0.0), 0.0, LIMITS)
        assert command.v >= 0.0, error


def test_reactive_limit_leaves_a_clear_path_alone():
    command = Command(0.15, 0.4, 0, False)
    assert reactive_limit(command, float('inf'), LIMITS) == command
    assert reactive_limit(command, 0.9, LIMITS) == command
    assert reactive_limit(command, LIMITS.slow_distance, LIMITS) == command


def test_reactive_limit_stops_forward_motion_at_the_threshold():
    command = Command(0.15, 0.4, 0, False)
    for clearance in (LIMITS.stop_distance, 0.15, 0.12, 0.0):
        limited = reactive_limit(command, clearance, LIMITS)
        assert limited.v == 0.0, clearance
        # The turn survives. A robot that cannot turn away from a wall it has
        # stopped in front of has not stopped, it has parked against it.
        assert limited.w == command.w


def test_reactive_limit_tapers_between_the_two_distances():
    command = Command(0.15, 0.0, 0, False)
    mid = (LIMITS.stop_distance + LIMITS.slow_distance) / 2
    assert math.isclose(reactive_limit(command, mid, LIMITS).v, 0.075)
    near = reactive_limit(command, 0.20, LIMITS).v
    far = reactive_limit(command, 0.28, LIMITS).v
    assert 0.0 < near < far < command.v


def test_reactive_limit_never_touches_a_stationary_or_reversing_command():
    for v in (0.0, -0.05):
        command = Command(v, 1.0, 0, False)
        assert reactive_limit(command, 0.05, LIMITS) == command


def test_forward_clearance_reads_only_what_is_ahead():
    # 360 beams at one degree, obstacle dead ahead at 0.4 m and a much closer one
    # directly behind, which must be ignored.
    ranges = [3.0] * 360
    ranges[0] = 0.4
    ranges[180] = 0.05
    got = forward_clearance(ranges, -math.pi, math.radians(1.0), LIMITS,
                            0.12, 3.5)
    assert math.isclose(got, 3.0, abs_tol=0.01)   # index 0 is behind here
    got = forward_clearance(ranges, 0.0, math.radians(1.0), LIMITS, 0.12, 3.5)
    assert math.isclose(got, 0.4)   # index 0 is dead ahead for this one


def test_forward_clearance_keeps_a_reading_on_the_range_minimum():
    # The simulator clamps a hit closer than range_min to range_min, so 0.12 m
    # means "this close or closer" and is the reading that matters most.
    ranges = [3.0] * 360
    ranges[2] = 0.12
    got = forward_clearance(ranges, 0.0, math.radians(1.0), LIMITS, 0.12, 3.5)
    assert math.isclose(got, 0.12, abs_tol=1e-3)


def test_forward_clearance_discards_out_of_range_readings():
    ranges = [float('inf')] * 360
    ranges[1] = float('nan')
    ranges[2] = 9.0
    ranges[3] = 0.01
    got = forward_clearance(ranges, 0.0, math.radians(1.0), LIMITS, 0.12, 3.5)
    assert got == float('inf')


def test_an_empty_strip_reads_as_clear_not_as_blocked():
    assert forward_clearance([], 0.0, 0.01, LIMITS) == float('inf')


def test_a_corner_off_to_one_side_of_the_path_counts():
    """A corner at 45 to 55 degrees is outside a forward cone and still in the way."""
    ranges = [3.0] * 360
    for i in range(-20, 21):
        ranges[i % 360] = 0.70
    ranges[-45 % 360] = 0.155
    ranges[-53 % 360] = 0.120
    got = forward_clearance(ranges, 0.0, math.radians(1.0), LIMITS, 0.12, 3.5)
    assert got <= LIMITS.stop_distance


def test_a_corridor_wall_beside_the_robot_does_not_stop_it():
    ranges = [3.0] * 360
    ranges[90] = ranges[270] = 0.30
    ranges[40] = 0.45
    got = forward_clearance(ranges, 0.0, math.radians(1.0), LIMITS, 0.12, 3.5)
    assert got >= LIMITS.slow_distance


def test_a_wall_being_slid_past_does_not_read_as_ahead():
    """A wall 0.118 m to the side in a dead end must not stop forward motion."""
    ranges = [3.0] * 360
    for bearing in range(66, 89):
        ranges[bearing] = 0.12
    got = forward_clearance(ranges, 0.0, math.radians(1.0), LIMITS, 0.12, 3.5)
    assert got > LIMITS.stop_distance
