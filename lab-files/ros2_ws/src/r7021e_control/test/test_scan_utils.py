"""Unit tests for reading a LaserScan.

The failure these exist to prevent is the one in wiki/closest-wall-point.md: a raw
min() over the range array returns 0.0 for the whole run, because the lidar reports
0.0 and inf for beams that measured nothing, and the plot is flat and wrong.
"""

import math

import pytest

from r7021e_control.scan_utils import beam_angle, closest_point, is_valid, sector_min

RANGE_MIN = 0.12
RANGE_MAX = 3.5

# Eight beams, 45 degrees apart, starting straight behind the robot. Index 4 is
# straight ahead, index 2 is to the right, index 6 is to the left.
ANGLE_MIN = -math.pi
ANGLE_INCREMENT = math.pi / 4


def find_closest(ranges):
    return closest_point(ranges, ANGLE_MIN, ANGLE_INCREMENT, RANGE_MIN, RANGE_MAX)


def find_sector(ranges, centre, width):
    return sector_min(
        ranges, ANGLE_MIN, ANGLE_INCREMENT, RANGE_MIN, RANGE_MAX, centre, width
    )


def test_beam_angle_indexes_from_angle_min():
    assert beam_angle(0, ANGLE_MIN, ANGLE_INCREMENT) == pytest.approx(-math.pi)
    assert beam_angle(4, ANGLE_MIN, ANGLE_INCREMENT) == pytest.approx(0.0)


def test_zero_and_inf_are_not_measurements():
    assert not is_valid(0.0, RANGE_MIN, RANGE_MAX)
    assert not is_valid(float('inf'), RANGE_MIN, RANGE_MAX)
    assert not is_valid(float('nan'), RANGE_MIN, RANGE_MAX)
    assert not is_valid(0.05, RANGE_MIN, RANGE_MAX)
    assert not is_valid(4.0, RANGE_MIN, RANGE_MAX)
    assert is_valid(1.0, RANGE_MIN, RANGE_MAX)


def test_the_zero_beam_does_not_win():
    # This is the whole point. Without the gate the answer would be 0.0 at index 1.
    ranges = [float('inf'), 0.0, 2.0, 1.0, 3.0, 0.05, 4.0, 2.5]
    distance, bearing, index = find_closest(ranges)
    assert distance == pytest.approx(1.0)
    assert index == 3
    assert bearing == pytest.approx(beam_angle(3, ANGLE_MIN, ANGLE_INCREMENT))


def test_a_scan_with_no_valid_beam_returns_none():
    # None is a real answer, not an error. A caller that reads it as zero puts a
    # phantom wall at the robot's own origin.
    assert find_closest([0.0, float('inf'), 10.0, float('nan')]) is None


def test_sector_ignores_beams_outside_it():
    # The closest beam overall is behind the robot at index 0. A front sector must not
    # report it.
    ranges = [0.3, 2.0, 2.0, 2.0, 1.5, 2.0, 2.0, 2.0]
    front = find_sector(ranges, 0.0, math.radians(60))
    assert front[0] == pytest.approx(1.5)
    assert front[2] == 4


def test_sector_centred_on_the_right_reads_the_right_side():
    ranges = [2.0, 2.0, 0.6, 2.0, 2.0, 2.0, 3.0, 2.0]
    right = find_sector(ranges, -0.5 * math.pi, math.radians(60))
    assert right[0] == pytest.approx(0.6)
    left = find_sector(ranges, 0.5 * math.pi, math.radians(60))
    assert left[0] == pytest.approx(3.0)


def test_sector_straddles_the_wrap_in_a_zero_to_two_pi_scan():
    # A TurtleBot3 scan runs 0 to 2*pi, not -pi to pi, so the front sector straddles
    # the wrap: half of it is near 0 and half near 2*pi. The offset from the sector
    # centre is wrapped rather than the bearing, which is what makes that work.
    # Beam 7 sits at 7*pi/4, which is 45 degrees to the right of straight ahead.
    ranges = [2.0, 2.0, 2.0, 2.0, 0.5, 2.0, 2.0, 0.9]
    front = sector_min(ranges, 0.0, ANGLE_INCREMENT, RANGE_MIN, RANGE_MAX,
                       0.0, math.radians(100))
    # 0.9 at beam 7 is inside the sector. 0.5 at beam 4 is directly behind and is not.
    assert front[0] == pytest.approx(0.9)
    assert front[2] == 7


def test_empty_sector_returns_none():
    # Every beam invalid.
    assert find_sector([0.0] * 8, 0.0, math.radians(10)) is None
    # Valid beams, but none of them inside a sector this narrow pointed between two.
    assert find_sector([2.0] * 8, math.radians(20), math.radians(1)) is None
