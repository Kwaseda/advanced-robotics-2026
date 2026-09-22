"""Unit tests for the figure of eight.

These check the claims the config file makes in its comments. If someone widens the
eight without changing the lap period, the speed assertion below fails and says so,
rather than the robot discovering it by lagging the setpoint for two laps.
"""

import math

import pytest

from r7021e_mpc.trajectories import CircleTrajectory, figure_eight, FigureEightTrajectory

# The defaults in config/trajectory.yaml.
WIDTH = 1.0
HEIGHT = 0.5
LAP_PERIOD = 50.0
LAPS = 2
START_DELAY = 3.0

MAX_V = 0.22
MAX_W = 2.84


def make(**kwargs):
    args = {
        'width': WIDTH, 'height': HEIGHT, 'centre_x': 0.0, 'centre_y': 0.0,
        'lap_period': LAP_PERIOD, 'laps': LAPS, 'start_delay': START_DELAY,
    }
    args.update(kwargs)
    return FigureEightTrajectory(**args)


def test_the_curve_crosses_itself_at_the_centre():
    # Phase 0 and phase pi are the same point. That crossing is what makes it an
    # eight rather than an oval.
    assert figure_eight(0.0, WIDTH, HEIGHT) == pytest.approx((0.0, 0.0), abs=1e-12)
    assert figure_eight(math.pi, WIDTH, HEIGHT) == pytest.approx((0.0, 0.0), abs=1e-12)


def test_the_curve_stays_inside_its_stated_box():
    # The clearance argument against the box world's walls rests on this: the eight
    # spans exactly 2*width by 2*height and never leaves it.
    for i in range(2001):
        x, y = figure_eight(2.0 * math.pi * i / 2000.0, WIDTH, HEIGHT)
        assert abs(x) <= WIDTH + 1e-12
        assert abs(y) <= HEIGHT + 1e-12


def test_the_centre_offset_moves_the_whole_figure():
    x, y = figure_eight(0.5 * math.pi, WIDTH, HEIGHT, centre_x=2.0, centre_y=-1.0)
    assert (x, y) == pytest.approx((3.0, -1.0), abs=1e-12)


def test_a_zero_or_negative_lap_period_is_rejected():
    with pytest.raises(ValueError):
        make(lap_period=0.0)
    with pytest.raises(ValueError):
        make(laps=0)


def test_the_setpoint_holds_still_during_the_start_delay():
    trajectory = make()
    assert trajectory.point_at(0.0)[:2] == pytest.approx(trajectory.point_at(2.9)[:2])


def test_the_run_finishes_after_the_requested_laps_and_then_holds():
    trajectory = make()
    assert trajectory.duration == pytest.approx(START_DELAY + LAPS * LAP_PERIOD)
    assert not trajectory.point_at(trajectory.duration - 0.1)[2]
    assert trajectory.point_at(trajectory.duration)[2]
    # It keeps returning the final point rather than falling silent, so the controller
    # has something to hold position against.
    assert trajectory.point_at(trajectory.duration + 500.0)[:2] == pytest.approx(
        trajectory.point_at(trajectory.duration)[:2]
    )


def test_the_setpoint_never_moves_faster_than_the_robot_can_drive():
    # The claim in config/trajectory.yaml: peak setpoint speed 0.175 m/s, which is 80
    # percent of the Burger's limit. A setpoint that outruns the robot turns the
    # trajectory plot into a picture of lag rather than of tracking.
    trajectory = make()
    step = 0.01
    peak = 0.0
    t = START_DELAY
    while t < START_DELAY + LAP_PERIOD:
        x0, y0, _ = trajectory.point_at(t)
        x1, y1, _ = trajectory.point_at(t + step)
        peak = max(peak, math.hypot(x1 - x0, y1 - y0) / step)
        t += step
    assert peak == pytest.approx(0.175, abs=0.005)
    assert peak < MAX_V


def test_the_turn_rate_the_path_demands_stays_inside_the_robot_limit():
    # Task 2's acceptance criterion is that the commanded velocity never saturates.
    # The tightest part of the eight is its crossing, and this is the turn rate the
    # path asks for there.
    trajectory = make()
    step = 0.01
    peak = 0.0
    t = START_DELAY
    while t < START_DELAY + LAP_PERIOD:
        x0, y0, _ = trajectory.point_at(t)
        x1, y1, _ = trajectory.point_at(t + step)
        x2, y2, _ = trajectory.point_at(t + 2 * step)
        h0 = math.atan2(y1 - y0, x1 - x0)
        h1 = math.atan2(y2 - y1, x2 - x1)
        delta = math.atan2(math.sin(h1 - h0), math.cos(h1 - h0))
        peak = max(peak, abs(delta) / step)
        t += step
    assert peak == pytest.approx(0.40, abs=0.02)
    assert peak < MAX_W


class TestCircleTrajectory:
    """Lab 2 task 4. Same interface as the eight, so the node can hold either."""

    def make(self, laps=2, start_delay=8.0):
        return CircleTrajectory(
            radius=0.8, centre_x=0.0, centre_y=0.0,
            lap_period=60.0, laps=laps, start_delay=start_delay)

    def test_phase_zero_is_one_radius_from_the_centre(self):
        x, y = self.make().point_at_phase(0.0)
        assert (x, y) == (pytest.approx(0.8), pytest.approx(0.0))

    def test_every_point_sits_on_the_circle(self):
        trajectory = self.make()
        for step in range(24):
            x, y = trajectory.point_at_phase(2.0 * math.pi * step / 24.0)
            assert math.hypot(x, y) == pytest.approx(0.8)

    def test_the_setpoint_holds_still_during_the_start_delay(self):
        trajectory = self.make()
        assert trajectory.point_at(0.0)[:2] == pytest.approx(trajectory.point_at(7.9)[:2])
        assert trajectory.point_at(0.0)[2] is False

    def test_a_lap_returns_to_the_start(self):
        trajectory = self.make()
        assert trajectory.point_at(8.0)[:2] == pytest.approx(
            trajectory.point_at(68.0)[:2], abs=1e-9)

    def test_it_finishes_and_then_holds_the_final_point(self):
        trajectory = self.make(laps=1)
        assert trajectory.duration == pytest.approx(68.0)
        finished = trajectory.point_at(1000.0)
        assert finished[2] is True
        # Still publishing a goal rather than falling silent, so the controller has
        # something to hold position against.
        assert math.hypot(finished[0], finished[1]) == pytest.approx(0.8)

    @pytest.mark.parametrize('radius,lap_period,laps',
                             [(0.0, 60.0, 1), (0.8, 0.0, 1), (0.8, 60.0, 0)])
    def test_invalid_parameters_are_rejected(self, radius, lap_period, laps):
        with pytest.raises(ValueError):
            CircleTrajectory(radius=radius, centre_x=0.0, centre_y=0.0,
                             lap_period=lap_period, laps=laps, start_delay=0.0)
