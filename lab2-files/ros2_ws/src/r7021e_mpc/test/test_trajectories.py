import math

import pytest

from r7021e_mpc.trajectories import CircleTrajectory


def make(laps=2):
    return CircleTrajectory(radius=0.8, centre_x=0.0, centre_y=0.0,
                            lap_period=60.0, laps=laps, start_delay=8.0)


def test_starts_one_radius_from_the_centre():
    x, y, _ = make().point_at(0.0)
    assert (x, y) == (pytest.approx(0.8), pytest.approx(0.0))


def test_every_point_sits_on_the_circle():
    circle = make()
    for t in range(0, 130, 5):
        x, y, _ = circle.point_at(float(t))
        assert math.hypot(x, y) == pytest.approx(0.8)


def test_holds_still_during_the_start_delay():
    circle = make()
    assert circle.point_at(0.0)[:2] == pytest.approx(circle.point_at(7.9)[:2])
    assert circle.point_at(7.9)[2] is False


def test_a_lap_returns_to_the_start():
    circle = make()
    assert circle.point_at(8.0)[:2] == pytest.approx(circle.point_at(68.0)[:2], abs=1e-9)


def test_finishes_and_holds_the_final_point():
    circle = make(laps=1)
    x, y, finished = circle.point_at(1000.0)
    assert finished is True
    assert math.hypot(x, y) == pytest.approx(0.8)


def test_path_speed_is_well_under_the_robot_limit():
    # 2*pi*0.8 m in 60 s is 0.084 m/s, leaving room to detour and catch up.
    circle = make()
    x0, y0, _ = circle.point_at(20.0)
    x1, y1, _ = circle.point_at(20.1)
    assert math.hypot(x1 - x0, y1 - y0) / 0.1 == pytest.approx(0.084, abs=0.001)


@pytest.mark.parametrize('radius,lap_period,laps', [(0.0, 60.0, 1), (0.8, 0.0, 1), (0.8, 60.0, 0)])
def test_invalid_parameters_are_rejected(radius, lap_period, laps):
    with pytest.raises(ValueError):
        CircleTrajectory(radius, 0.0, 0.0, lap_period, laps, 0.0)
