"""MPC tests without ROS: roll the closed loop with Euler steps and check the constraints hold."""

import math

import pytest

from r7021e_mpc.mpc_controller import MPCController, inflate_obstacles
from r7021e_mpc.trajectories import CircleTrajectory

T_STEP = 0.1
BIG_BOX = (-2.0, 2.0)


def build(obstacles=(), box=(-1.0, 1.0), n_horizon=10, soft=False):
    return MPCController(
        t_step=T_STEP, n_horizon=n_horizon, max_linear_velocity=0.22,
        max_angular_velocity=0.8, min_linear_velocity=0.0, x_bounds=box, y_bounds=box,
        obstacles=obstacles, q_position=1.0, q_terminal=1.0, r_input=0.01,
        goal_tolerance=0.05, obstacle_soft=soft, obstacle_penalty=1e4)


def roll(controller, goal, steps=200):
    """Closed loop from the origin. Returns the path and the (v, omega) commands."""
    x = y = theta = 0.0
    path, commands = [(x, y)], []
    for _ in range(steps):
        v, omega = controller.compute(x, y, theta, *goal)
        commands.append((v, omega))
        x += T_STEP * v * math.cos(theta)
        y += T_STEP * v * math.sin(theta)
        theta += T_STEP * omega
        path.append((x, y))
        if math.hypot(goal[0] - x, goal[1] - y) < 0.05:
            break
    return path, commands


def clearance(path, obstacle):
    return min(math.hypot(x - obstacle[0], y - obstacle[1]) for x, y in path)


def test_inflation_is_added_to_the_radius():
    assert inflate_obstacles([1.0], [2.0], [0.2], 0.105) == [(1.0, 2.0, pytest.approx(0.305))]


def test_mismatched_obstacle_lists_are_rejected():
    with pytest.raises(ValueError):
        inflate_obstacles([1.0, 2.0], [1.0], [0.2, 0.2], 0.0)


def test_task1_reaches_a_goal_inside_the_box_within_the_input_limits():
    path, commands = roll(build(), (0.8, 0.5))
    assert math.hypot(0.8 - path[-1][0], 0.5 - path[-1][1]) < 0.05
    assert all(0.0 <= v <= 0.22 and abs(w) <= 0.8 for v, w in commands)


def test_task1_never_reverses_for_a_goal_behind():
    _, commands = roll(build(), (-0.6, 0.0))
    assert all(v >= 0.0 for v, _ in commands)


def test_task1b_goal_outside_the_box_stops_on_the_boundary():
    path, _ = roll(build(), (1.5, 0.0), steps=120)
    assert max(x for x, _ in path) <= 1.0 + 1e-3
    assert path[-1][0] == pytest.approx(1.0, abs=0.02)


def test_task2_detours_around_the_obstacle_and_arrives():
    obstacles = inflate_obstacles([0.75], [0.08], [0.15], 0.105)
    path, _ = roll(build(obstacles, BIG_BOX, 20), (1.5, 0.0), steps=400)
    assert clearance(path, obstacles[0]) >= obstacles[0][2] - 1e-3
    assert max(abs(y) for _, y in path) > 0.1
    assert math.hypot(1.5 - path[-1][0], path[-1][1]) < 0.06


def test_obstacle_dead_on_the_line_stalls_on_symmetry():
    obstacles = inflate_obstacles([0.75], [0.0], [0.15], 0.105)
    path, _ = roll(build(obstacles, BIG_BOX, 20), (1.5, 0.0))
    assert max(abs(y) for _, y in path) < 1e-6
    assert path[-1][0] == pytest.approx(0.75 - obstacles[0][2], abs=0.01)


def test_task3_respects_both_obstacles():
    obstacles = inflate_obstacles([0.7, 1.3], [0.15, -0.15], [0.15, 0.15], 0.105)
    controller = build(obstacles, BIG_BOX, 20)
    path, _ = roll(controller, (1.8, 0.0), steps=500)
    assert all(clearance(path, o) >= o[2] - 1e-3 for o in obstacles)


def test_tutorial_obstacles_are_infeasible_at_burger_speed():
    obstacles = inflate_obstacles([0.5, 1.4], [0.1, -0.3], [0.2, 0.3], 0.105)
    controller = build(obstacles, BIG_BOX, 20)
    roll(controller, (1.8, 0.0))
    assert 'Infeasible' in controller.last_status


def test_reference_horizon_overrides_a_fixed_goal():
    controller = build(box=BIG_BOX)
    controller.set_reference_horizon([(0.5, 0.0), (0.6, 0.0)])
    assert controller.compute(0.0, 0.0, 0.0, 0.0, 0.0)[0] > 0.0
    controller.set_reference_horizon(None)
    assert controller.compute(0.0, 0.0, 0.0, 0.0, 0.0) == (0.0, 0.0)


def test_prediction_covers_the_horizon_and_solve_fits_the_budget():
    controller = build()
    controller.compute(0.0, 0.0, 0.0, 0.8, 0.5)
    assert len(controller.predicted_path()) == 11
    assert 0.0 < controller.last_solve_time < T_STEP


def circle_lap(soft, delay_steps):
    """Task 4 at a desk. delay_steps stands in for the real robot's actuation lag."""
    circle = CircleTrajectory(0.8, 0.0, 0.0, 60.0, 1, 8.0)
    controller = build(inflate_obstacles([0.0], [0.62], [0.12], 0.105), (-1.5, 1.5), 20, soft)
    x = y = theta = t = 0.0
    queue = [(0.0, 0.0)] * delay_steps
    closest, errors = float('inf'), []
    while t < 68.0:
        controller.set_reference_horizon(
            [circle.point_at(t + k * T_STEP)[:2] for k in range(21)])
        gx, gy, _ = circle.point_at(t)
        queue.append(controller.compute(x, y, theta, gx, gy))
        if not controller.last_success:
            return None, controller.last_status, None
        v, omega = queue.pop(0)
        x += T_STEP * v * math.cos(theta)
        y += T_STEP * v * math.sin(theta)
        theta += T_STEP * omega
        t += T_STEP
        closest = min(closest, math.hypot(x, y - 0.62))
        if t > 8.0:
            errors.append(math.hypot(gx - x, gy - y))
    return closest, controller.last_status, sum(errors) / len(errors)


def test_task4_clean_lap_tracks_and_clears():
    closest, _, mean_error = circle_lap(soft=True, delay_steps=0)
    assert mean_error < 0.05
    assert closest >= 0.225 - 1e-3


def test_task4_hard_constraint_deadlocks_under_delay():
    closest, status, _ = circle_lap(soft=False, delay_steps=2)
    assert closest is None and 'Infeasible' in status


def test_task4_soft_constraint_survives_the_same_delay():
    closest, _, _ = circle_lap(soft=True, delay_steps=2)
    assert closest is not None and closest > 0.225 - 0.105
