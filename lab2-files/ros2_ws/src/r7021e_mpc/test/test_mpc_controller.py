"""Unit tests for the Lab 2 MPC control law. No ROS, no robot, no simulator.

What is checked is not that the solver converges prettily but that the constraints the
lab names are actually enforced on the resulting trajectory. Each test rolls the closed
loop forward with the same Euler integration the tutorial's simulation loop uses, so a
violation found here is a violation the real controller would command.
"""

import math

import pytest

from r7021e_mpc.controller_base import Pose2D
from r7021e_mpc.mpc_controller import MPCController, Obstacle

# Small horizon: every test builds its own optimizer, and 10 steps keeps the file to a
# few seconds while still being long enough for the obstacle constraint to bind.
HORIZON = 10
T_STEP = 0.1


def build(obstacles=(), x_bounds=(-1.0, 1.0), y_bounds=(-1.0, 1.0), n_horizon=HORIZON,
          obstacle_soft=False):
    """An MPCController with the lab's input constraints and the Burger's limits."""
    return MPCController(
        t_step=T_STEP, n_horizon=n_horizon,
        max_linear_velocity=0.22, max_angular_velocity=0.8, min_linear_velocity=0.0,
        x_bounds=x_bounds, y_bounds=y_bounds, obstacles=obstacles,
        q_position=1.0, q_terminal=1.0, r_input=0.01, goal_tolerance=0.05,
        obstacle_soft=obstacle_soft, obstacle_penalty=1e4)


def roll(controller, goal, start=(0.0, 0.0, 0.0), steps=200):
    """Run the closed loop and return the path and the commands issued."""
    x, y, theta = start
    path = [(x, y)]
    commands = []
    for _ in range(steps):
        command = controller.compute(Pose2D(x, y, theta), goal[0], goal[1])
        commands.append(command)
        x += T_STEP * command.v * math.cos(theta)
        y += T_STEP * command.v * math.sin(theta)
        theta += T_STEP * command.omega
        path.append((x, y))
        if math.hypot(goal[0] - x, goal[1] - y) < 0.05:
            break
    return path, commands


class TestObstacle:
    def test_inflation_is_added_to_the_physical_radius(self):
        obstacle = Obstacle.from_arrays([1.0], [2.0], [0.2], inflation=0.105)[0]
        assert (obstacle.x, obstacle.y) == (1.0, 2.0)
        assert obstacle.radius == pytest.approx(0.305)

    def test_mismatched_array_lengths_are_rejected(self):
        with pytest.raises(ValueError, match='same length'):
            Obstacle.from_arrays([1.0, 2.0], [1.0], [0.2, 0.2], inflation=0.0)

    def test_non_positive_radius_is_rejected(self):
        with pytest.raises(ValueError, match='radius'):
            Obstacle.from_arrays([1.0], [1.0], [0.0], inflation=0.1)

    def test_no_obstacles_is_a_valid_configuration(self):
        assert Obstacle.from_arrays([], [], [], inflation=0.105) == []


class TestConstruction:
    def test_horizon_below_one_is_rejected(self):
        with pytest.raises(ValueError, match='n_horizon'):
            build(n_horizon=0)

    def test_inverted_boundary_is_rejected(self):
        with pytest.raises(ValueError, match='lower < upper'):
            build(x_bounds=(1.0, -1.0))


class TestSetpointTracking:
    """Task 1: setpoint tracking with input and boundary constraints."""

    def test_reaches_a_setpoint_inside_the_boundary(self):
        path, _ = roll(build(), (0.8, 0.5))
        end_x, end_y = path[-1]
        assert math.hypot(0.8 - end_x, 0.5 - end_y) < 0.05

    def test_commands_respect_the_input_constraints(self):
        _, commands = roll(build(), (0.8, 0.5))
        # The lab states 0 < v < 0.5 and -0.8 < omega < 0.8; the Burger's 0.22 m/s is
        # tighter on v, so that is the bound the controller is built with.
        assert all(0.0 <= c.v <= 0.22 for c in commands)
        assert all(abs(c.omega) <= 0.8 for c in commands)

    def test_never_reverses(self):
        """v has a lower bound of zero, so a goal behind the robot means turning first."""
        _, commands = roll(build(), (-0.6, 0.0))
        assert all(c.v >= 0.0 for c in commands)

    def test_a_setpoint_outside_the_boundary_stops_at_the_boundary(self):
        """Task 1b. The hard bound shrinks the feasible set; the optimum moves to its edge."""
        path, _ = roll(build(), (1.5, 0.0), steps=120)
        assert max(abs(x) for x, _ in path) <= 1.0 + 1e-3
        assert path[-1][0] == pytest.approx(1.0, abs=0.02)


class TestObstacleAvoidance:
    """Tasks 2 and 3. Coordinates match mpc_task2.yaml and mpc_task3.yaml."""

    TASK2 = ([0.75], [0.08], [0.15])
    TASK3 = ([0.7, 1.3], [0.15, -0.15], [0.15, 0.15])

    def obstacles(self, arrays):
        return Obstacle.from_arrays(*arrays, inflation=0.105)

    def test_single_obstacle_is_never_entered(self):
        obstacles = self.obstacles(self.TASK2)
        controller = build(obstacles=obstacles, x_bounds=(-2.0, 2.0), y_bounds=(-2.0, 2.0),
                           n_horizon=20)
        path, _ = roll(controller, (1.5, 0.0), steps=400)
        clearance = min(math.hypot(x - 0.75, y - 0.08) for x, y in path)
        assert clearance >= obstacles[0].radius - 1e-3

    def test_the_obstacle_forces_a_detour_and_the_goal_is_still_reached(self):
        """A constraint that never binds proves nothing, so check that it did work."""
        controller = build(obstacles=self.obstacles(self.TASK2),
                           x_bounds=(-2.0, 2.0), y_bounds=(-2.0, 2.0), n_horizon=20)
        path, _ = roll(controller, (1.5, 0.0), steps=400)
        # A straight run from (0,0) to (1.5,0) would stay on y = 0 throughout.
        assert max(abs(y) for _, y in path) > 0.1
        end_x, end_y = path[-1]
        assert math.hypot(1.5 - end_x, -end_y) < 0.06

    def test_an_obstacle_exactly_on_the_axis_deadlocks(self):
        """Why mpc_task2.yaml offsets its obstacle by 0.08 m.

        Head-on, detouring left and right have identical cost, the gradient is zero, and
        the robot drives up to the keep-out boundary and stops. Pinned down as a test so
        it is not rediscovered as a tuning mystery on a lab day.
        """
        obstacles = Obstacle.from_arrays([0.75], [0.0], [0.15], inflation=0.105)
        controller = build(obstacles=obstacles, x_bounds=(-2.0, 2.0), y_bounds=(-2.0, 2.0),
                           n_horizon=20)
        path, _ = roll(controller, (1.5, 0.0), steps=200)
        assert max(abs(y) for _, y in path) < 1e-6
        assert path[-1][0] == pytest.approx(0.75 - obstacles[0].radius, abs=0.01)

    def test_two_obstacles_are_both_respected(self):
        obstacles = self.obstacles(self.TASK3)
        controller = build(obstacles=obstacles, x_bounds=(-2.0, 2.0), y_bounds=(-2.0, 2.0),
                           n_horizon=20)
        path, _ = roll(controller, (1.8, 0.0), steps=500)
        for obstacle in obstacles:
            clearance = min(math.hypot(x - obstacle.x, y - obstacle.y) for x, y in path)
            assert clearance >= obstacle.radius - 1e-3

    def test_the_tutorial_coordinates_are_infeasible_at_burger_speed(self):
        """Why mpc_task3.yaml does not use the tutorial's own obstacles.

        The tutorial bounds vx at 0.8 m/s; the Burger does 0.22, so the same horizon
        covers 1.6 m there and 0.44 m here, and inflating both obstacles by the robot's
        footprint closes the corridor between them from a head-on start.
        """
        obstacles = Obstacle.from_arrays([0.5, 1.4], [0.1, -0.3], [0.2, 0.3], inflation=0.105)
        controller = build(obstacles=obstacles, x_bounds=(-2.0, 2.0), y_bounds=(-2.0, 2.0),
                           n_horizon=20)
        path, _ = roll(controller, (1.8, 0.0), steps=200)
        end_x, end_y = path[-1]
        assert math.hypot(1.8 - end_x, -end_y) > 1.0
        assert not controller.last_success
        assert 'Infeasible' in controller.last_status


class TestHorizonReference:
    """Task 4: the horizon is filled with the reference's future, not one held point."""

    def test_a_reference_shorter_than_the_horizon_holds_its_last_point(self):
        controller = build(x_bounds=(-2.0, 2.0), y_bounds=(-2.0, 2.0))
        controller.set_reference_horizon([(0.5, 0.0), (0.6, 0.0)])
        # The goal arguments say "stay put" and the reference says "go to 0.6".
        assert controller.compute(Pose2D(0.0, 0.0, 0.0), 0.0, 0.0).v > 0.0

    def test_clearing_the_reference_returns_to_the_single_goal(self):
        controller = build(x_bounds=(-2.0, 2.0), y_bounds=(-2.0, 2.0))
        controller.set_reference_horizon([(0.5, 0.0)])
        controller.set_reference_horizon(None)
        command = controller.compute(Pose2D(0.0, 0.0, 0.0), 0.0, 0.0)
        assert (command.v, command.omega) == (0.0, 0.0)


class TestPredictionAndReset:
    def test_predicted_path_spans_the_whole_horizon(self):
        controller = build()
        controller.compute(Pose2D(0.0, 0.0, 0.0), 0.8, 0.5)
        assert len(controller.predicted_path()) == HORIZON + 1

    def test_reset_discards_the_warm_start(self):
        controller = build()
        controller.compute(Pose2D(0.0, 0.0, 0.0), 0.8, 0.5)
        controller.reset()
        assert controller.last_status == 'reset'

    def test_solve_time_is_measured(self):
        controller = build()
        controller.compute(Pose2D(0.0, 0.0, 0.0), 0.8, 0.5)
        assert 0.0 < controller.last_solve_time < controller.t_step


class TestHardConstraintDeadlock:
    """Why mpc_task4.yaml uses a soft obstacle constraint and tasks 2 and 3 do not.

    A hard nl_cons is enforced at every horizon step including step zero, and step zero is
    the measured state rather than a decision variable, so a measurement that already
    violates it makes the problem infeasible for every input. That only bites when the
    optimal path rides the constraint boundary, which is what tracking a reference through
    an obstacle asks for. An actuation delay is the cheapest desk stand-in for what a real
    robot does: the command is computed from a pose that is already old.
    """

    def lap(self, soft, delay_steps):
        from r7021e_mpc.trajectories import CircleTrajectory

        t_step, horizon = 0.1, 20
        trajectory = CircleTrajectory(0.8, 0.0, 0.0, 60.0, 1, 8.0)
        obstacles = Obstacle.from_arrays([0.0], [0.62], [0.12], inflation=0.105)
        controller = MPCController(
            t_step=t_step, n_horizon=horizon, max_linear_velocity=0.22,
            max_angular_velocity=0.8, min_linear_velocity=0.0,
            x_bounds=(-1.5, 1.5), y_bounds=(-1.5, 1.5), obstacles=obstacles,
            q_position=1.0, q_terminal=1.0, r_input=0.01, goal_tolerance=0.05,
            obstacle_soft=soft, obstacle_penalty=1e4)

        x, y, theta, t = 0.0, 0.0, 0.0, 0.0
        pipeline = [(0.0, 0.0)] * delay_steps
        closest = float('inf')
        errors = []
        while t < 68.0:
            controller.set_reference_horizon(
                [trajectory.point_at(t + k * t_step)[:2] for k in range(horizon + 1)])
            goal_x, goal_y, _ = trajectory.point_at(t)
            command = controller.compute(Pose2D(x, y, theta), goal_x, goal_y)
            if not controller.last_success:
                return None, controller.last_status, None
            pipeline.append((command.v, command.omega))
            v, omega = pipeline.pop(0)
            x += t_step * v * math.cos(theta)
            y += t_step * v * math.sin(theta)
            theta += t_step * omega
            t += t_step
            closest = min(closest, math.hypot(x, y - 0.62))
            if t > 8.0:
                errors.append(math.hypot(goal_x - x, goal_y - y))
        return closest, controller.last_status, sum(errors) / len(errors)

    def test_a_clean_lap_tracks_the_circle_and_clears_the_obstacle(self):
        closest, _, mean_error = self.lap(soft=True, delay_steps=0)
        assert closest is not None
        assert mean_error < 0.05
        assert closest >= 0.225 - 1e-3

    def test_hard_constraint_deadlocks_under_actuation_delay(self):
        closest, status, _ = self.lap(soft=False, delay_steps=2)
        assert closest is None
        assert 'Infeasible' in status

    def test_soft_constraint_survives_the_same_delay(self):
        closest, _, _ = self.lap(soft=True, delay_steps=2)
        assert closest is not None, 'soft constraint should complete the lap'
        # The slack is spent against the inflation margin, not against real clearance:
        # inflation is 0.105 m, so a few millimetres of slack still leaves about 0.10 m
        # between the robot and the physical obstacle.
        assert closest > 0.225 - 0.105

    def test_soft_and_hard_agree_when_nothing_perturbs_them(self):
        """The soft constraint must not buy clearance cheaply when it has a choice."""
        hard, _, _ = self.lap(soft=False, delay_steps=0)
        soft, _, _ = self.lap(soft=True, delay_steps=0)
        assert hard is not None and soft is not None
        assert abs(hard - soft) < 1e-3
