"""Nonlinear Model Predictive Control for the TurtleBot3 unicycle, using do-mpc.

At every control step the controller predicts `n_horizon` steps ahead with the unicycle
model, minimises a cost over that horizon subject to the constraints, and applies only
the first input of the resulting plan. The next step re-measures and re-solves, which is
the receding horizon that keeps model error from accumulating.

Model, written symbolically so CasADi can differentiate it:

    x_dot     = v cos(theta)
    y_dot     = v sin(theta)
    theta_dot = omega

Cost:

    lterm  squared position error at every predicted step
    mterm  squared position error at the final predicted step
    rterm  penalty on the change in input between steps, which keeps motion smooth

Constraints:

    bounds on _u        input limits
    bounds on _x        the boundary box
    set_nl_cons         one circular keep-out zone per obstacle

Plain Python throughout. No rclpy, no topics, so the control law can be unit tested at a
desk. mpc_node.py owns the ROS side.

The setpoint is a time-varying parameter (`_tvp`) rather than a constant in the
objective, because a constant would mean rebuilding the compiled optimizer every time the
goal changed. It also lets the horizon be filled with where the reference *will* be,
which is what trajectory tracking needs.
"""

from dataclasses import dataclass
import math
import time
import warnings

# do-mpc warns at import about optional features the base install does not ship. Three
# unactionable warnings per node start, in the middle of the launch log.
with warnings.catch_warnings():
    warnings.simplefilter('ignore', UserWarning)
    import do_mpc

import casadi as ca
import numpy as np

from .controller_base import Command, Controller, Pose2D
from .geometry import clamp


@dataclass(frozen=True)
class Obstacle:
    """A circular keep-out zone in the odometry frame.

    `radius` is the constraint radius handed to the solver: the physical obstacle plus an
    inflation for the robot's own footprint, since the MPC model is a point. Inflating in
    one place keeps the constraint and the RViz marker in agreement.
    """

    x: float
    y: float
    radius: float

    @staticmethod
    def from_arrays(xs, ys, radii, inflation: float):
        """Build the obstacle list from three parallel parameter arrays."""
        if not (len(xs) == len(ys) == len(radii)):
            raise ValueError(
                'obstacle_x, obstacle_y and obstacle_radius must be the same length, '
                'got %d, %d and %d' % (len(xs), len(ys), len(radii))
            )
        obstacles = []
        for x, y, r in zip(xs, ys, radii):
            if r <= 0.0:
                raise ValueError('obstacle radius must be greater than zero, got %r' % r)
            obstacles.append(Obstacle(float(x), float(y), float(r) + inflation))
        return obstacles


class MPCController(Controller):
    """Position and trajectory tracking MPC for the TurtleBot3 Burger.

    Every argument is a parameter read from YAML by the node. The optimizer is built once
    and reused: do-mpc compiles the horizon, the bounds and the obstacle set into the NLP
    at setup() time, so only the setpoint is allowed to change afterwards.
    """

    def __init__(
        self,
        t_step: float,
        n_horizon: int,
        max_linear_velocity: float,
        max_angular_velocity: float,
        min_linear_velocity: float,
        x_bounds,
        y_bounds,
        obstacles,
        q_position: float,
        q_terminal: float,
        r_input: float,
        goal_tolerance: float,
        n_robust: int = 0,
        solver_verbose: bool = False,
        obstacle_soft: bool = False,
        obstacle_penalty: float = 1e4,
    ) -> None:
        if t_step <= 0.0:
            raise ValueError('t_step must be greater than zero, got %r' % t_step)
        if n_horizon < 1:
            raise ValueError('n_horizon must be at least one, got %r' % n_horizon)
        if x_bounds[0] >= x_bounds[1] or y_bounds[0] >= y_bounds[1]:
            raise ValueError('boundary constraints must be lower < upper, got %r and %r'
                             % (x_bounds, y_bounds))

        self.t_step = t_step
        self.n_horizon = n_horizon
        self.max_linear_velocity = max_linear_velocity
        self.max_angular_velocity = max_angular_velocity
        self.min_linear_velocity = min_linear_velocity
        self.x_bounds = tuple(x_bounds)
        self.y_bounds = tuple(y_bounds)
        self.obstacles = list(obstacles)
        self.goal_tolerance = goal_tolerance
        self.obstacle_soft = obstacle_soft
        self.obstacle_penalty = obstacle_penalty

        # Outcome of the most recent solve, for the node to log. A failed solve is a fact
        # about the problem, not an exception to swallow.
        self.last_success = False
        self.last_status = 'no solve yet'
        self.last_solve_time = 0.0

        self._reference = None
        self._initialised = False

        self.model = self._build_model()
        self.mpc = self._build_mpc(q_position, q_terminal, r_input, n_robust, solver_verbose)

    def _build_model(self):
        """Unicycle kinematics, plus the two time-varying setpoint parameters."""
        model = do_mpc.model.Model('continuous')

        x = model.set_variable(var_type='_x', var_name='x', shape=(1, 1))
        y = model.set_variable(var_type='_x', var_name='y', shape=(1, 1))
        th = model.set_variable(var_type='_x', var_name='th', shape=(1, 1))

        vx = model.set_variable(var_type='_u', var_name='vx')
        vt = model.set_variable(var_type='_u', var_name='vt')

        model.set_variable(var_type='_tvp', var_name='xdes', shape=(1, 1))
        model.set_variable(var_type='_tvp', var_name='ydes', shape=(1, 1))

        # ca.cos / ca.sin, not math: these build symbolic expressions the solver
        # differentiates, and math would demand a float.
        model.set_rhs('x', vx * ca.cos(th))
        model.set_rhs('y', vx * ca.sin(th))
        model.set_rhs('th', vt)

        model.setup()
        return model

    def _build_mpc(self, q_position, q_terminal, r_input, n_robust, solver_verbose):
        """Cost, bounds, obstacle constraints, and the compiled optimizer."""
        mpc = do_mpc.controller.MPC(self.model)

        setup = {
            'n_horizon': self.n_horizon,
            't_step': self.t_step,
            # n_robust builds a scenario tree over uncertain model parameters. This model
            # declares none, so anything above zero costs solve time and buys nothing.
            'n_robust': n_robust,
            # Keeps the predicted horizon available for predicted_path().
            'store_full_solution': True,
        }
        if not solver_verbose:
            # IPOPT prints an iteration table per solve; at 10 Hz that floods the log.
            setup['nlpsol_opts'] = {
                'ipopt.print_level': 0,
                'ipopt.sb': 'yes',
                'print_time': 0,
            }
        mpc.set_param(**setup)

        error_squared = (
            (self.model.x['x'] - self.model.tvp['xdes']) ** 2
            + (self.model.x['y'] - self.model.tvp['ydes']) ** 2
        )
        # Without mterm nothing scores where the horizon ends, so the solver may return a
        # plan still travelling fast past the goal at the final step.
        mpc.set_objective(mterm=q_terminal * error_squared, lterm=q_position * error_squared)
        # Penalty on the change in input, not its size. Without it the cheapest plan is
        # often jerky, and jerky v on a real Burger is wheel slip.
        mpc.set_rterm(vx=r_input, vt=r_input)

        # The node passes the tighter of the lab's stated bound and the robot's own limit.
        mpc.bounds['lower', '_u', 'vx'] = self.min_linear_velocity
        mpc.bounds['upper', '_u', 'vx'] = self.max_linear_velocity
        mpc.bounds['lower', '_u', 'vt'] = -self.max_angular_velocity
        mpc.bounds['upper', '_u', 'vt'] = self.max_angular_velocity

        # Boundary constraints, hard: a plan leaving the box at any predicted step is
        # rejected, which is what makes a setpoint outside the box unreachable rather
        # than merely expensive.
        mpc.bounds['lower', '_x', 'x'] = self.x_bounds[0]
        mpc.bounds['upper', '_x', 'x'] = self.x_bounds[1]
        mpc.bounds['lower', '_x', 'y'] = self.y_bounds[0]
        mpc.bounds['upper', '_x', 'y'] = self.y_bounds[1]

        # r^2 - d^2 <= 0 says the squared distance to the centre is at least the squared
        # radius. Squared on both sides so no square root enters the Jacobian.
        for index, obstacle in enumerate(self.obstacles, start=1):
            constraint = {'ub': 0.0, 'soft_constraint': self.obstacle_soft}
            if self.obstacle_soft:
                constraint['penalty_term_cons'] = self.obstacle_penalty
            mpc.set_nl_cons(
                'obs%d' % index,
                obstacle.radius ** 2
                - ((self.model.x['x'] - obstacle.x) ** 2 + (self.model.x['y'] - obstacle.y) ** 2),
                **constraint,
            )

        # Allocated once and mutated in place each tick; do-mpc reads whatever the
        # template currently holds at every make_step.
        self._tvp_template = mpc.get_tvp_template()
        mpc.set_tvp_fun(lambda t_now: self._tvp_template)

        mpc.setup()
        return mpc

    def set_reference_horizon(self, points) -> None:
        """Supply where the reference will be at each future step of the horizon.

        `points` is a sequence of (x, y), index k meaning k steps of t_step ahead. Shorter
        than the horizon is fine; the last point is held for the remainder. Pass None to
        go back to holding the single current goal across the horizon, which is what a
        fixed setpoint wants.
        """
        self._reference = list(points) if points else None

    def _write_tvp(self, goal_x: float, goal_y: float) -> None:
        """Fill every horizon slot of the setpoint template."""
        for k in range(self.n_horizon + 1):
            if self._reference:
                point = self._reference[min(k, len(self._reference) - 1)]
                x_k, y_k = point[0], point[1]
            else:
                x_k, y_k = goal_x, goal_y
            self._tvp_template['_tvp', k, 'xdes'] = x_k
            self._tvp_template['_tvp', k, 'ydes'] = y_k

    def tracking_error(self, pose: Pose2D, goal_x: float, goal_y: float):
        """Error vector and straight line distance from the robot's centre to the goal."""
        e_x = goal_x - pose.x
        e_y = goal_y - pose.y
        return e_x, e_y, math.hypot(e_x, e_y)

    def compute(self, pose: Pose2D, goal_x: float, goal_y: float) -> Command:
        """One receding horizon solve. Returns the first input of the optimal plan."""
        _, _, distance = self.tracking_error(pose, goal_x, goal_y)

        # The solver never returns exactly zero, so without a tolerance the robot creeps
        # against odometry noise indefinitely.
        if self._reference is None and distance <= self.goal_tolerance:
            self.last_success = True
            self.last_status = 'inside goal tolerance'
            return Command(0.0, 0.0)

        state = np.array([pose.x, pose.y, pose.theta]).reshape(-1, 1)

        if not self._initialised:
            self.mpc.x0 = state
            self.mpc.set_initial_guess()
            self._initialised = True

        self._write_tvp(goal_x, goal_y)

        started = time.perf_counter()
        try:
            u = self.mpc.make_step(state)
        except Exception as error:
            # A solver failure is a fact about this problem, not a reason to kill the
            # node. The caller publishes zero and reports why.
            self.last_success = False
            self.last_status = 'solver raised %s: %s' % (type(error).__name__, error)
            self.last_solve_time = time.perf_counter() - started
            return Command(0.0, 0.0)

        # Wall time around make_step: do-mpc 5.1.1 does not report t_wall_total, and the
        # whole call is what has to fit inside t_step anyway.
        self.last_solve_time = time.perf_counter() - started
        stats = getattr(self.mpc, 'solver_stats', {}) or {}
        self.last_success = bool(stats.get('success', True))
        self.last_status = str(stats.get('return_status', 'unknown'))

        if not self.last_success:
            # No input sequence respects every constraint over the horizon. Returning the
            # solver's best effort anyway would drive toward a constraint it cannot meet.
            return Command(0.0, 0.0)

        v = clamp(float(u[0, 0]), self.min_linear_velocity, self.max_linear_velocity)
        omega = clamp(float(u[1, 0]), -self.max_angular_velocity, self.max_angular_velocity)
        return Command(v, omega)

    def predicted_path(self):
        """The horizon the last solve planned, as a list of (x, y). Empty if none."""
        if not self.last_success:
            return []
        try:
            xs = self.mpc.data.prediction(('_x', 'x'))[0, :, 0]
            ys = self.mpc.data.prediction(('_x', 'y'))[0, :, 0]
        except (KeyError, IndexError, AttributeError):
            return []
        return [(float(a), float(b)) for a, b in zip(xs, ys)]

    def reset(self) -> None:
        """Throw away the warm start, for when the goal jumps discontinuously.

        The previous solution is a good initial guess only for a problem close to the
        previous one, and a new setpoint is not that.
        """
        self._initialised = False
        self._reference = None
        self.last_success = False
        self.last_status = 'reset'
