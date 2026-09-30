"""Nonlinear MPC for the TurtleBot3 unicycle model, built with do-mpc.

No ROS in this file, so it can be tested on its own. mpc_node.py does the ROS side.
"""

import math
import time
import warnings

# do-mpc warns about optional features (ONNX, OPC UA) that are not installed.
with warnings.catch_warnings():
    warnings.simplefilter('ignore', UserWarning)
    import do_mpc

import casadi as ca
import numpy as np


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def inflate_obstacles(xs, ys, radii, inflation):
    """Return (x, y, radius) per obstacle, radius grown by the robot's own size."""
    if not len(xs) == len(ys) == len(radii):
        raise ValueError('obstacle_x, obstacle_y and obstacle_radius must be the same length')
    return [(float(x), float(y), float(r) + inflation) for x, y, r in zip(xs, ys, radii)]


class MPCController:

    def __init__(self, t_step, n_horizon, max_linear_velocity, max_angular_velocity,
                 min_linear_velocity, x_bounds, y_bounds, obstacles,
                 q_position, q_terminal, r_input, goal_tolerance,
                 obstacle_soft=False, obstacle_penalty=1e4):
        self.t_step = t_step
        self.n_horizon = n_horizon
        self.max_v = max_linear_velocity
        self.max_w = max_angular_velocity
        self.min_v = min_linear_velocity
        self.goal_tolerance = goal_tolerance

        self.last_success = False
        self.last_status = 'no solve yet'
        self.last_solve_time = 0.0
        self._reference = None
        self._initialised = False

        self.model = self._build_model()
        self.mpc = self._build_mpc(x_bounds, y_bounds, obstacles, q_position, q_terminal,
                                   r_input, obstacle_soft, obstacle_penalty)

    def _build_model(self):
        model = do_mpc.model.Model('continuous')
        model.set_variable(var_type='_x', var_name='x')
        model.set_variable(var_type='_x', var_name='y')
        th = model.set_variable(var_type='_x', var_name='th')
        vx = model.set_variable(var_type='_u', var_name='vx')
        vt = model.set_variable(var_type='_u', var_name='vt')

        # The setpoint is a time-varying parameter so it can change without rebuilding the solver.
        model.set_variable(var_type='_tvp', var_name='xdes')
        model.set_variable(var_type='_tvp', var_name='ydes')

        # Unicycle model. ca.cos, not math.cos, because th is a symbol here.
        model.set_rhs('x', vx * ca.cos(th))
        model.set_rhs('y', vx * ca.sin(th))
        model.set_rhs('th', vt)
        model.setup()
        return model

    def _build_mpc(self, x_bounds, y_bounds, obstacles, q_position, q_terminal, r_input,
                   obstacle_soft, obstacle_penalty):
        mpc = do_mpc.controller.MPC(self.model)
        mpc.set_param(
            n_horizon=self.n_horizon,
            t_step=self.t_step,
            n_robust=0,
            store_full_solution=True,
            nlpsol_opts={'ipopt.print_level': 0, 'ipopt.sb': 'yes', 'print_time': 0},
        )

        x, y = self.model.x['x'], self.model.x['y']
        error_squared = (x - self.model.tvp['xdes']) ** 2 + (y - self.model.tvp['ydes']) ** 2
        mpc.set_objective(lterm=q_position * error_squared, mterm=q_terminal * error_squared)
        mpc.set_rterm(vx=r_input, vt=r_input)

        mpc.bounds['lower', '_u', 'vx'] = self.min_v
        mpc.bounds['upper', '_u', 'vx'] = self.max_v
        mpc.bounds['lower', '_u', 'vt'] = -self.max_w
        mpc.bounds['upper', '_u', 'vt'] = self.max_w

        mpc.bounds['lower', '_x', 'x'] = x_bounds[0]
        mpc.bounds['upper', '_x', 'x'] = x_bounds[1]
        mpc.bounds['lower', '_x', 'y'] = y_bounds[0]
        mpc.bounds['upper', '_x', 'y'] = y_bounds[1]

        # r^2 - d^2 <= 0: stay outside each circle. Squared, so there is no square root.
        for i, (ox, oy, radius) in enumerate(obstacles, start=1):
            options = {'ub': 0.0, 'soft_constraint': obstacle_soft}
            if obstacle_soft:
                options['penalty_term_cons'] = obstacle_penalty
            mpc.set_nl_cons('obs%d' % i, radius ** 2 - ((x - ox) ** 2 + (y - oy) ** 2), **options)

        # do-mpc reads this template at every make_step, so writing into it sets the goal.
        self._tvp = mpc.get_tvp_template()
        mpc.set_tvp_fun(lambda t_now: self._tvp)
        mpc.setup()
        return mpc

    def set_reference_horizon(self, points):
        """Future setpoints, one per horizon step, or None to hold a single goal."""
        self._reference = list(points) if points else None

    def compute(self, x, y, theta, goal_x, goal_y):
        """One receding-horizon solve. Returns (v, omega), the first input of the plan."""
        # Stop inside the tolerance, or the robot creeps forever on odometry noise.
        if self._reference is None and math.hypot(goal_x - x, goal_y - y) <= self.goal_tolerance:
            self.last_success = True
            self.last_status = 'inside goal tolerance'
            return 0.0, 0.0

        state = np.array([[x], [y], [theta]])
        if not self._initialised:
            self.mpc.x0 = state
            self.mpc.set_initial_guess()
            self._initialised = True

        for k in range(self.n_horizon + 1):
            if self._reference:
                gx, gy = self._reference[min(k, len(self._reference) - 1)]
            else:
                gx, gy = goal_x, goal_y
            self._tvp['_tvp', k, 'xdes'] = gx
            self._tvp['_tvp', k, 'ydes'] = gy

        started = time.perf_counter()
        try:
            u = self.mpc.make_step(state)
        except Exception as error:
            self.last_success = False
            self.last_status = 'solver error: %s' % error
            return 0.0, 0.0
        finally:
            self.last_solve_time = time.perf_counter() - started

        stats = self.mpc.solver_stats
        self.last_success = bool(stats['success'])
        self.last_status = str(stats['return_status'])
        if not self.last_success:
            return 0.0, 0.0

        v = clamp(float(u[0, 0]), self.min_v, self.max_v)
        omega = clamp(float(u[1, 0]), -self.max_w, self.max_w)
        return v, omega

    def predicted_path(self):
        """The (x, y) points of the last plan, for drawing in RViz."""
        if not (self.last_success and self._initialised):
            return []
        xs = self.mpc.data.prediction(('_x', 'x'))[0, :, 0]
        ys = self.mpc.data.prediction(('_x', 'y'))[0, :, 0]
        return list(zip(xs.tolist(), ys.tolist()))

    def reset(self):
        """Drop the warm start. Called when the goal jumps to somewhere new."""
        self._initialised = False
        self._reference = None
        self.last_success = False
        self.last_status = 'reset'
