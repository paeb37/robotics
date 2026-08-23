"""Constrained linear MPC for balancing near upright.

The contrast with LQR is the whole point of the project:

    LQR  solves an unconstrained infinite-horizon problem ONCE, offline, in
         closed form. At runtime it is a matrix-vector product. It has no way to
         express a force or track limit -- which is why it cheerfully commanded
         474 N and drove the cart 353 m off the track.

    MPC  solves a constrained finite-horizon problem EVERY control step. The
         limits are rows in a quadratic program, so they are respected by
         construction rather than by clipping the output afterwards.

Near upright with no constraint active the two produce nearly the same control
law -- MPC only earns its keep when a constraint binds or the state leaves the
region where the linearisation holds. That equivalence is also the sharpest
correctness check available, and verify_mpc.py leans on it.
"""

import time

import numpy as np
from pydrake.solvers import MathematicalProgram, OsqpSolver
from pydrake.systems.framework import LeafSystem

from cartpole.controllers import wrapped_error
from cartpole.lqr import DEFAULT_Q, DEFAULT_R, linearize_upright, lqr_gain_and_cost_to_go

CONTROL_DT = 0.01  # 100 Hz -- about 15 decisions per 149 ms doubling time
HORIZON = 50       # steps, so 0.5 s of lookahead
U_MAX = 20.0       # N, matches the swing-up plan's force limit

# Cart track limit. Must be physically achievable: recovering from a shove means
# driving the cart under the falling pole, and at 20 N on an 11 kg system that
# takes room. LQR needs ~1.6 m for the smallest disturbance tested, so a limit
# below that asks for something no controller could deliver.
X_MAX = 2.0        # m
SLACK_PENALTY = 1e4  # cost per metre of track-limit violation


def matrix_exp(M, terms=24):
    """exp(M) by scaling-and-squaring with a truncated Taylor series.

    Hand-rolled: scipy is not a dependency and Drake does not expose expm.
    Scaling keeps ||M / 2^s|| small so the series converges fast; the repeated
    squaring then undoes the scaling exactly, since exp(M) = exp(M/2^s)^(2^s).
    """
    norm = float(np.max(np.abs(M)))
    s = int(max(0, np.ceil(np.log2(norm)) + 1)) if norm > 0.5 else 0
    scaled = M / (2.0**s)

    result = np.eye(M.shape[0])
    term = np.eye(M.shape[0])
    for k in range(1, terms + 1):
        term = term @ scaled / k
        result = result + term
    for _ in range(s):
        result = result @ result
    return result


def discretize(A, B, dt):
    """Exact zero-order-hold discretisation via the block-matrix identity

        expm([[A, B], [0, 0]] * dt) = [[Ad, Bd], [0, I]]

    Exact (not Euler) because the controller runs at 100 Hz against dynamics with
    a 149 ms doubling time -- Euler error would not be negligible.
    """
    n, m = B.shape
    block = np.zeros((n + m, n + m))
    block[:n, :n] = A
    block[:n, n:] = B
    expanded = matrix_exp(block * dt)
    return expanded[:n, :n], expanded[:n, n:]


def upright_discrete_model(dt=CONTROL_DT):
    """(Ad, Bd) for the plant linearised about upright and held at rate dt."""
    linear = linearize_upright()
    return discretize(linear.A(), linear.B(), dt)


class LinearMPC(LeafSystem):
    """Re-solve a constrained QP every control step.

    Discrete-time by construction: a periodic update solves the QP and writes the
    result into discrete state, and the output port reads that state directly. So
    the input is genuinely held between updates -- a real zero-order-hold
    controller, which is also what makes the Phase 4 latency question meaningful.

    The QP, over horizon N, in coordinates e = x - x_upright:

        minimise    sum_k  e[k]' Q e[k] + u[k]' R u[k]  +  e[N]' S e[N]
        subject to  e[k+1] = Ad e[k] + Bd u[k]
                    |u[k]| <= u_max
                    |e[k][0]| <= x_max          (cart track)
                    e[0] = current measured error

    The terminal cost S is the LQR cost-to-go from the Riccati equation. That is
    the standard device for making a short horizon behave like an infinite one:
    beyond the horizon, the cost-to-go is exactly what LQR would incur.
    """

    def __init__(self, Ad, Bd, Q=DEFAULT_Q, R=DEFAULT_R, S=None, horizon=HORIZON,
                 dt=CONTROL_DT, u_max=U_MAX, x_max=X_MAX,
                 slack_penalty=SLACK_PENALTY):
        LeafSystem.__init__(self)
        self._Ad, self._Bd = np.asarray(Ad), np.asarray(Bd)
        self._Q, self._R = np.asarray(Q), np.asarray(R)
        self._S = np.asarray(S if S is not None else lqr_gain_and_cost_to_go(Q, R)[1])
        self._N = int(horizon)
        self._dt = float(dt)
        self._u_max = float(u_max)
        self._x_max = None if x_max is None else float(x_max)
        self._slack_penalty = float(slack_penalty)
        self._solver = OsqpSolver()

        # Diagnostics for phase 4. Wall-clock per solve, and how often the QP
        # came back infeasible.
        self.solve_times = []
        self.infeasible_count = 0

        self.DeclareVectorInputPort("state", 4)
        state_index = self.DeclareDiscreteState(1)
        self.DeclarePeriodicDiscreteUpdateEvent(dt, 0.0, self._update)
        self.DeclareStateOutputPort("actuation", state_index)

    def solve_qp(self, e0):
        """Solve for the first input. Returns None if the QP is infeasible.

        Rebuilt from scratch each call -- the simple implementation, and the slow
        one. Building once and updating only the parts that change is the
        optimisation; measuring this first makes that a result rather than a
        guess.
        """
        prog = MathematicalProgram()
        n_x, n_u, N = 4, 1, self._N
        x = prog.NewContinuousVariables(N + 1, n_x, "x")
        u = prog.NewContinuousVariables(N, n_u, "u")

        prog.AddBoundingBoxConstraint(e0, e0, x[0])

        # Track constraint is SOFT: slack variables, penalised in the cost.
        #
        # The rule is input constraints hard, state constraints soft. You can
        # always respect your own actuator limit, so |u| <= u_max is a promise
        # you can keep. You CANNOT always respect a state limit -- a disturbance
        # can put the cart outside the track -- and a hard state constraint makes
        # the QP infeasible at precisely the moment a control action matters
        # most. Hard constraints here caused 1125 of 1200 solves to fail, the
        # controller to freeze at its last command, and the pole to fall.
        slack = None
        if self._x_max is not None:
            slack = prog.NewContinuousVariables(N, "slack")
            prog.AddBoundingBoxConstraint(0.0, np.inf, slack)
            prog.AddLinearCost(self._slack_penalty * np.ones(N), 0.0, slack)

        dynamics = np.hstack([self._Ad, self._Bd, -np.eye(n_x)])
        zeros = np.zeros(n_x)
        soft_band = np.array([[1.0, -1.0], [-1.0, -1.0]])
        for k in range(N):
            prog.AddLinearEqualityConstraint(
                dynamics, zeros, np.concatenate([x[k], u[k], x[k + 1]])
            )
            prog.AddBoundingBoxConstraint(-self._u_max, self._u_max, u[k])
            if slack is not None:
                #  x_cart - s <= x_max   and   -x_cart - s <= x_max
                prog.AddLinearConstraint(
                    soft_band,
                    np.full(2, -np.inf),
                    np.full(2, self._x_max),
                    np.array([x[k + 1][0], slack[k]]),
                )
            # Two scale factors here, and both matter:
            #
            #   * Drake's AddQuadraticCost computes 0.5 v'Mv, hence the 2.
            #   * The dt turns a SUM into an INTEGRAL. sum_k (e'Qe + u'Ru) over N
            #     steps approximates (1/dt) * the continuous integral, so without
            #     the dt the stage costs are over-weighted by 1/dt = 100x
            #     relative to the terminal cost S -- which came from the
            #     CONTINUOUS Riccati equation. Over-weighting R that badly makes
            #     MPC roughly 10x too timid compared to LQR.
            prog.AddQuadraticCost(2 * self._dt * self._Q, zeros, x[k])
            prog.AddQuadraticCost(2 * self._dt * self._R, np.zeros(n_u), u[k])
        prog.AddQuadraticCost(2 * self._S, zeros, x[N])

        started = time.perf_counter()
        result = self._solver.Solve(prog)
        self.solve_times.append(time.perf_counter() - started)

        if not result.is_success():
            self.infeasible_count += 1
            return None
        return float(result.GetSolution(u[0])[0])

    def _update(self, context, discrete_state):
        error = wrapped_error(self.get_input_port(0).Eval(context))
        command = self.solve_qp(error)
        if command is None:
            # Infeasible: hold the previous command. This is the recursive
            # feasibility problem showing up in practice -- once the cart is
            # already outside the track limit there is no trajectory satisfying
            # every constraint, so the QP has nothing to return. A soft
            # (slack-penalised) state constraint is the usual fix.
            return
        discrete_state.set_value([command])

    def latency_report(self):
        """p50/p99/max solve time, against the 149 ms doubling-time budget."""
        if not self.solve_times:
            return "no solves recorded"
        times_ms = np.array(self.solve_times) * 1e3
        return (f"solves={len(times_ms)}  p50={np.percentile(times_ms, 50):.2f}ms  "
                f"p99={np.percentile(times_ms, 99):.2f}ms  "
                f"max={times_ms.max():.2f}ms  infeasible={self.infeasible_count}")


def make_mpc_controller(horizon=HORIZON, dt=CONTROL_DT, u_max=U_MAX, x_max=X_MAX,
                        Q=DEFAULT_Q, R=DEFAULT_R, slack_penalty=SLACK_PENALTY):
    """Build a LinearMPC against the upright linearisation."""
    Ad, Bd = upright_discrete_model(dt)
    _K, S = lqr_gain_and_cost_to_go(Q, R)
    return LinearMPC(Ad, Bd, Q=Q, R=R, S=S, horizon=horizon, dt=dt,
                     u_max=u_max, x_max=x_max, slack_penalty=slack_penalty)
