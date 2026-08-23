"""Constrained MPC.

Several of these encode bugs that actually happened:

  * test_matches_lqr_when_unconstrained would have caught the missing dt factor
    that made MPC roughly 10x too timid;
  * test_feasible_with_cart_outside_the_track would have caught hard state
    constraints, which made 1125 of 1200 solves infeasible;
  * test_discretisation_is_not_euler guards the zero-order-hold discretisation.
"""

import numpy as np
import pytest

from cartpole.lqr import DEFAULT_Q, DEFAULT_R, linearize_upright
from cartpole.mpc import (
    CONTROL_DT,
    U_MAX,
    LinearMPC,
    discretize,
    matrix_exp,
    upright_discrete_model,
)

STATES = [
    np.array([0.0, 0.05, 0.0, 0.0]),
    np.array([0.1, 0.02, 0.0, 0.0]),
    np.array([0.0, 0.0, 0.0, 0.3]),
    np.array([0.2, -0.05, 0.1, -0.2]),
    np.array([-0.3, 0.1, 0.0, 0.5]),
]


@pytest.fixture(scope="module")
def model():
    return upright_discrete_model(CONTROL_DT)


def test_matrix_exp_against_known_values():
    np.testing.assert_allclose(matrix_exp(np.zeros((3, 3))), np.eye(3), atol=1e-12)
    diagonal = np.diag([1.0, -2.0, 0.5])
    np.testing.assert_allclose(np.diag(matrix_exp(diagonal)),
                               np.exp([1.0, -2.0, 0.5]), rtol=1e-10)


def test_discretisation_is_not_euler():
    """Ad -> I + A*dt as dt -> 0, with second-order error. Euler would be exact."""
    linear = linearize_upright()
    A, B = linear.A(), linear.B()
    errors = []
    for dt in (1e-4, 1e-3):
        Ad, _Bd = discretize(A, B, dt)
        errors.append(np.max(np.abs(Ad - (np.eye(4) + A * dt))))
    # Ten times the step should give about a hundred times the error.
    assert errors[1] / errors[0] == pytest.approx(100.0, rel=0.2)


def test_matches_lqr_when_unconstrained(model, lqr):
    """With nothing binding, MPC should reproduce LQR.

    The residual is real and structural -- a finite 0.3 s horizon and a
    zero-order hold against LQR's continuous infinite-horizon solution -- so the
    tolerance is a few percent, not machine precision.
    """
    K, S = lqr
    Ad, Bd = model
    mpc = LinearMPC(Ad, Bd, S=S, u_max=1e6, x_max=None)
    for state in STATES:
        expected = float(-K @ state)
        got = mpc.solve_qp(state)
        assert got is not None
        assert got == pytest.approx(expected, rel=0.05)


def test_respects_the_force_limit(model, lqr):
    """The whole point: the limit holds by construction, not by clipping."""
    _K, S = lqr
    Ad, Bd = model
    mpc = LinearMPC(Ad, Bd, S=S)
    for thetadot in (0.5, 1.0, 3.0, 10.0):
        command = mpc.solve_qp(np.array([0.0, 0.0, 0.0, thetadot]))
        assert command is not None
        assert abs(command) <= U_MAX + 1e-6


def test_feasible_with_cart_outside_the_track(model, lqr):
    """Soft state constraints keep the QP solvable when a disturbance puts the
    cart somewhere the constraint forbids. Hard constraints would go infeasible
    exactly when a control action matters most."""
    _K, S = lqr
    Ad, Bd = model
    mpc = LinearMPC(Ad, Bd, S=S, x_max=2.0)
    for cart in (2.5, 5.0, -8.0):
        command = mpc.solve_qp(np.array([cart, 0.05, 0.0, 0.0]))
        assert command is not None, f"infeasible at x = {cart}"
    assert mpc.infeasible_count == 0


def test_pushes_back_towards_the_track(model, lqr):
    """Outside the track on the left, the command should drive right."""
    _K, S = lqr
    Ad, Bd = model
    mpc = LinearMPC(Ad, Bd, S=S, x_max=2.0)
    assert mpc.solve_qp(np.array([-5.0, 0.0, 0.0, 0.0])) > 0.0
    assert mpc.solve_qp(np.array([5.0, 0.0, 0.0, 0.0])) < 0.0


def test_program_reuse_matches_rebuilding(model, lqr):
    """The optimisation must not change the answer, only the time it takes."""
    _K, S = lqr
    Ad, Bd = model
    rebuilt = LinearMPC(Ad, Bd, S=S, rebuild=True, warm_start=False)
    reused = LinearMPC(Ad, Bd, S=S, rebuild=False, warm_start=True)
    for state in STATES:
        assert reused.solve_qp(state) == pytest.approx(rebuilt.solve_qp(state), abs=1e-6)


def test_default_weights_penalise_angle_over_cart():
    """Deliberate: cart drift is recoverable, losing the pole is not."""
    assert DEFAULT_Q[1, 1] > DEFAULT_Q[0, 0]
    assert DEFAULT_R.shape == (1, 1)
