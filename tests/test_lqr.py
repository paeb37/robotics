"""LQR design, and the boundary of where it works."""

import numpy as np
import pytest
from pydrake.systems.analysis import Simulator

from cartpole.controllers import StateFeedback, wrapped_error
from cartpole.lqr import linearize_upright, make_lqr_controller
from cartpole.model import UPRIGHT, read_state, set_state
from cartpole.mpc import CONTROL_DT, upright_discrete_model
from cartpole.sim import build_cartpole

EXPECTED_LAMBDA = 4.6456      # 1/s
EXPECTED_DOUBLING = 0.1492    # s


def test_unstable_eigenvalue_and_doubling_time():
    """The clock the whole project is paced against."""
    eigenvalues = np.linalg.eigvals(linearize_upright().A())
    unstable = max(eigenvalues.real)
    assert unstable == pytest.approx(EXPECTED_LAMBDA, abs=1e-3)
    assert np.log(2) / unstable == pytest.approx(EXPECTED_DOUBLING, abs=1e-3)


def test_continuous_and_discrete_stability_criteria_agree():
    """Re(lambda) < 0 maps to |lambda| < 1 through lambda_d = exp(lambda_c * dt)."""
    continuous = np.linalg.eigvals(linearize_upright().A())
    discrete = np.linalg.eigvals(upright_discrete_model(CONTROL_DT)[0])
    expected = np.sort(np.abs(np.exp(continuous * CONTROL_DT)))
    np.testing.assert_allclose(np.sort(np.abs(discrete)), expected, rtol=1e-9)


def test_cost_to_go_is_positive_definite(lqr):
    """S must be PD for V(x) = e'Se to be a Lyapunov function at all."""
    _K, S = lqr
    np.testing.assert_allclose(S, S.T, atol=1e-9)
    assert np.linalg.eigvalsh(S).min() > 0.0


def test_gain_signs(lqr):
    """Pole tilting one way must produce force pushing the cart the same way.

    Catches a sign error that would otherwise look like a mysterious instability.
    """
    K, _S = lqr
    tilt = np.array([0.0, 0.05, 0.0, 0.0])   # theta above pi
    assert float(-K @ tilt) < 0.0            # so drive the cart negative
    np.testing.assert_allclose(float(-K @ (-tilt)), -float(-K @ tilt))


def test_wrapped_error_handles_the_branch_cut():
    """theta = -pi + eps is next to upright, not 2*pi away from it."""
    near = wrapped_error(np.array([0.0, -np.pi + 0.04, 0.0, 0.0]))
    assert abs(near[1]) == pytest.approx(0.04, abs=1e-9)
    assert abs(wrapped_error(np.array([0.0, UPRIGHT, 0.0, 0.0]))[1]) < 1e-12


def _simulate(theta0, duration=20.0, thetadot0=0.0):
    diagram, plant = build_cartpole(controller=make_lqr_controller(), meshcat=None)
    context = diagram.CreateDefaultContext()
    plant_context = plant.GetMyContextFromRoot(context)
    set_state(plant, plant_context, theta=theta0, thetadot=thetadot0)
    simulator = Simulator(diagram, context)
    simulator.Initialize()
    simulator.AdvanceTo(duration)
    return read_state(plant, plant_context)


def test_recovers_inside_the_basin():
    x, theta, _, _ = _simulate(UPRIGHT - 1.0)
    assert abs(theta - UPRIGHT) < 0.01
    assert abs(x) < 1.0


def test_runs_away_outside_the_basin():
    """The failure mode is the cart departing, NOT the pole dropping.

    An angle-only assertion would pass here, which is exactly the false positive
    the basin sweep was built to avoid.
    """
    x, _theta, _, _ = _simulate(UPRIGHT - 1.4)
    assert abs(x) > 50.0


def test_unconstrained_lqr_exceeds_the_actuator(lqr):
    """LQR has no way to express a force limit -- the premise for MPC."""
    K, _S = lqr
    demand = abs(float(-K @ np.array([0.0, 0.0, 0.0, 0.6])))
    assert demand > 20.0


def test_saturation_is_applied_when_requested(lqr):
    K, _S = lqr
    controller = StateFeedback(K, u_limit=20.0)
    context = controller.CreateDefaultContext()
    controller.get_input_port(0).FixValue(context, np.array([0.0, 0.0, 0.0, 5.0]))
    assert abs(controller.get_output_port(0).Eval(context)[0]) == pytest.approx(20.0)
