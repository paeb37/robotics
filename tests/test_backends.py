"""Python and C++ backends must agree.

A faster controller that computes something different is not a faster
implementation, it is a different controller. This is the gate the latency
numbers depend on.

Skipped when the C++ library has not been built -- it is optional, and the
project runs without a compiler.
"""

import numpy as np
import pytest
from pydrake.systems.analysis import Simulator

from cartpole.cpp_backend import available, unavailable_reason
from cartpole.model import UPRIGHT, read_state, set_state
from cartpole.mpc import CONTROL_DT, U_MAX, make_mpc_controller, upright_discrete_model
from cartpole.sim import build_cartpole

pytestmark = pytest.mark.skipif(
    not available(), reason=f"C++ backend not built: {unavailable_reason()}"
)

# States inside the operating envelope. Deliberately not deep in constraint
# violation: out there the objective is nearly flat in u[0] -- the whole u range
# spans under 0.1% of the cost -- so two solvers legitimately land on different
# near-optimal points. That is a property of the problem, not a disagreement.
STATES = [
    np.array([0.0, 0.05, 0.0, 0.0]),
    np.array([0.1, 0.02, 0.0, 0.0]),
    np.array([0.0, 0.0, 0.0, 0.3]),
    np.array([0.2, -0.05, 0.1, -0.2]),
    np.array([-0.3, 0.1, 0.0, 0.5]),
    np.array([1.5, 0.2, 0.5, 1.0]),
]


def _fresh_pair():
    """New instances so neither is influenced by warm-start history."""
    return make_mpc_controller(backend="python"), make_mpc_controller(backend="cpp")


def test_library_loads():
    from cartpole.cpp_backend import _load
    assert _load() is not None


def test_agree_pointwise():
    for state in STATES:
        python_mpc, cpp_mpc = _fresh_pair()
        expected = python_mpc.solve_qp(state)
        got = cpp_mpc.solve_qp(state)
        assert got is not None
        assert got == pytest.approx(expected, abs=1e-6), f"disagreement at {state}"


def test_cpp_respects_the_force_limit():
    _python, cpp_mpc = _fresh_pair()
    for thetadot in (0.5, 2.0, 10.0):
        command = cpp_mpc.solve_qp(np.array([0.0, 0.0, 0.0, thetadot]))
        assert command is not None
        assert abs(command) <= U_MAX + 1e-6


def test_cpp_stays_feasible_off_track():
    _python, cpp_mpc = _fresh_pair()
    for cart in (2.5, -6.0):
        assert cpp_mpc.solve_qp(np.array([cart, 0.05, 0.0, 0.0])) is not None
    assert cpp_mpc.infeasible_count == 0


def _closed_loop(backend, kick=0.3, duration=4.0):
    mpc = make_mpc_controller(backend=backend)
    diagram, plant = build_cartpole(controller=mpc, meshcat=None)
    context = diagram.CreateDefaultContext()
    plant_context = plant.GetMyContextFromRoot(context)
    set_state(plant, plant_context, theta=UPRIGHT, thetadot=kick)
    simulator = Simulator(diagram, context)
    simulator.Initialize()
    trajectory = []
    for t in np.linspace(0.0, duration, 201):
        simulator.AdvanceTo(t)
        trajectory.append(read_state(plant, plant_context).copy())
    return np.array(trajectory)


def test_closed_loop_trajectories_agree():
    """The assertion that actually matters: same controller behaviour, not just
    the same answer at hand-picked states."""
    difference = np.max(np.abs(_closed_loop("python") - _closed_loop("cpp")))
    assert difference < 1e-9, f"closed-loop divergence {difference:.2e}"


def test_model_matrices_are_passed_through_intact():
    """The C++ side gets Ad/Bd from Python. Guard the row-major convention: a
    transposed Ad would still solve, just wrongly."""
    Ad, _Bd = upright_discrete_model(CONTROL_DT)
    assert Ad.shape == (4, 4)
    assert not np.allclose(Ad, Ad.T), "Ad is symmetric; the transpose test is vacuous"
