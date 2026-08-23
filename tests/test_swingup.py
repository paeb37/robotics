"""The swing-up trajectory.

Asserted against a committed fixture rather than re-solved: the optimisation
takes minutes, and a suite nobody runs protects nothing. The slow test that
actually re-solves is marked and deselected by default.
"""

import numpy as np
import pytest

from cartpole.swingup import (
    HANGING_STATE,
    U_MAX,
    UPRIGHT_STATE,
    solve_swingup,
)


def _samples(state_traj, input_traj, count=600):
    times = np.linspace(0.0, state_traj.end_time(), count)
    states = np.hstack([state_traj.value(t) for t in times])
    inputs = np.hstack([input_traj.value(t) for t in times])
    return times, states, inputs


def test_starts_hanging_at_rest(swingup):
    state_traj, _ = swingup
    np.testing.assert_allclose(state_traj.value(0.0).flatten(), HANGING_STATE, atol=1e-5)


def test_ends_upright_at_rest(swingup):
    state_traj, _ = swingup
    final = state_traj.value(state_traj.end_time()).flatten()
    np.testing.assert_allclose(final, UPRIGHT_STATE, atol=1e-4)


def test_respects_the_force_limit(swingup):
    """The constraint the trajectory was planned under."""
    _, _, u = _samples(*swingup)
    assert np.max(np.abs(u)) <= U_MAX + 1e-3


def test_force_limit_is_actually_active(swingup):
    """If |u| never approaches the limit, the limit was not shaping the solution
    and the problem was easier than intended."""
    _, _, u = _samples(*swingup)
    assert np.max(np.abs(u)) > 0.99 * U_MAX


def test_pumps_energy_over_several_swings(swingup):
    """At 20 N the pole cannot be lifted directly, so the optimiser has to rock
    the cart in sympathy with the pendulum. Sign changes in thetadot count the
    swings; a direct lift would show almost none."""
    _, states, _ = _samples(*swingup)
    sign_changes = int(np.sum(np.diff(np.sign(states[3, :])) != 0))
    assert sign_changes >= 5


def test_cart_excursion_is_bounded(swingup):
    """A swing-up that needs 50 m of rail is not a useful plan."""
    _, states, _ = _samples(*swingup)
    assert np.max(np.abs(states[0, :])) < 3.0


def test_duration_is_plausible(swingup):
    state_traj, _ = swingup
    assert 5.0 < state_traj.end_time() < 20.0


@pytest.mark.slow
def test_resolving_reproduces_the_fixture(swingup):
    """Re-run the optimisation and check it still lands somewhere equivalent.

    Not asserted pointwise: the program is nonconvex, so the solver may find a
    different local optimum. What must hold is that the boundary conditions and
    the force limit are still satisfied.
    """
    state_traj, input_traj, result = solve_swingup()
    assert result.is_success()
    np.testing.assert_allclose(state_traj.value(0.0).flatten(), HANGING_STATE, atol=1e-5)
    np.testing.assert_allclose(
        state_traj.value(state_traj.end_time()).flatten(), UPRIGHT_STATE, atol=1e-4
    )
    _, _, u = _samples(state_traj, input_traj)
    assert np.max(np.abs(u)) <= U_MAX + 1e-3
