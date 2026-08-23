"""Shared fixtures.

Simulation-based tests are the slow ones, so anything reusable across tests --
the plant, the LQR gain, the cached swing-up trajectory -- is session-scoped.
"""

from pathlib import Path

import numpy as np
import pytest

from cartpole.lqr import lqr_gain_and_cost_to_go
from cartpole.model import make_plant
from cartpole.swingup import load_trajectory

FIXTURE_TRAJECTORY = Path(__file__).parent / "fixtures" / "swingup.npz"


@pytest.fixture(scope="session")
def plant():
    return make_plant()


@pytest.fixture(scope="session")
def lqr():
    """(K, S) for the upright linearisation."""
    K, S = lqr_gain_and_cost_to_go()
    return np.asarray(K).reshape(4), np.asarray(S)


@pytest.fixture(scope="session")
def swingup():
    """The cached swing-up solution, as (state_trajectory, input_trajectory).

    Committed as a fixture rather than re-solved: the optimisation takes minutes,
    and a test suite nobody runs because it is slow protects nothing.
    """
    state, control = load_trajectory(FIXTURE_TRAJECTORY)
    assert state is not None, f"missing fixture {FIXTURE_TRAJECTORY}"
    return state, control
