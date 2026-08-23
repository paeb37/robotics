"""The plant itself: shape, state handling, and the equilibria."""

import numpy as np

from cartpole.model import (
    HANGING,
    POLE_LENGTH,
    UPRIGHT,
    read_state,
    set_state,
    upright_context,
)


def test_underactuation(plant):
    """rank(B) = 1 < 2. This is the premise of the whole project."""
    assert plant.num_positions() == 2      # x, theta
    assert plant.num_velocities() == 2
    assert plant.num_actuators() == 1      # force on the cart only
    assert plant.get_actuation_input_port().size() == 1


def test_state_ordering(plant):
    """[x, theta, xdot, thetadot]. Q's diagonal is meaningless if this changes."""
    assert plant.GetPositionNames() == ["CartPole_CartSlider_x", "CartPole_PolePin_q"]
    assert plant.GetVelocityNames() == ["CartPole_CartSlider_v", "CartPole_PolePin_w"]


def test_state_roundtrip(plant):
    context = plant.CreateDefaultContext()
    written = np.array([1.5, 2.0, -3.0, 0.25])
    set_state(plant, context, *written)
    np.testing.assert_allclose(read_state(plant, context), written)


def _derivatives(plant, context):
    return plant.EvalTimeDerivatives(context).CopyToVector()


def test_upright_is_an_equilibrium(plant):
    """Zero velocity, zero input, theta = pi -> zero acceleration."""
    context = upright_context(plant)
    np.testing.assert_allclose(_derivatives(plant, context), np.zeros(4), atol=1e-12)


def test_hanging_is_an_equilibrium(plant):
    context = plant.CreateDefaultContext()
    set_state(plant, context, theta=HANGING)
    plant.get_actuation_input_port().FixValue(context, [0.0])
    np.testing.assert_allclose(_derivatives(plant, context), np.zeros(4), atol=1e-12)


def test_upright_and_hanging_differ_by_half_a_turn():
    assert np.isclose(UPRIGHT - HANGING, np.pi)


def test_pole_falls_from_near_upright(plant):
    """Sanity: upright is unstable, so perturbing it produces motion away from pi."""
    context = plant.CreateDefaultContext()
    set_state(plant, context, theta=UPRIGHT - 0.05)
    plant.get_actuation_input_port().FixValue(context, [0.0])
    thetaddot = _derivatives(plant, context)[3]
    # theta < pi, so gravity should drive it further from upright: thetaddot < 0.
    assert thetaddot < -1.0


def test_pole_length_matches_the_sdf(plant):
    """POLE_LENGTH is used for the arc-length argument behind Q; keep it honest."""
    context = plant.CreateDefaultContext()
    set_state(plant, context, theta=HANGING)
    pole = plant.GetBodyByName("Pole")
    cart = plant.GetBodyByName("Cart")
    offset = (plant.EvalBodyPoseInWorld(context, pole).translation()
              - plant.EvalBodyPoseInWorld(context, cart).translation())
    assert np.isclose(np.linalg.norm(offset), POLE_LENGTH, atol=1e-9)
