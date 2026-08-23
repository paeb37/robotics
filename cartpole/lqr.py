"""LQR design and linearization diagnostics.

Pure design: given the plant, produce a gain. Nothing here simulates.
"""

import numpy as np
from pydrake.systems.controllers import LinearQuadraticRegulator
from pydrake.systems.primitives import Linearize

from cartpole.model import make_plant, upright_context

# State ordering is [x, theta, xdot, thetadot] -- confirmed via
# plant.GetPositionNames() / GetVelocityNames().
#
# Arc-length normalization says a radian of pole angle is worth l*theta = 0.5 m
# of tip travel, which would put q_theta at a QUARTER of q_x. We deliberately go
# the other way and weight the angle higher: cart drift is recoverable, losing
# the pole past the linearization's validity is not.
DEFAULT_Q = np.diag([1.0, 4.0, 1.0, 4.0])

# R is 1x1 (one actuator). Only the Q:R ratio matters, so fixing R = 1 and
# tuning Q is the conventional normalization.
DEFAULT_R = np.array([[1.0]])


def make_lqr_controller(Q=DEFAULT_Q, R=DEFAULT_R):
    """Linearize about upright, solve the Riccati equation, return the controller.

    Returns an AffineSystem -- affine rather than linear because the control law
    is u = -K(x - x0) + u0, and that offset is a constant term. It consumes full
    plant state and emits plant actuation, so it drops straight into a diagram.
    """
    plant = make_plant()
    context = upright_context(plant)
    return LinearQuadraticRegulator(
        plant,
        context,
        Q,
        R,
        input_port_index=plant.get_actuation_input_port().get_index(),
    )


def linearize_upright():
    """Return the LinearSystem (A, B) for the plant linearized about upright."""
    plant = make_plant()
    context = upright_context(plant)
    return Linearize(
        plant,
        context,
        input_port_index=plant.get_actuation_input_port().get_index(),
        output_port_index=plant.get_state_output_port().get_index(),
    )


def report_linearization():
    """Print A, B, the open-loop eigenvalues, and the doubling time.

    The doubling time is the natural clock of the instability: how long an
    uncontrolled deviation takes to double. It sets the control-rate budget.
    """
    linear = linearize_upright()
    eigenvalues = np.linalg.eigvals(linear.A())
    unstable = max(eigenvalues.real)

    print("A =\n", np.round(linear.A(), 4))
    print("B =\n", np.round(linear.B(), 4))
    print("open-loop eigenvalues:", np.round(eigenvalues, 4))
    print(f"unstable lambda: {unstable:.4f} 1/s")
    print(f"doubling time  : {np.log(2) / unstable:.4f} s")
    return linear
