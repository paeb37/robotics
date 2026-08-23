"""The cart-pole model: what the robot is.

No control, no simulation. This is the single place that knows the model file,
the joint names, and the physical constants -- everything else imports from here.
"""

import numpy as np
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import MultibodyPlant

# Package URI, not a filesystem path -- resolves for anyone with Drake installed.
CART_POLE_URL = "package://drake/examples/multibody/cart_pole/cart_pole.sdf"

CART_JOINT = "CartSlider"  # prismatic, actuated
POLE_JOINT = "PolePin"     # revolute, effort limit 0 -- this IS the underactuation

# From cart_pole.sdf.
CART_MASS = 10.0   # kg
POLE_MASS = 1.0    # kg
POLE_LENGTH = 0.5  # m

# The pole's point mass sits at z = -0.5 relative to the cart when the joint
# angle is zero, i.e. hanging straight down. Upright is therefore pi.
HANGING = 0.0
UPRIGHT = np.pi


def make_plant(time_step=0.0):
    """A standalone finalized plant with no SceneGraph.

    Geometry plays no part in dynamics, so leaving SceneGraph out keeps analysis
    (linearization, trajectory optimization) free of geometry-port questions.

    time_step=0.0 is continuous time, which linearizes to xdot = Ax + Bu. Nonzero
    makes it discrete (x[k+1] = Ad x[k] + Bd u[k]) and requires discrete-time LQR.
    """
    plant = MultibodyPlant(time_step=time_step)
    Parser(plant).AddModelsFromUrl(CART_POLE_URL)
    plant.Finalize()
    return plant


def set_state(plant, context, x=0.0, theta=0.0, xdot=0.0, thetadot=0.0):
    """Write [x, theta, xdot, thetadot] into a plant context."""
    plant.GetJointByName(CART_JOINT).set_translation(context, x)
    plant.GetJointByName(CART_JOINT).set_translation_rate(context, xdot)
    plant.GetJointByName(POLE_JOINT).set_angle(context, theta)
    plant.GetJointByName(POLE_JOINT).set_angular_rate(context, thetadot)


def read_state(plant, context):
    """Read [x, theta, xdot, thetadot] out of a plant context."""
    return np.array([
        plant.GetJointByName(CART_JOINT).get_translation(context),
        plant.GetJointByName(POLE_JOINT).get_angle(context),
        plant.GetJointByName(CART_JOINT).get_translation_rate(context),
        plant.GetJointByName(POLE_JOINT).get_angular_rate(context),
    ])


def upright_context(plant):
    """A context at the upright equilibrium PAIR (x*, u*).

    State at upright AND actuation fixed at zero. An equilibrium is a
    state/input pair; linearizing about a state alone is meaningless.
    """
    context = plant.CreateDefaultContext()
    set_state(plant, context, theta=UPRIGHT)
    plant.get_actuation_input_port().FixValue(context, [0.0])
    return context
