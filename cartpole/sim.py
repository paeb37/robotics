"""Cart-pole simulation and LQR stabilization.

Builds the Drake diagram for the cart-pole: a MultibodyPlant holding the physics,
a SceneGraph holding the geometry, and optional MeshCat visualization.

Phase 1 step 1: `simulate_passive` -- no controller, the pole falls.
Phase 1 step 2: `simulate_lqr`     -- LQR balances the pole near upright.
"""

import numpy as np
from pydrake.geometry import StartMeshcat
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph, MultibodyPlant
from pydrake.systems.analysis import Simulator
from pydrake.systems.controllers import LinearQuadraticRegulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.primitives import Linearize
from pydrake.visualization import AddDefaultVisualization

# Package URI, not a filesystem path -- resolves for anyone with Drake installed.
CART_POLE_URL = "package://drake/examples/multibody/cart_pole/cart_pole.sdf"

# In this SDF the pole's point mass sits at z = -0.5 relative to the cart when
# the joint angle is zero, i.e. hanging straight down. Upright is therefore pi.
UPRIGHT = np.pi

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


def _analysis_plant():
    """A standalone finalized plant, with its context at the upright equilibrium.

    No SceneGraph: geometry plays no part in dynamics, and leaving it out avoids
    any unconnected-geometry-port questions. Continuous time (time_step=0.0).

    The context is set to the equilibrium *pair* (x*, u*) -- state at upright AND
    actuation fixed at zero. Linearizing about a state alone is meaningless.

    Returns (plant, context).
    """
    plant = MultibodyPlant(time_step=0.0)
    Parser(plant).AddModelsFromUrl(CART_POLE_URL)
    plant.Finalize()

    context = plant.CreateDefaultContext()
    plant.GetJointByName("CartSlider").set_translation(context, 0.0)
    plant.GetJointByName("PolePin").set_angle(context, UPRIGHT)
    plant.SetVelocities(context, np.zeros(plant.num_velocities()))
    plant.get_actuation_input_port().FixValue(context, [0.0])
    return plant, context


def make_lqr_controller(Q=DEFAULT_Q, R=DEFAULT_R):
    """Linearize about upright, solve the Riccati equation, return the controller.

    Returns an AffineSystem -- affine rather than linear because the control law
    is u = -K(x - x0) + u0, and that offset is a constant term. It consumes full
    plant state and emits plant actuation, so it drops straight into the diagram.
    """
    plant, context = _analysis_plant()
    return LinearQuadraticRegulator(
        plant,
        context,
        Q,
        R,
        input_port_index=plant.get_actuation_input_port().get_index(),
    )


def report_linearization():
    """Print A, B, the open-loop eigenvalues, and the doubling time."""
    plant, context = _analysis_plant()

    linear = Linearize(
        plant,
        context,
        input_port_index=plant.get_actuation_input_port().get_index(),
        output_port_index=plant.get_state_output_port().get_index(),
    )

    # we need to print the eigenvalues to see how unstable the system is, and how fast it will diverge
    eigenvalues = np.linalg.eigvals(linear.A())
    unstable = max(eigenvalues.real)

    print("A =\n", np.round(linear.A(), 4))
    print("B =\n", np.round(linear.B(), 4))
    print("open-loop eigenvalues:", np.round(eigenvalues, 4))
    print(f"unstable lambda: {unstable:.4f} 1/s")
    print(f"doubling time  : {np.log(2) / unstable:.4f} s")
    return linear


def build_cartpole(controller=None, meshcat=None, time_step=0.0):
    """Build the cart-pole diagram, optionally with a controller in the loop.

    `controller` is any System taking full plant state in and emitting plant
    actuation out -- the LQR AffineSystem today, an MPC system later. Pass None
    for the uncontrolled plant.

    time_step=0.0 makes the plant continuous-time, which is what we want for
    LQR: it linearizes to xdot = Ax + Bu. A nonzero time_step would make it
    discrete (x[k+1] = Ad x[k] + Bd u[k]) and require the discrete-time LQR.

    Returns (diagram, plant). The plant is returned separately because callers
    need it to reach into the diagram's context and to query the model.
    """
    builder = DiagramBuilder()

    # One call creates BOTH the plant and the scene_graph and wires them
    # together -- plant publishes body poses to scene_graph, scene_graph
    # reports geometry back.
    plant, _scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=time_step)

    Parser(plant).AddModelsFromUrl(CART_POLE_URL)

    # The "compile" step: fixes the tree topology, sizes the state vector,
    # declares the ports
    plant.Finalize()

    if controller is not None:
        # AddSystem hands back the system now owned by the builder -- that's the
        # handle we connect to.
        controller = builder.AddSystem(controller)

        # Close the loop. This is a cycle, and it's legal because the plant has
        # no direct feedthrough: actuation affects state only through integration.
        builder.Connect(plant.get_state_output_port(), controller.get_input_port(0))
        builder.Connect(controller.get_output_port(0), plant.get_actuation_input_port())

    if meshcat is not None:
        AddDefaultVisualization(builder, meshcat)

    return builder.Build(), plant


def simulate(theta0=UPRIGHT - 0.05, duration=5.0, controller=None, meshcat=None):
    """Simulate the cart-pole from theta0, with or without a controller."""
    if meshcat is None:
        meshcat = StartMeshcat()

    diagram, plant = build_cartpole(controller=controller, meshcat=meshcat)

    # The diagram's context is the single source of truth. The plant context is
    # a VIEW into it, not a copy.
    diagram_context = diagram.CreateDefaultContext()
    plant_context = plant.GetMyContextFromRoot(diagram_context)

    plant.GetJointByName("CartSlider").set_translation(plant_context, 0.0)
    plant.GetJointByName("PolePin").set_angle(plant_context, theta0)

    if controller is None:
        # Nothing feeds actuation, and Drake errors on an unconnected input port.
        # Size 1: only CartSlider is actuated.
        plant.get_actuation_input_port().FixValue(plant_context, [0.0])

    simulator = Simulator(diagram, diagram_context)
    simulator.set_target_realtime_rate(1.0)
    simulator.Initialize()
    simulator.AdvanceTo(duration)

    final_theta = plant.GetJointByName("PolePin").get_angle(plant_context)
    print(f"theta: {theta0:.4f} -> {final_theta:.4f} (upright = {UPRIGHT:.4f})")
    return simulator


def simulate_passive(theta0=UPRIGHT - 0.05, duration=5.0, meshcat=None):
    """Simulate with zero actuation. The pole should fall."""
    return simulate(theta0, duration, controller=None, meshcat=meshcat)


def simulate_lqr(theta0=UPRIGHT - 0.8, duration=5.0, Q=DEFAULT_Q, R=DEFAULT_R,
                 meshcat=None):
    """Simulate with LQR closing the loop. The pole should stay up."""
    print(np.degrees(theta0 - UPRIGHT), "degrees from upright")
    
    return simulate(theta0, duration, controller=make_lqr_controller(Q, R),
                    meshcat=meshcat)


if __name__ == "__main__":
    report_linearization()

    angle = UPRIGHT - 2.0 # testing extreme angle (farther from upright)
    #simulate_passive(theta0=UPRIGHT - 0.05, duration=30.0)   # falls and swings forever
    simulate_lqr(theta0=angle, duration=30.0)
    
    input("Simulation finished. Press Enter to shut down the MeshCat server.")
