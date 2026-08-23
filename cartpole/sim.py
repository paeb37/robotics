"""Diagram assembly and simulation.

Controller-agnostic by design: this module takes a controller as a parameter and
never constructs one. Adding a new controller means touching controllers.py and a
script, not this file.
"""

from pydrake.geometry import StartMeshcat
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.visualization import AddDefaultVisualization

from cartpole.model import CART_POLE_URL, read_state, set_state


def build_cartpole(controller=None, meshcat=None, time_step=0.0):
    """Build the cart-pole diagram, optionally with a controller in the loop.

    `controller` is any System taking full plant state in and emitting plant
    actuation out -- the LQR AffineSystem, a swing-up controller, an MPC system.
    Pass None for the uncontrolled plant.

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
    # declares the ports.
    plant.Finalize()

    if controller is not None:
        # AddSystem hands back the system now owned by the builder -- that's the
        # handle we connect to. Note ownership is permanent: a System belongs to
        # exactly one Diagram, so a controller cannot be reused across builds.
        controller = builder.AddSystem(controller)

        # Feedback controllers take plant state in. An open-loop TrajectorySource
        # has no input port at all -- it is a pure function of time -- so only
        # wire the state connection when there is somewhere to wire it to.
        if controller.num_input_ports() > 0:
            # Closing the loop makes a cycle, and it's legal because the plant has
            # no direct feedthrough: actuation affects state only via integration.
            builder.Connect(plant.get_state_output_port(), controller.get_input_port(0))

        builder.Connect(controller.get_output_port(0), plant.get_actuation_input_port())

    if meshcat is not None:
        AddDefaultVisualization(builder, meshcat)

    return builder.Build(), plant


def simulate(theta0, duration=5.0, controller=None, meshcat=None, x0=0.0,
             realtime=True):
    """Simulate from an initial pole angle, with or without a controller.

    Set realtime=False for headless analysis runs -- otherwise the simulation is
    throttled to wall-clock speed so it can be watched.

    Returns (simulator, plant, plant_context) so callers can read the final state.
    """
    if meshcat is None:
        meshcat = StartMeshcat()

    diagram, plant = build_cartpole(controller=controller, meshcat=meshcat)

    # The diagram's context is the single source of truth. The plant context is
    # a VIEW into it, not a copy -- which is why we pull it from the root rather
    # than calling plant.CreateDefaultContext().
    diagram_context = diagram.CreateDefaultContext()
    plant_context = plant.GetMyContextFromRoot(diagram_context)
    set_state(plant, plant_context, x=x0, theta=theta0)

    if controller is None:
        # Nothing feeds actuation, and Drake errors on an unconnected input port.
        # Size 1: only CartSlider is actuated.
        plant.get_actuation_input_port().FixValue(plant_context, [0.0])

    simulator = Simulator(diagram, diagram_context)
    if realtime:
        simulator.set_target_realtime_rate(1.0)
    simulator.Initialize()
    simulator.AdvanceTo(duration)

    return simulator, plant, plant_context


def final_state(plant, plant_context):
    """Convenience: read [x, theta, xdot, thetadot] after a simulation."""
    return read_state(plant, plant_context)
