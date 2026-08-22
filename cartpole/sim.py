"""Cart-pole simulation setup.

Builds the Drake diagram for the cart-pole: a MultibodyPlant holding the physics,
a SceneGraph holding the geometry, and optional MeshCat visualization.

No controller here. Phase 1 step 1 is the passive system falling over.
"""

import numpy as np
from pydrake.geometry import StartMeshcat
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.visualization import AddDefaultVisualization

# Package URI, not a filesystem path -- resolves for anyone with Drake installed.
CART_POLE_URL = "package://drake/examples/multibody/cart_pole/cart_pole.sdf"

# In this SDF the pole's point mass sits at z = -0.5 relative to the cart when
# the joint angle is zero, i.e. hanging straight down. Upright is therefore pi.
UPRIGHT = np.pi


def build_cartpole(meshcat=None, time_step=0.0):
    """Build the cart-pole diagram.

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
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=time_step)

    Parser(plant).AddModelsFromUrl(CART_POLE_URL)

    # The "compile" step: fixes the tree topology, sizes the state vector,
    # declares the ports. Nothing below here would work before this call.
    plant.Finalize()

    if meshcat is not None:
        AddDefaultVisualization(builder, meshcat)

    diagram = builder.Build()
    return diagram, plant


def simulate_passive(theta0=UPRIGHT - 0.05, duration=5.0, meshcat=None):
    """Simulate with zero actuation. The pole should fall over."""
    if meshcat is None:
        meshcat = StartMeshcat()

    diagram, plant = build_cartpole(meshcat)

    # The diagram's context is the single source of truth. The plant context is
    # a VIEW into it, not a copy -- which is why we pull it from the root rather
    # than calling plant.CreateDefaultContext(). That would give us an unrelated
    # context whose state the simulator never reads.
    diagram_context = diagram.CreateDefaultContext()
    plant_context = plant.GetMyContextFromRoot(diagram_context)

    plant.GetJointByName("CartSlider").set_translation(plant_context, 0.0)
    plant.GetJointByName("PolePin").set_angle(plant_context, theta0)

    # No controller yet, but the actuation input port still has to be fed or
    # Drake errors on the unconnected port. Size 1: only CartSlider is actuated.
    plant.get_actuation_input_port().FixValue(plant_context, [0.0])

    simulator = Simulator(diagram, diagram_context)
    simulator.set_target_realtime_rate(1.0)  # run at wall-clock speed so it's watchable
    simulator.Initialize()
    simulator.AdvanceTo(duration)

    return simulator


if __name__ == "__main__":
    simulate_passive()
    input("Simulation finished. Press Enter to shut down the MeshCat server.")
