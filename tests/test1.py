from cartpole.sim import build_cartpole, make_lqr_controller, UPRIGHT
from pydrake.systems.analysis import Simulator
import numpy as np

diagram, plant = build_cartpole(controller=make_lqr_controller(), meshcat=None)
ctx = diagram.CreateDefaultContext()
pc = plant.GetMyContextFromRoot(ctx)
plant.GetJointByName("PolePin").set_angle(pc, UPRIGHT - 0.8)
sim = Simulator(diagram, ctx); sim.Initialize()
for t in np.arange(0, 30.1, 1.0):
    sim.AdvanceTo(t)
    x = plant.GetJointByName("CartSlider").get_translation(pc)
    th = np.degrees(plant.GetJointByName("PolePin").get_angle(pc) - UPRIGHT)
    print(f"{t:5.1f}  x={x:8.4f}  tilt={th:8.3f} deg")
