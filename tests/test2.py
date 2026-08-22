import numpy as np
from cartpole.sim import build_cartpole, make_lqr_controller, UPRIGHT
from pydrake.systems.analysis import Simulator

for pert in np.arange(0.8, 2.01, 0.1):
    diagram, plant = build_cartpole(controller=make_lqr_controller(), meshcat=None)
    ctx = diagram.CreateDefaultContext()
    pc = plant.GetMyContextFromRoot(ctx)
    plant.GetJointByName("PolePin").set_angle(pc, UPRIGHT - pert)
    sim = Simulator(diagram, ctx); sim.Initialize(); sim.AdvanceTo(30.0)
    err = abs(plant.GetJointByName("PolePin").get_angle(pc) - UPRIGHT)
    x = plant.GetJointByName("CartSlider").get_translation(pc)
    print(f"{pert:.1f} rad ({np.degrees(pert):5.1f} deg): err={err:7.4f} rad, x={x:8.2f} m  "
          f"{'OK' if err < 0.01 else 'FAILED'}")
