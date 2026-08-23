"""Measure LQR's region of attraction along the pole-angle axis.

    python scripts/basin_sweep.py

Sweeps the initial pole angle away from upright and reports where the controller
stops recovering. Headless -- no visualization.

Two things to know about the result:

  * This is a ONE-DIMENSIONAL SLICE through a 4D basin. Every run starts at rest
    with the cart centred. The same angle reached while swinging fast is a
    different point in state space and very likely outside the basin.

  * The system fails by RUNNING AWAY, not by dropping the pole. Past the edge the
    final angle is essentially arbitrary, so `x` is the reliable indicator --
    an angle-only criterion reports false successes.
"""

import numpy as np
from pydrake.systems.analysis import Simulator

from cartpole.lqr import make_lqr_controller
from cartpole.model import UPRIGHT, read_state, set_state
from cartpole.sim import build_cartpole

# Converged means the pole is upright AND the cart stayed on a plausible track.
ANGLE_TOL = 0.01  # rad
CART_TOL = 1.0    # m


def sweep(start=0.8, stop=2.01, step=0.1, duration=30.0):
    # Build ONCE. A System can belong to exactly one Diagram (builder.AddSystem
    # takes ownership), so the controller cannot be reused across builds. The
    # diagram is immutable after Build() anyway -- all mutable state lives in the
    # context, so we make a fresh context per sweep point instead.
    diagram, plant = build_cartpole(controller=make_lqr_controller(), meshcat=None)

    print(f"{'pert':>6} {'deg':>7} {'err (rad)':>11} {'x (m)':>10}   result")
    for pert in np.arange(start, stop, step):
        context = diagram.CreateDefaultContext()
        plant_context = plant.GetMyContextFromRoot(context)
        set_state(plant, plant_context, theta=UPRIGHT - pert)

        simulator = Simulator(diagram, context)
        simulator.Initialize()
        simulator.AdvanceTo(duration)

        x, theta, _, _ = read_state(plant, plant_context)
        err = abs(theta - UPRIGHT)
        ok = err < ANGLE_TOL and abs(x) < CART_TOL
        print(f"{pert:6.2f} {np.degrees(pert):7.1f} {err:11.4f} {x:10.2f}   "
              f"{'OK' if ok else 'FAILED'}")


if __name__ == "__main__":
    sweep()
