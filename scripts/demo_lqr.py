"""Phase 1 demo: LQR balancing the cart-pole near upright.

    python scripts/demo_lqr.py            # small perturbation
    python scripts/demo_lqr.py 0.8        # 46 deg -- a visible catch
    python scripts/demo_lqr.py 1.3        # past the basin edge: cart runs away

The wiring happens here, at the top of the stack. sim.py never names a controller.
"""

import sys

import numpy as np

from cartpole.lqr import make_lqr_controller, report_linearization
from cartpole.model import UPRIGHT
from cartpole.sim import final_state, simulate


def main(perturbation=0.05, duration=30.0):
    report_linearization()

    theta0 = UPRIGHT - perturbation
    print(f"\nstarting {np.degrees(perturbation):.1f} deg from upright")

    _, plant, plant_context = simulate(
        theta0=theta0,
        duration=duration,
        controller=make_lqr_controller(),
    )

    x, theta, _, _ = final_state(plant, plant_context)
    print(f"final: x = {x:.4f} m, tilt = {np.degrees(theta - UPRIGHT):.3f} deg")


if __name__ == "__main__":
    main(float(sys.argv[1]) if len(sys.argv) > 1 else 0.05)
    input("Simulation finished. Press Enter to shut down the MeshCat server.")
