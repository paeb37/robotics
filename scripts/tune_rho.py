"""Find rho: the largest LQR cost-to-go sublevel set that is actually a basin.

    python scripts/tune_rho.py

Method: grid initial conditions over (theta error, thetadot), run LQR alone from
each, and record whether it converged. Then

    rho_max_safe  = smallest V(x) among the points that DIVERGED
    rho_reachable = largest  V(x) among the points that CONVERGED

Any rho below rho_max_safe gives a sublevel set containing no known-diverging
point. We take a safety factor below it, because a grid only samples the basin --
it does not certify it. Proving containment is what sum-of-squares programming is
for; this is the empirical stand-in.

Note this grids only (theta, thetadot) with the cart at rest at the origin. The
real basin is 4D. That is a knowingly incomplete slice, but it is the slice a
swing-up arrival actually lives in.
"""

import numpy as np
from pydrake.systems.analysis import Simulator

from cartpole.controllers import wrapped_error
from cartpole.lqr import lqr_gain_and_cost_to_go, make_lqr_controller
from cartpole.model import UPRIGHT, read_state, set_state
from cartpole.sim import build_cartpole

ANGLE_TOL = 0.01  # rad
CART_TOL = 1.0    # m
SAFETY = 0.5      # use this fraction of the largest safe rho


def converges(diagram, plant, theta_err, theta_dot, duration=30.0):
    context = diagram.CreateDefaultContext()
    plant_context = plant.GetMyContextFromRoot(context)
    set_state(plant, plant_context, theta=UPRIGHT + theta_err, thetadot=theta_dot)

    simulator = Simulator(diagram, context)
    simulator.Initialize()
    simulator.AdvanceTo(duration)

    x, theta, _, _ = read_state(plant, plant_context)
    return abs(theta - UPRIGHT) < ANGLE_TOL and abs(x) < CART_TOL


def main():
    _K, S = lqr_gain_and_cost_to_go()
    diagram, plant = build_cartpole(controller=make_lqr_controller(), meshcat=None)

    converged_V, diverged_V = [], []
    for theta_err in np.arange(-1.4, 1.41, 0.2):
        row = ""
        for theta_dot in np.arange(-4.0, 4.01, 0.5):
            state = np.array([0.0, UPRIGHT + theta_err, 0.0, theta_dot])
            e = wrapped_error(state)
            V = float(e @ S @ e)
            if converges(diagram, plant, theta_err, theta_dot):
                converged_V.append(V)
                row += "."
            else:
                diverged_V.append(V)
                row += "X"
        print(f"theta_err {theta_err:+.1f}  {row}")

    print(f"\ngrid: {len(converged_V)} converged, {len(diverged_V)} diverged")
    print(f"largest  V among converged: {max(converged_V):10.1f}")
    if diverged_V:
        smallest_bad = min(diverged_V)
        print(f"smallest V among diverged : {smallest_bad:10.1f}")
        print(f"\nrho (with {SAFETY:.0%} safety factor): {SAFETY * smallest_bad:.1f}")
    else:
        print("nothing diverged -- widen the grid")


if __name__ == "__main__":
    main()
