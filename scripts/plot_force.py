"""Figure: commanded force over time, LQR vs MPC, against the actuator limit.

    python scripts/plot_force.py            # writes docs/media/force.png
    python scripts/plot_force.py --show

This exists because the LQR-vs-MPC difference is invisible on video. Both
controllers balance the pole and both look the same in MeshCat; the entire
distinction is a number. Plotted against the limit line it is unmissable.

What the figure shows: given the same shove, LQR demands more force than the
actuator has. That command cannot be executed -- on hardware you get 20 N and
whatever follows. MPC has the limit as a row in its QP, so it plans within it.

What the figure does NOT show, and the README says so: MPC recovering *better*.
It does not. On this system a clipped LQR performs about as well. The difference
is that MPC's command is executable by construction, while clipping LQR discards
the guarantee its stability argument depends on.
"""

import sys
from pathlib import Path

import numpy as np
from pydrake.systems.analysis import Simulator

from cartpole.controllers import StateFeedback
from cartpole.lqr import lqr_gain_and_cost_to_go
from cartpole.model import UPRIGHT, read_state, set_state
from cartpole.mpc import U_MAX, make_mpc_controller
from cartpole.sim import build_cartpole

KICK = 0.6
DURATION = 6.0
SAMPLES = 1200
TRACK = 10.0  # wide enough not to bind, so force is the only limit in play
OUTPUT = Path(__file__).resolve().parents[1] / "docs" / "media" / "force.png"


def trace(controller):
    """Return (t, commanded force, cart position, tilt in degrees)."""
    diagram, plant = build_cartpole(controller=controller, meshcat=None)
    context = diagram.CreateDefaultContext()
    plant_context = plant.GetMyContextFromRoot(context)
    set_state(plant, plant_context, theta=UPRIGHT, thetadot=KICK)

    simulator = Simulator(diagram, context)
    simulator.Initialize()
    controller_context = controller.GetMyContextFromRoot(context)

    times = np.linspace(0.0, DURATION, SAMPLES)
    force, cart, tilt = [], [], []
    for t in times:
        simulator.AdvanceTo(t)
        force.append(controller.get_output_port(0).Eval(controller_context)[0])
        x, theta, _, _ = read_state(plant, plant_context)
        cart.append(x)
        tilt.append(np.degrees((theta - UPRIGHT + np.pi) % (2 * np.pi) - np.pi))
    return times, np.array(force), np.array(cart), np.array(tilt)


def main(show=False):
    import matplotlib
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    K, _S = lqr_gain_and_cost_to_go()
    t, f_lqr, x_lqr, a_lqr = trace(StateFeedback(K))
    _, f_mpc, x_mpc, a_mpc = trace(make_mpc_controller(x_max=TRACK))

    fig, axes = plt.subplots(3, 1, figsize=(8, 8), sharex=True)

    ax = axes[0]
    ax.axhspan(U_MAX, max(f_lqr.max(), U_MAX) * 1.15, color="#d62728", alpha=0.08)
    ax.axhspan(-max(abs(f_lqr.min()), U_MAX) * 1.15, -U_MAX, color="#d62728", alpha=0.08)
    ax.axhline(U_MAX, color="#d62728", ls="--", lw=1.2, label=f"actuator limit ({U_MAX:.0f} N)")
    ax.axhline(-U_MAX, color="#d62728", ls="--", lw=1.2)
    ax.plot(t, f_lqr, color="#1f77b4", lw=1.6, label="LQR (unconstrained)")
    ax.plot(t, f_mpc, color="#2ca02c", lw=1.6, label="MPC (constrained)")
    ax.set_ylabel("commanded force (N)")
    ax.set_title(f"Same {KICK} rad/s shove. LQR asks for {abs(f_lqr).max():.1f} N "
                 f"from a {U_MAX:.0f} N actuator.")
    ax.legend(loc="upper right", fontsize=9)
    ax.grid(alpha=0.25)

    axes[1].plot(t, a_lqr, color="#1f77b4", lw=1.6)
    axes[1].plot(t, a_mpc, color="#2ca02c", lw=1.6)
    axes[1].set_ylabel("pole tilt (deg)")
    axes[1].grid(alpha=0.25)

    axes[2].plot(t, x_lqr, color="#1f77b4", lw=1.6)
    axes[2].plot(t, x_mpc, color="#2ca02c", lw=1.6)
    axes[2].set_ylabel("cart position (m)")
    axes[2].set_xlabel("time (s)")
    axes[2].grid(alpha=0.25)

    fig.suptitle("Both stabilise. Only one of them is executable.", y=0.995,
                 fontsize=11, style="italic")
    fig.tight_layout()

    if show:
        plt.show()
    else:
        OUTPUT.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(OUTPUT, dpi=140, bbox_inches="tight")
        print(f"wrote {OUTPUT}")
        print(f"  LQR peak |u| = {abs(f_lqr).max():.1f} N   "
              f"MPC peak |u| = {abs(f_mpc).max():.1f} N")


if __name__ == "__main__":
    main(show="--show" in sys.argv)
