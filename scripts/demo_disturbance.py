"""What a limit-unaware controller does. Phase 1 and 3, visually.

    python scripts/demo_disturbance.py             # MeshCat
    python scripts/demo_disturbance.py --headless  # numbers only

An honest note on what this does and does not show.

I went looking for a case where MPC visibly outperforms a saturated LQR and did
not find one. Across u_max from 6 to 12 N and shoves from 0.3 to 0.6 rad/s, both
recover every time, and MPC generally uses slightly MORE track. On this system,
clipping LQR works about as well.

What MPC actually buys is not performance, it is guarantees:

  * its command respects the force limit BY CONSTRUCTION, where LQR has to be
    clipped afterwards -- and clipping voids the Riccati stability argument LQR
    rests on, so it works here by observation rather than by theorem;
  * it can express a track limit at all, which LQR structurally cannot.

For a cart-pole with room to manoeuvre, that guarantee may not be worth much.
For a system where breaching a limit is catastrophic, it is the whole point.

Act 3 is the one with real visual drama, and it is about neither: it is what
happens when a LOCAL controller is used outside its region of attraction.
"""

import sys

import numpy as np
from pydrake.geometry import Box, Rgba
from pydrake.math import RigidTransform
from pydrake.systems.analysis import Simulator

from cartpole.controllers import StateFeedback
from cartpole.lqr import lqr_gain_and_cost_to_go
from cartpole.model import UPRIGHT, read_state, set_state
from cartpole.mpc import U_MAX, make_mpc_controller
from cartpole.sim import build_cartpole

KICK = 0.6         # rad/s: enough that LQR asks for more than 20 N
BASIN_ANGLE = 1.3  # rad from upright -- just outside the measured basin
DURATION = 12.0
SAMPLES = 600
TRACK = 10.0       # wide enough not to bind, so force is the only limit in play


def draw_marks(meshcat, x_max):
    """Red posts at +/- x_max. Without them a position limit is invisible."""
    if meshcat is None:
        return
    meshcat.Delete("track")
    for sign in (-1.0, 1.0):
        meshcat.SetObject(f"track/post{sign:+.0f}", Box(0.02, 0.4, 0.8),
                          Rgba(0.85, 0.15, 0.15, 0.65))
        meshcat.SetTransform(f"track/post{sign:+.0f}",
                             RigidTransform([sign * x_max, 0.0, 0.15]))


def announce(title, body, interactive):
    print("\n" + "=" * 70)
    print(f"  {title}")
    print("=" * 70)
    print(body)
    if interactive:
        input("\nPress Enter to run this act...")


def run(controller, meshcat, label, theta0=None, thetadot=KICK, x_max=TRACK):
    draw_marks(meshcat, x_max)
    diagram, plant = build_cartpole(controller=controller, meshcat=meshcat)
    context = diagram.CreateDefaultContext()
    plant_context = plant.GetMyContextFromRoot(context)
    set_state(plant, plant_context,
              theta=UPRIGHT if theta0 is None else theta0, thetadot=thetadot)

    simulator = Simulator(diagram, context)
    if meshcat is not None:
        simulator.set_target_realtime_rate(1.0)
    simulator.Initialize()
    controller_context = controller.GetMyContextFromRoot(context)

    peak_u = peak_x = 0.0
    for t in np.linspace(0.0, DURATION, SAMPLES):
        simulator.AdvanceTo(t)
        peak_u = max(peak_u, abs(controller.get_output_port(0).Eval(controller_context)[0]))
        peak_x = max(peak_x, abs(read_state(plant, plant_context)[0]))

    _x, theta, _xd, _td = read_state(plant, plant_context)
    upright = abs((theta - UPRIGHT + np.pi) % (2 * np.pi) - np.pi) < 0.02
    over = "  <-- EXCEEDS THE 20 N ACTUATOR" if peak_u > U_MAX * 1.01 else ""
    print(f"{label:<24} peak|u|={peak_u:7.1f} N   peak|x|={peak_x:8.2f} m   "
          f"{'balanced' if upright else 'lost the pole'}{over}")


def main(headless=False):
    meshcat = None
    if not headless:
        from pydrake.geometry import StartMeshcat
        meshcat = StartMeshcat()

    K, _S = lqr_gain_and_cost_to_go()
    interactive = meshcat is not None
    print(f"actuator limit {U_MAX:.0f} N; track deliberately wide ({TRACK:.0f} m) "
          f"so force is the only limit in play\n")

    announce(
        "ACT 1 -- LQR, unconstrained",
        f"A {KICK} rad/s shove at upright. LQR recovers it easily. Watch the\n"
        "commanded force in the summary line: it asks for more than the 20 N\n"
        "the actuator can deliver. On hardware that command does not happen --\n"
        "you get 20 N and whatever behaviour follows from that.",
        interactive,
    )
    run(StateFeedback(K), meshcat, "LQR unconstrained")

    announce(
        "ACT 2 -- MPC, same shove",
        "The force limit is a row in the QP, so the command is inside 20 N by\n"
        "construction rather than by clipping afterwards. On screen this looks\n"
        "much like act 1 -- the difference is that this one is executable.",
        interactive,
    )
    run(make_mpc_controller(x_max=TRACK), meshcat, "MPC")

    announce(
        "ACT 3 -- LQR outside its region of attraction",
        f"Start the pole {BASIN_ANGLE} rad ({np.degrees(BASIN_ANGLE):.0f} deg) from upright, just\n"
        "past the measured basin edge of ~1.2 rad. LQR is a LOCAL controller\n"
        "built from a linearisation about upright; out here that model is\n"
        "fiction, and it confidently drives the cart into the distance.\n"
        "This is why the swing-up needs trajectory optimisation, not feedback.",
        interactive,
    )
    run(StateFeedback(K), meshcat, "LQR past the basin",
        theta0=UPRIGHT - BASIN_ANGLE, thetadot=0.0)


if __name__ == "__main__":
    main(headless="--headless" in sys.argv)
    if "--headless" not in sys.argv:
        input("\nPress Enter to shut down the MeshCat server.")
