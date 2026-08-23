"""Phase 2.3: swing up, then hand off to LQR at the top.

    python scripts/demo_swingup_lqr.py            # with MeshCat
    python scripts/demo_swingup_lqr.py --headless # numbers only

Three acts, and the middle one is a deliberate part of the story:

  1. the planned trajectory                (solved by direct collocation)
  2. open-loop execution                   (diverges near upright -- unavoidable)
  3. the same plan with LQR taking over    (works)

Act 2 is not a failed attempt. Error near upright grows as e^(lambda*t) with
lambda = 4.65 1/s, so open-loop execution would need its seeded error held below
machine epsilon to survive. Feedback is not an optimization here; it is the only
thing that can work.
"""

import sys

import numpy as np
from pydrake.systems.analysis import Simulator
from pydrake.systems.primitives import TrajectorySource

from cartpole.controllers import SwingUpAndBalance
from cartpole.lqr import lqr_gain_and_cost_to_go
from cartpole.model import HANGING, UPRIGHT, read_state, set_state
from cartpole.sim import build_cartpole
from cartpole.swingup import U_MAX, get_swingup

RHO = 4239.6  # from scripts/tune_rho.py


def announce(title, body, interactive):
    """Label the act and wait, so the two runs don't blur together on screen."""
    print("\n" + "=" * 70)
    print(f"  {title}")
    print("=" * 70)
    print(body)
    if interactive:
        input("\nPress Enter to run this act...")


def run(controller, duration, meshcat=None, label=""):
    diagram, plant = build_cartpole(controller=controller, meshcat=meshcat)
    context = diagram.CreateDefaultContext()
    plant_context = plant.GetMyContextFromRoot(context)
    set_state(plant, plant_context, theta=HANGING)

    simulator = Simulator(diagram, context)
    if meshcat is not None:
        simulator.set_target_realtime_rate(1.0)
    simulator.Initialize()
    simulator.AdvanceTo(duration)

    x, theta, xdot, thetadot = read_state(plant, plant_context)
    tilt = np.degrees((theta - UPRIGHT + np.pi) % (2 * np.pi) - np.pi)
    print(f"{label:<22} x={x:8.3f} m  tilt={tilt:9.3f} deg  "
          f"xdot={xdot:7.3f}  thetadot={thetadot:7.3f}")
    return abs(tilt) < 1.0 and abs(x) < 1.0


def main(headless=False):
    meshcat = None
    if not headless:
        from pydrake.geometry import StartMeshcat
        meshcat = StartMeshcat()

    state_traj, input_traj = get_swingup()
    plan_duration = state_traj.end_time()
    K, S = lqr_gain_and_cost_to_go()

    # Where the plan itself would trigger the handoff, if executed perfectly.
    controller = SwingUpAndBalance(input_traj, K, S, RHO)
    entry = next(
        (t for t in np.linspace(0, plan_duration, 2000)
         if controller.in_basin(state_traj.value(t).flatten())),
        None,
    )
    print(f"plan duration     : {plan_duration:.3f} s")
    print(f"rho               : {RHO}")
    print("plan enters basin : "
          + (f"{entry:.3f} s ({entry / plan_duration:.0%} through)"
             if entry is not None else "never -- rho is too tight"))
    print()

    interactive = meshcat is not None
    total = plan_duration + 15.0

    announce(
        "ACT 2 -- OPEN LOOP (no feedback)",
        "Replays the planned u(t) blindly, never looking at the pole.\n"
        "The swing-up starts correctly -- the plan is good. But near upright,\n"
        "error grows as e^(4.65t), doubling every 149 ms. The pole misses the\n"
        "top, tumbles, and the cart keeps following its recording downrange.",
        interactive,
    )
    run(TrajectorySource(input_traj), total, meshcat, "open-loop (act 2)")

    # Saturating LQR at the planned force limit. Unsaturated it commands 139 N --
    # seven times the limit the whole trajectory was designed around -- which
    # would make the demo work by quietly abandoning its own constraint.
    #
    # Handing off later (theta_max=0.5, thetadot_max=2.0) keeps LQR under 20 N
    # without saturation, but pushes the switch to 5.50 s, and the open-loop
    # replay blows up around 5.9 s. That is 0.4 s of margin. We take the earlier
    # handoff and saturate instead.
    #
    # Note that saturation here is luck, not a guarantee: clipping a controller's
    # output is not something LQR knows about, and it can destabilise. Knowing in
    # advance that the limit will be respected is what MPC buys (phase 3).
    same_for = (f"for the first {entry:.1f} s" if entry is not None
                else "until the state enters the basin")
    announce(
        "ACT 3 -- SWING-UP + LQR (feedback)",
        f"The SAME plan, byte for byte, {same_for} -- these two runs are\n"
        "identical up to the handoff. Then LQR takes over, and the controller\n"
        "starts reacting to where the pole actually is instead of where a\n"
        "recording says it should be.",
        interactive,
    )
    ok = run(SwingUpAndBalance(input_traj, K, S, RHO, u_limit=U_MAX), total,
             meshcat, "swing-up + LQR (act 3)")
    print("\n" + ("balanced at the top" if ok else "FAILED to balance"))


if __name__ == "__main__":
    main(headless="--headless" in sys.argv)
    if "--headless" not in sys.argv:
        input("Press Enter to shut down the MeshCat server.")
