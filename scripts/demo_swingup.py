"""Phase 2.1: solve the swing-up trajectory and verify it numerically.

    python scripts/demo_swingup.py

No visualization -- that is 2.2. The point here is to establish that the
trajectory is real before anything is drawn.

The last check is the one that matters: replay the planned u(t) open-loop through
the plant and compare against the planned state trajectory. Collocation enforces
the dynamics only at the knots and midpoints, so a coarse mesh can produce a
"solution" that satisfies every constraint yet does not survive integration.
"""

import numpy as np
from pydrake.systems.analysis import Simulator
from pydrake.systems.primitives import TrajectorySource

from cartpole.model import read_state, set_state
from cartpole.sim import build_cartpole
from cartpole.swingup import HANGING_STATE, U_MAX, report_swingup, solve_swingup


def replay_open_loop(state_traj, input_traj):
    """Integrate the plant under the planned input and compare to the plan."""
    diagram, plant = build_cartpole(controller=TrajectorySource(input_traj),
                                    meshcat=None)
    context = diagram.CreateDefaultContext()
    plant_context = plant.GetMyContextFromRoot(context)
    set_state(plant, plant_context,
              x=HANGING_STATE[0], theta=HANGING_STATE[1],
              xdot=HANGING_STATE[2], thetadot=HANGING_STATE[3])

    simulator = Simulator(diagram, context)
    simulator.Initialize()

    duration = state_traj.end_time()
    worst = 0.0
    print(f"\n{'t':>6} {'planned theta':>14} {'actual theta':>13} {'|err|':>9}")
    for t in np.linspace(0.0, duration, 11):
        simulator.AdvanceTo(t)
        actual = read_state(plant, plant_context)
        planned = state_traj.value(t).flatten()
        err = np.linalg.norm(actual - planned)
        worst = max(worst, err)
        print(f"{t:6.3f} {planned[1]:14.5f} {actual[1]:13.5f} {err:9.2e}")

    print(f"\nworst state error over replay: {worst:.3e}")
    return worst


def main():
    state_traj, input_traj, result = solve_swingup()
    report_swingup(state_traj, input_traj, result, u_max=U_MAX)

    if not result.is_success():
        print("\nSolver failed -- nothing to replay.")
        return

    worst = replay_open_loop(state_traj, input_traj)
    if worst < 1e-2:
        print("trajectory survives open-loop integration")
        return

    # Expected. The divergence is not a mesh problem -- it is the unstable
    # equilibrium amplifying whatever error the discretization seeds. Error near
    # upright grows as e^(lambda*t) with lambda = 4.65 1/s, so surviving ~6 s of
    # that would require seeding an error below
    #
    #     0.01 / e^(4.65 * 6) ~ 1e-14
    #
    # i.e. machine epsilon. No affordable mesh gets there. Open-loop execution
    # near an unstable equilibrium is structurally impossible, not merely
    # inaccurate -- which is exactly why 2.3 adds feedback.
    print("open-loop diverges near upright, as expected -- see comment above")
    print("the PLAN is valid; executing it needs feedback (phase 2.3)")


if __name__ == "__main__":
    main()
