"""Phase 4: MPC solve latency, Python vs C++.

    python scripts/bench_mpc.py

Measures the FULL per-step cost -- QP construction plus solve -- because that is
what has to fit inside the 10 ms control period. An earlier version timed only
the solve and flattered the naive implementation by about 6x.

The first WARMUP solves are discarded. They are dominated by one-off costs (cold
caches, first allocations, lazy initialisation) and including them put the
reported max at index 0 for nearly every configuration -- measuring startup, not
steady-state control.

The deadline is 10 ms (100 Hz). Missing it does not lose the pole: the 149 ms
doubling time leaves plenty of slack, so a missed deadline degrades performance
rather than causing failure. But a controller that cannot hold its advertised
rate is not one you would ship.
"""

import numpy as np
from pydrake.systems.analysis import Simulator

from cartpole.cpp_backend import available as cpp_available
from cartpole.cpp_backend import unavailable_reason
from cartpole.model import UPRIGHT, read_state, set_state
from cartpole.mpc import CONTROL_DT, make_mpc_controller
from cartpole.sim import build_cartpole

DEADLINE_MS = CONTROL_DT * 1e3
WARMUP = 20
DURATION = 10.0
SAMPLES = 501
# (kick rad/s, track limit m). The track has to be wide enough that recovery is
# physically possible -- see compare_lqr_mpc.py.
CASES = [(0.3, 2.0), (0.6, 2.0), (0.8, 3.0), (1.0, 4.0)]


def run(kick, x_max, **kwargs):
    """One closed-loop run. Returns (trajectory, per-step times in ms)."""
    mpc = make_mpc_controller(x_max=x_max, **kwargs)
    diagram, plant = build_cartpole(controller=mpc, meshcat=None)
    context = diagram.CreateDefaultContext()
    plant_context = plant.GetMyContextFromRoot(context)
    set_state(plant, plant_context, theta=UPRIGHT, thetadot=kick)

    simulator = Simulator(diagram, context)
    simulator.Initialize()
    trajectory = []
    for t in np.linspace(0.0, DURATION, SAMPLES):
        simulator.AdvanceTo(t)
        trajectory.append(read_state(plant, plant_context).copy())

    return np.array(trajectory), np.array(mpc.solve_times)[WARMUP:] * 1e3


def main():
    configs = [("python", {"backend": "python"})]
    if cpp_available():
        configs.append(("c++", {"backend": "cpp"}))
    else:
        print(f"C++ backend unavailable:\n{unavailable_reason()}\n")

    print(f"deadline {DEADLINE_MS:.0f} ms/step ({1 / CONTROL_DT:.0f} Hz), "
          f"{DURATION} s per run, first {WARMUP} solves discarded\n")
    header = (f"{'kick':>5} {'x_max':>6} {'backend':<8} {'p50':>8} {'p99':>8} "
              f"{'max':>8} {'misses':>7} {'traj err':>10}")
    print(header)
    print("-" * len(header))

    speedups = []
    for kick, x_max in CASES:
        reference = None
        for name, kwargs in configs:
            trajectory, times = run(kick, x_max, **kwargs)
            if reference is None:
                reference, error = trajectory, 0.0
                baseline_p50 = np.percentile(times, 50)
            else:
                error = np.max(np.abs(trajectory - reference))
                speedups.append(baseline_p50 / np.percentile(times, 50))
            print(f"{kick:5.1f} {x_max:6.1f} {name:<8} "
                  f"{np.percentile(times, 50):7.3f}m {np.percentile(times, 99):7.3f}m "
                  f"{times.max():7.3f}m {int((times > DEADLINE_MS).sum()):3d}/{len(times):<4d} "
                  f"{error:10.2e}")
        print()

    if speedups:
        print(f"median-case speedup: {min(speedups):.1f}x - {max(speedups):.1f}x")
    print(
        "Notes:\n"
        "  traj err  -- max |state difference| vs the Python run over the whole\n"
        "               closed loop. Machine precision in the unconstrained\n"
        "               regime; larger once the track constraint binds, where the\n"
        "               objective is nearly flat in u[0] and two solvers can land\n"
        "               on different near-optimal points.\n"
        "  misses    -- solves exceeding the control period. The C++ backend wins\n"
        "               decisively on the median but shows occasional spikes at\n"
        "               active-set transitions, which is the number that matters\n"
        "               for hard real-time."
    )


if __name__ == "__main__":
    main()
