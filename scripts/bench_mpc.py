"""Phase 4 step 1: does building the QP once instead of every step fix the deadline?

    python scripts/bench_mpc.py

Three implementations of the SAME controller, measured over the same closed-loop
run so the state sequence is identical:

    rebuild            reconstruct the whole program every step (the baseline)
    reuse              build once, move the initial-state bound
    reuse + warm start build once, and seed OSQP with the previous solution

Timing covers the FULL per-step cost -- construction plus solve -- because that
is what has to fit inside the 10 ms control period. An earlier version timed only
the solve and therefore flattered the baseline.

The deadline is 10 ms (100 Hz). Missing it does not lose the pole -- the 149 ms
doubling time gives plenty of slack -- but a controller that cannot hold its own
advertised rate is not a controller you would ship.
"""

import numpy as np
from pydrake.systems.analysis import Simulator

from cartpole.model import UPRIGHT, read_state, set_state
from cartpole.mpc import CONTROL_DT, make_mpc_controller
from cartpole.sim import build_cartpole

DEADLINE_MS = CONTROL_DT * 1e3
KICK = 0.3       # inside what 20 N can handle, so every variant recovers
DURATION = 8.0


def measure(label, **kwargs):
    mpc = make_mpc_controller(**kwargs)
    diagram, plant = build_cartpole(controller=mpc, meshcat=None)
    context = diagram.CreateDefaultContext()
    plant_context = plant.GetMyContextFromRoot(context)
    set_state(plant, plant_context, theta=UPRIGHT, thetadot=KICK)

    simulator = Simulator(diagram, context)
    simulator.Initialize()
    simulator.AdvanceTo(DURATION)

    _x, theta, _xd, _td = read_state(plant, plant_context)
    recovered = abs((theta - UPRIGHT + np.pi) % (2 * np.pi) - np.pi) < 0.02

    times_ms = np.array(mpc.solve_times) * 1e3
    misses = int(np.sum(times_ms > DEADLINE_MS))
    return {
        "label": label,
        "n": len(times_ms),
        "p50": np.percentile(times_ms, 50),
        "p99": np.percentile(times_ms, 99),
        "max": times_ms.max(),
        "misses": misses,
        "miss_pct": 100.0 * misses / len(times_ms),
        "recovered": recovered,
    }


def main():
    print(f"deadline {DEADLINE_MS:.0f} ms/step ({1/CONTROL_DT:.0f} Hz), "
          f"kick {KICK} rad/s, {DURATION} s per run\n")

    rows = [
        measure("rebuild every step", rebuild=True, warm_start=False),
        measure("build once, reuse", rebuild=False, warm_start=False),
        measure("reuse + warm start", rebuild=False, warm_start=True),
    ]

    header = f"{'implementation':<22} {'p50':>8} {'p99':>8} {'max':>8} {'misses':>14}  ok"
    print(header)
    print("-" * len(header))
    for r in rows:
        print(f"{r['label']:<22} {r['p50']:7.2f}ms {r['p99']:7.2f}ms {r['max']:7.2f}ms "
              f"{r['misses']:5d}/{r['n']:<4d} {r['miss_pct']:4.1f}%  "
              f"{'yes' if r['recovered'] else 'NO'}")

    base, best = rows[0], rows[-1]
    print(f"\nspeedup p50: {base['p50'] / best['p50']:.1f}x   "
          f"max: {base['max'] / best['max']:.1f}x")
    print("deadline met" if best["max"] < DEADLINE_MS
          else f"still missing: max {best['max']:.2f}ms > {DEADLINE_MS:.0f}ms")


if __name__ == "__main__":
    main()
