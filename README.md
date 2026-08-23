# Cart-Pole Control

[![CI](https://github.com/paeb37/robotics/actions/workflows/ci.yml/badge.svg)](https://github.com/paeb37/robotics/actions/workflows/ci.yml)

Swing-up and constrained balancing of an underactuated cart-pole, built on
[Drake](https://drake.mit.edu). Trajectory optimization plans the swing-up, LQR
catches it at the top, and model-predictive control balances it while respecting
the actuator and track limits. The MPC hot loop has a C++ implementation behind a
C ABI, benchmarked against the Python one in the same control loop.

| Open-loop | With feedback |
|:--:|:--:|
| ![Open-loop swing-up fails](docs/media/swingup_openloop.gif) | ![Swing-up with feedback succeeds](docs/media/swingup_feedback.gif) |
| The planned forces replayed blind. The swing-up itself is correct — then the pole is lost near the top and the cart leaves the frame. | The **identical** forces, byte for byte, for the first 4.3 s. Then feedback takes over and holds it. |

*Both clips run at 4× speed and start from the same state. The plan was never
the problem; executing it open-loop was — error near the unstable equilibrium
grows as `e^(4.65t)`, doubling every 149 ms.*

---

## The system

A cart on a rail with a freely-swinging pole. Two configuration variables
(`x`, `θ`) and **one actuator** — a force on the cart. Nothing acts on the pole
directly, so `rank(B) = 1 < 2`: the system is underactuated everywhere, and there
is no torque you can command that cancels the pole's dynamics.

The upright equilibrium is unstable with eigenvalue `λ = +4.6456 1/s`, so a
deviation **doubles every 149 ms**. That number sets everything downstream — the
control rate, the MPC horizon, and what counts as an acceptable solve time.

## What it does

**1. Swing-up by direct collocation.** From hanging at rest to upright at rest,
subject to `|u| ≤ 20 N`. The solver is handed the force limit and produces an
11.78 s trajectory with **eight pumping swings** — it discovers resonant energy
pumping on its own, because at 20 N it cannot simply lift the pole.

**2. Open-loop execution fails, unavoidably.** Replaying the planned forces
without feedback loses the pole every time. This is not a mesh-resolution
problem: error near upright grows as `e^(4.65t)`, so surviving six seconds of it
would require seeding an error below `1e-14`. That is machine epsilon. **Open-loop
execution near an unstable equilibrium is structurally impossible, not merely
inaccurate.**

**3. LQR catches it.** The controller hands off from the plan to LQR once the
state enters the region of attraction — tested with the LQR cost-to-go,
`V(x) = eᵀSe ≤ ρ`, not an angle threshold, because the basin is a 4D region and
a pole at the right angle moving quickly is a different point in state space.

**4. MPC balances it within limits.** Force and track limits are rows in a
quadratic program re-solved every 10 ms.

## Results

### The limits are real, and only one controller knows it

![Commanded force vs the actuator limit](docs/media/force.png)

Given the same disturbance, LQR asks for **30.8 N from a 20 N actuator**. That
command cannot be executed. MPC has the limit in its QP and plans within it.

Pushed further outside its design envelope, LQR gets worse in a way that is
easy to miss when reading only the angle:

| initial tilt | LQR outcome |
|---|---|
| ≤ 1.2 rad (69°) | recovers, cart settles at 0.00 m |
| 1.3 rad (74°) | pole nearly recovered — 353 m down the track |
| 2.0 rad (115°) | commands 474 N, cart reaches 359 m |

The basin edge is a cliff, not a slope, and **the failure mode is running away,
not dropping the pole** — so an angle-only success criterion reports false
positives.

### Honest finding: MPC is not faster or better here

I expected MPC to outperform a saturated LQR under a weak actuator. **It does
not.** Across `u_max` from 6–12 N and disturbances from 0.3–0.6 rad/s, both
recover every time, and MPC generally uses slightly more track.

What MPC actually buys is *guarantees*, not performance:

- its command respects the force limit **by construction**, where LQR must be
  clipped afterwards — and clipping voids the Riccati stability argument LQR
  rests on, so a clipped LQR works by observation rather than by theorem;
- it can express a track limit at all, which LQR structurally cannot.

For a cart-pole with room to manoeuvre, that may not be worth much. For a system
where breaching a limit is catastrophic, it is the entire point.

### Latency: C++ backend

Same `LeafSystem`, same diagram, same closed loop — only the QP solver differs,
so the comparison is controlled. Full per-step cost (construction + solve),
first 20 solves discarded as warmup.

| disturbance | backend | p50 | p99 | max | over 10 ms |
|---|---|---|---|---|---|
| 0.3 rad/s | python | 2.160 ms | 2.963 ms | 3.627 ms | 0 / 980 |
| 0.3 rad/s | **c++** | **0.176 ms** | 0.696 ms | 1.636 ms | 0 / 980 |
| 1.0 rad/s | python | 1.492 ms | 2.717 ms | 3.523 ms | 0 / 980 |
| 1.0 rad/s | **c++** | **0.178 ms** | 1.626 ms | 12.634 ms | 1 / 980 |

**8–12× on the median**, and notably flat — 0.176 ms at every disturbance level.

**But the C++ tail is worse.** Two solves exceed the 10 ms control period at
higher disturbances, which Python never does. Those are ADMM iteration bursts at
active-set transitions (I first blamed OSQP's polishing step; disabling it
reproduced the spikes, so that was wrong). For a hard real-time loop the tail is
the number that binds, so **this is not an unqualified win.**

Closed-loop trajectories from the two backends agree to **1.9e-15** in the
unconstrained regime.

## How it works

```
cartpole/
  model.py        the plant: SDF, joint names, constants, state helpers
  lqr.py          LQR design and linearization diagnostics
  swingup.py      direct collocation -> a trajectory (data, not a controller)
  controllers.py  Systems that plug into a diagram
  mpc.py          constrained MPC, python or c++ backend
  cpp_backend.py  ctypes bridge
  sim.py          diagram assembly -- names no controller
cpp/              Eigen + OSQP solver behind a flat C ABI
```

The dependency graph is a DAG with `sim.py` depending only on `model.py`. Adding
a controller touches `controllers.py` and a script — never the simulator.

**Two design decisions worth calling out.** `swingup.py` returns a *trajectory*
rather than a controller, so the optimizer's output can be inspected, asserted on
and cached without building a diagram. And the C++ boundary is a flat C ABI
called via `ctypes`, not pybind11: only plain doubles cross it, so there is no ABI
coupling to pydrake's build, and it is the same interface you would ship to a
microcontroller. The library exports exactly four symbols; OSQP is statically
linked with hidden visibility so it cannot collide with the copy inside
`libdrake.so`.

## Setup

Requires macOS 15+ (arm64) or Ubuntu 24.04/26.04, Python 3.13 or 3.14. On macOS
use Homebrew Python — Drake does not support Apple's system Python.

```bash
git clone https://github.com/paeb37/robotics.git
cd robotics

python3 -m venv .venv
source .venv/bin/activate
pip install -e ".[dev]"
```

Verify:

```bash
python -c "
import pydrake.all
from pydrake.solvers import OsqpSolver, SnoptSolver, IpoptSolver
print('OSQP', OsqpSolver().available(), '| SNOPT', SnoptSolver().available(),
      '| IPOPT', IpoptSolver().available())"
```

The C++ backend is optional; everything runs without it.

```bash
brew install cmake eigen          # or: apt install cmake libeigen3-dev
cmake -S cpp -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

## Docker

Multi-stage: the C++ solver is compiled in a builder image and only its shared
library is carried into the runtime, so the runtime ships no compiler, no Eigen
and no OSQP sources. The build ends in a smoke test that solves a QP through
both backends and compares them — a broken image fails the build rather than
shipping.

```bash
docker build -t cartpole-control .
docker run --rm cartpole-control                    # latency benchmark
docker run --rm cartpole-control python scripts/compare_lqr_mpc.py
```

~500 MB, almost entirely pydrake. Ubuntu 24.04 because that is a Drake-supported
platform; note Drake also needs `libx11-6 libsm6 libglib2.0-0t64` at runtime even
headless, or `import pydrake` fails.

## Tests

```bash
pytest                # 38 tests, ~2.5 s
pytest -m slow        # also re-solves the trajectory optimisation (minutes)
```

The swing-up trajectory is committed as a fixture rather than re-solved. Several
tests encode bugs that actually happened — MPC matching LQR when unconstrained
(which would have caught a `dt` scaling error), and QP feasibility with the cart
outside the track (which would have caught hard state constraints). Both were
verified by reintroducing the bugs and confirming the suite fails.

## Running

```bash
python scripts/demo_swingup_lqr.py    # swing-up: open-loop vs feedback
python scripts/demo_disturbance.py    # limits, and what ignoring them costs
python scripts/basin_sweep.py         # measure LQR's region of attraction
python scripts/compare_lqr_mpc.py     # LQR vs MPC constraint satisfaction
python scripts/bench_mpc.py           # latency, python vs c++
python scripts/plot_force.py          # regenerate the force figure
```

Visualization is served by MeshCat at a local URL printed on startup. Demo
scripts pause between acts; pass `--headless` for numbers only.

## What I would do differently

**Fix the latency tail before the median.** The C++ backend is 8–12× faster
typically but occasionally blows the deadline. OSQP's `time_limit` setting plus
a defined fallback would make degradation predictable, which matters more than
another 2× on the median.

**Stabilize the swing-up trajectory.** The first 4.3 s are open-loop with no
disturbance rejection at all. TVLQR along the trajectory would close that gap;
it is the clearest missing piece.

**Certify the region of attraction instead of sampling it.** `ρ` is tuned by
gridding initial conditions. Sum-of-squares programming would produce a proven
inner estimate rather than an empirical one — and my sampled version needed a
hard box gate anyway, because the sublevel set of a cost-to-go derived from a
linearization inherits that linearization's locality. Trusting `V(x)` far from
upright repeats the exact mistake it was meant to guard against.

**Model error.** Everything here plans and executes against the same model, so
the plan is validated against a tautology. Perturbing the plant's masses relative
to the controller's model is the obvious next experiment.

## Notes

Development notes, including the bugs and what they taught, are in
[notes/NOTES.md](notes/NOTES.md). The ones worth knowing about: a `dt` factor
that made MPC 10× too timid and was caught only by an equivalence check against
LQR; hard state constraints that made 1125 of 1200 QPs infeasible; and IEEE
`infinity()` where OSQP wanted its own sentinel, which broke convergence at every
tolerance.

## License

MIT
