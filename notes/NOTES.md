# NOTES

Running technical notes, written in my own words

---

## Phase 0 — Environment

Setup - done.

---

## Phase 1 — Cart-pole model + LQR balancing

- Works when the initial position of pole is close to the upright position (pi)
- However, when lower (i.e. upright - 2.0 rad) then our LQR controller applies a very high force (474 N) to the cart, to make the pole swing up
- This causes the cart to zoom out of the frame (-> 359 m) in meshcat ...

- Specific limit: 1.3 rad (Tested)

Lessons
- To debug, better to print out the actual x, u values rather than visually inspect in meshcat
- Control period needs to be fraction of doubling period (time it takes for error to double)
- Basin: the range of states where the robot will reach equilibrium

---

## Phase 2 — Swing-up via direct collocation

- Collocation enables us to have both state, control as decision variables
- Introduces constraints as well (i.e. motor cannot exert force more than 20N)
- So direct colloc. still computes the trajectory offline (i.e. before cart moves)

Results
- While the direct colloc. part respected the 20N constraint, we had an issue when switching it off to LQR (near the upright equilibrium point)
- LQR commanded 139 (which is 7 times the planned limit)
- This motivates the reason for MPC (need to respect constraints at all times)

"Act 2" - no feedback, just does the traj. initially planned. results in the cart moving far out of the plane (because discretization errors which compound)
"Act 3" - here, we switch to LQR near the upright point so it succeeds. reacts to where the pole actually is

---

## Phase 3 — Constrained MPC vs. LQR

Weaknesses of phase 2 / why we need MPC
- The first 4.3 seconds of the simulation are "open-loop" - no feedback based on the updated state
- If we have a different model (i.e. cart), starting angle/pos, then it will fail to solve

- MPC factors in constraints (i.e. motor force) into the QP equation
- Re-plans with each timestep
- So we use MPC just to replace the LQR towards the top. The initial part of the swinging is still open loop (to fix we need TVLQR or nonlinear MPC along the entire traj. Out of scope)


Results
- Slow, because it is solving a QP every timestep (not just matrix multiply, like LQR is)
- Needed a soft constraint on state (since robot state can be affected by things outside our control) but hard constraint on control (motor) still
- Confirmed unconstrained MPC (with terminal Riccati cost) reproduced LQR first
- MPC sits under the 20N limit, unlike LQR + the track length limit
- p99 = 9.09 ms (99% of the solves took ~9ms or less)

Latency baseline (the number to beat)
---
Config: Python, OSQP, horizon N=50 (0.5 s), dt=0.01 (100 Hz),
        QP rebuilt from scratch every step, 1200 solves, M-series Mac.

    p50   3.03 ms
    p99   9.09 ms     <- 1 miss/sec at 100 Hz
    max  14.60 ms     <- 46% over the 10 ms control period

Deadline      : 10 ms   (control period at 100 Hz)
Stability slack: 149 ms  (doubling time -- why missing a deadline
                          degrades performance rather than losing the pole)

Phase 4 target: max < 10 ms, i.e. never miss the deadline.
Suspected win : build the QP once and update only the changing parts,
                instead of reconstructing ~250 variables and ~350
                constraints every step.
---

Next steps
- To fix latency issue, need to port to C++

---

## Phase 4 — C++ controller and benchmarks

Context (why C++)
OSQP is being asked to do this: at N=30: ~180 variables, ~250 constraints, banded structure, warm-started so it converges in a handful of iterations

It is taking 3.32 ms. So somewhere between 90% and 99% of that time is not OSQP solving — it's Drake traversing the program, marshalling bindings into solver format, and crossing the Python/C++ boundary on every call.

A direct implementation that builds the sparse CSC matrices once and calls osqp_update_bounds + osqp_solve skips all of it. I'd expect 10× or more speedup


Phase 4: C++ MPC backend (Eigen + OSQP v1.0, ctypes, ~180 LOC C++)
  median  8-12x faster (0.176 ms flat vs 1.5-2.2 ms)
  p99     better at low disturbance, comparable at high
  max     WORSE: two spikes >10 ms at active-set transitions
  correctness: closed-loop trajectories match to 1.9e-15 in the
               unconstrained regime; diverge where the objective is
               flat in u[0] under active slack

Not an unqualified win: better typical case, worse tail. For hard
real-time the tail is what binds. Next step would be OSQP time_limit
+ fallback for predictable degradation, but out of scope for now

---

## Phase 5 — Production layer

Docker - done.