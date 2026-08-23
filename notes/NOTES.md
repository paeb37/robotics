# NOTES

Running technical notes, written in my own words. The rule: before moving to the next
phase, I write the explanation here without looking at any AI-generated explanation. If I
can't write it fluently, I'm not ready to move on.

The test: **could I explain this piece to an interviewer without looking at my own code?**

---

## Phase 0 — Environment

Done

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

Phase 4 step 1 (Python - time horizon 30, dt 0.01, 800 steps):
  naive rebuild      p50 11.89  p99 16.32  max 35.52   65% deadline misses
  reuse + warm start p50  3.32  p99  4.17  max  7.87    0% misses
  3.6x p50, 4.5x max. Deadline (10 ms) met.

Results: program reuse 1.4x; warm start 2.6x and p99 tail 14.2 -> 4.2 ms;
      horizon 50 -> 30 free (terminal cost = LQR cost-to-go).
Solve cost scales ~N^2.5 -> banded structure not exploited -> C++ hypothesis.



---

## Phase 5 — Production layer

*(pending)*
