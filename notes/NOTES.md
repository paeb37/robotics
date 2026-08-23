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

*(pending)*

Questions:
- Why MPC here rather than LQR, in one sentence?
- What exactly does LQR do when it hits the force limit, and why can't it anticipate it?
- Discrete-time stability is `|λ| < 1`, not `Re(λ) < 0`. Where did that bite me?

---

## Phase 4 — C++ controller and benchmarks

*(pending)*

---

## Phase 5 — Production layer

*(pending)*
