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

---

## Phase 2 — Swing-up via direct collocation

*(pending)*

Questions:
- Why collocation rather than shooting?
- What are the decision variables and constraints in the program I wrote?
- Why does swing-up need trajectory optimization at all instead of feedback?

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
