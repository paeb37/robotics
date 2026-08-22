# NOTES

Running technical notes, written in my own words. The rule: before moving to the next
phase, I write the explanation here without looking at any AI-generated explanation. If I
can't write it fluently, I'm not ready to move on.

The test: **could I explain this piece to an interviewer without looking at my own code?**

---

## Phase 0 — Environment

*(pending)*

---

## Phase 1 — Cart-pole model + LQR balancing

*(pending)*

Questions to be able to answer here:
- Why is the cart-pole underactuated, stated as a rank condition?
- What do `Q` and `R` actually trade off, and what happened when I changed them?
- LQR is a *local* stabilizer — how far from upright could I start before it failed, and why?

Notes:
- 
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
