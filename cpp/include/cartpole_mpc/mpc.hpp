// Constrained linear MPC for the cart-pole, in coordinates e = x - x_upright.
//
// Deliberately knows nothing about Drake. The model matrices (Ad, Bd, Q, R, S)
// are computed once in Python -- that is what Drake is for -- and handed over at
// construction. At run time this solves the same QP the Python backend does:
//
//     minimise    sum_k  dt*(e[k]' Q e[k] + u[k]' R u[k])  +  e[N]' S e[N]
//                        + slack_penalty * sum_k s[k]
//     subject to  e[k+1] = Ad e[k] + Bd u[k]
//                 |u[k]| <= u_max
//                 |e[k+1][0]| <= x_max + s[k],   s[k] >= 0
//                 e[0]  = measured state
//
// This split -- model identification offline, fixed matrices on the target -- is
// how embedded controllers are actually built, and is the same structure TinyMPC
// uses.
#pragma once

#include <memory>
#include <vector>

namespace cartpole {

struct MpcOptions {
  int horizon = 30;
  double dt = 0.01;
  double u_max = 20.0;
  double x_max = 2.0;          // negative disables the track constraint
  double slack_penalty = 1e4;

  // Solver tolerances. Exposed because they are a real engineering tradeoff:
  // tighter costs iterations, and this problem is badly scaled (S entries ~6e3,
  // Q ~1, dt 1e-2, slack penalty 1e4), so an over-tight tolerance simply fails
  // to converge within the iteration cap.
  // Loose ADMM tolerance plus polishing. This ordering is deliberate and was
  // measured: polishing solves the reduced KKT system exactly once the active
  // set is identified, so it delivers a machine-precision answer regardless of
  // how loosely ADMM converged. Tightening eps instead just burns iterations --
  // eps 1e-5 without polishing cost 13x the median solve time and was LESS
  // accurate than eps 1e-3 with it.
  double eps_abs = 1e-3;
  double eps_rel = 1e-3;
  bool polishing = true;
  int max_iter = 4000;
};

class Mpc {
 public:
  static constexpr int kNumStates = 4;
  static constexpr int kNumInputs = 1;

  // Matrices arrive row-major: Ad is 4x4, Bd 4x1, Q 4x4, R 1x1, S 4x4.
  Mpc(const double* Ad, const double* Bd, const double* Q, const double* R,
      const double* S, const MpcOptions& options);
  ~Mpc();

  Mpc(const Mpc&) = delete;
  Mpc& operator=(const Mpc&) = delete;

  // Solve for the first input given the measured error state. Returns true on
  // success. The sparse structure is built once in the constructor; each call
  // only updates the bounds pinning e[0] and re-solves, warm-started from the
  // previous solution.
  bool Solve(const double* e0, double* u_out);

  int last_iterations() const;

  // Raw OSQP status_val from the last Solve(); 0 if never solved.
  int last_status() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace cartpole
