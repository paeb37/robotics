#include "cartpole_mpc/c_api.h"

#include <new>

#include "cartpole_mpc/mpc.hpp"

using cartpole::Mpc;
using cartpole::MpcOptions;

struct CartpoleMpc {
  Mpc mpc;
  CartpoleMpc(const double* Ad, const double* Bd, const double* Q,
              const double* R, const double* S, const MpcOptions& options)
      : mpc(Ad, Bd, Q, R, S, options) {}
};

extern "C" {

CartpoleMpc* cartpole_mpc_create(const double* Ad, const double* Bd,
                                 const double* Q, const double* R,
                                 const double* S, int horizon, double dt,
                                 double u_max, double x_max,
                                 double slack_penalty, double eps_abs,
                                 double eps_rel, int polishing, int max_iter) {
  if (!Ad || !Bd || !Q || !R || !S || horizon < 1 || dt <= 0.0 ||
      u_max <= 0.0) {
    return nullptr;
  }
  MpcOptions options;
  options.horizon = horizon;
  options.dt = dt;
  options.u_max = u_max;
  options.x_max = x_max;
  options.slack_penalty = slack_penalty;
  options.eps_abs = eps_abs;
  options.eps_rel = eps_rel;
  options.polishing = polishing != 0;
  options.max_iter = max_iter;

  // The boundary must not leak exceptions -- C callers cannot catch them, and
  // unwinding across the ABI is undefined.
  try {
    return new CartpoleMpc(Ad, Bd, Q, R, S, options);
  } catch (...) {
    return nullptr;
  }
}

int cartpole_mpc_solve(CartpoleMpc* handle, const double* e0, double* u_out) {
  if (!handle || !e0 || !u_out) return 0;
  try {
    return handle->mpc.Solve(e0, u_out) ? 1 : 0;
  } catch (...) {
    return 0;
  }
}

int cartpole_mpc_last_iterations(CartpoleMpc* handle) {
  if (!handle) return -1;
  return handle->mpc.last_iterations();
}

int cartpole_mpc_last_status(CartpoleMpc* handle) {
  if (!handle) return 0;
  return handle->mpc.last_status();
}

void cartpole_mpc_destroy(CartpoleMpc* handle) { delete handle; }

}  // extern "C"
