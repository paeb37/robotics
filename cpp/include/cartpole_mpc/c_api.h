// Flat C ABI for the MPC solver.
//
// Deliberately C, not pybind11. Only plain doubles cross this boundary, so there
// is no ABI coupling to pydrake's build, nothing to go wrong with type
// conversion or ownership, and the same interface is what you would ship to a
// microcontroller. Python calls it through ctypes.
#ifndef CARTPOLE_MPC_C_API_H
#define CARTPOLE_MPC_C_API_H

#ifdef __cplusplus
extern "C" {
#endif

// Everything else in this library is compiled with hidden visibility; these
// three symbols are the entire public surface.
#define CARTPOLE_API __attribute__((visibility("default")))

typedef struct CartpoleMpc CartpoleMpc;

// Matrices are row-major: Ad 4x4, Bd 4x1, Q 4x4, R 1x1, S 4x4.
// Pass x_max < 0 to disable the track constraint.
// Returns NULL on invalid arguments.
CARTPOLE_API CartpoleMpc* cartpole_mpc_create(
    const double* Ad, const double* Bd, const double* Q, const double* R,
    const double* S, int horizon, double dt, double u_max, double x_max,
    double slack_penalty, double eps_abs, double eps_rel, int polishing,
    int max_iter);

// Returns 1 on success (u_out written), 0 if the solve failed.
CARTPOLE_API int cartpole_mpc_solve(CartpoleMpc* handle, const double* e0,
                                    double* u_out);

// Solver iterations used by the last call; -1 if unavailable. Warm starting
// should drive this down to a handful, and it is the diagnostic that tells you
// whether the speedup came from the solver or from removing overhead.
CARTPOLE_API int cartpole_mpc_last_iterations(CartpoleMpc* handle);

// Raw OSQP status code from the last solve. Useful for telling "infeasible"
// apart from "hit the iteration limit" -- which are very different problems.
CARTPOLE_API int cartpole_mpc_last_status(CartpoleMpc* handle);

CARTPOLE_API void cartpole_mpc_destroy(CartpoleMpc* handle);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // CARTPOLE_MPC_C_API_H
