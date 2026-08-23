"""ctypes bridge to the C++ MPC solver.

Only plain doubles cross this boundary. That is the point: no pybind11, no ABI
coupling to pydrake's build, and the same flat interface you would ship to a
microcontroller.

The C++ library is optional. If it has not been built, `available()` returns
False and the Python backend is used -- nothing in the project requires a
compiler to run.
"""

import ctypes
from pathlib import Path

import numpy as np

_REPO_ROOT = Path(__file__).resolve().parents[1]
_CANDIDATES = [
    _REPO_ROOT / "build" / "libcartpole_mpc.dylib",
    _REPO_ROOT / "build" / "libcartpole_mpc.so",
]

_lib = None
_load_error = None

# OSQP status_val meanings, for turning a failure into a diagnosis.
OSQP_STATUS = {
    1: "solved", 2: "solved_inaccurate", 3: "primal_infeasible",
    4: "primal_infeasible_inaccurate", 5: "dual_infeasible",
    6: "dual_infeasible_inaccurate", 7: "max_iter_reached",
    8: "time_limit_reached", 9: "non_convex", 10: "interrupted",
    11: "unsolved",
}


def _load():
    """Load the shared library once, caching success or failure."""
    global _lib, _load_error
    if _lib is not None or _load_error is not None:
        return _lib

    path = next((p for p in _CANDIDATES if p.exists()), None)
    if path is None:
        _load_error = (
            "libcartpole_mpc not found. Build it with:\n"
            "  cmake -S cpp -B build -DCMAKE_BUILD_TYPE=Release\n"
            "  cmake --build build"
        )
        return None

    lib = ctypes.CDLL(str(path))
    dbl = ctypes.POINTER(ctypes.c_double)

    lib.cartpole_mpc_create.restype = ctypes.c_void_p
    lib.cartpole_mpc_create.argtypes = [
        dbl, dbl, dbl, dbl, dbl,          # Ad, Bd, Q, R, S
        ctypes.c_int, ctypes.c_double,    # horizon, dt
        ctypes.c_double, ctypes.c_double, ctypes.c_double,  # u_max, x_max, penalty
        ctypes.c_double, ctypes.c_double,                   # eps_abs, eps_rel
        ctypes.c_int, ctypes.c_int,                         # polishing, max_iter
    ]
    lib.cartpole_mpc_solve.restype = ctypes.c_int
    lib.cartpole_mpc_solve.argtypes = [ctypes.c_void_p, dbl, dbl]
    lib.cartpole_mpc_last_iterations.restype = ctypes.c_int
    lib.cartpole_mpc_last_iterations.argtypes = [ctypes.c_void_p]
    lib.cartpole_mpc_last_status.restype = ctypes.c_int
    lib.cartpole_mpc_last_status.argtypes = [ctypes.c_void_p]
    lib.cartpole_mpc_destroy.restype = None
    lib.cartpole_mpc_destroy.argtypes = [ctypes.c_void_p]

    _lib = lib
    return _lib


def available():
    return _load() is not None


def unavailable_reason():
    _load()
    return _load_error


def _as_c_double_ptr(array):
    """Row-major contiguous float64 view, plus the pointer C expects.

    The array itself is returned too and must be kept alive by the caller --
    letting a temporary be garbage collected while C holds its pointer is the
    classic way to corrupt a ctypes call.
    """
    contiguous = np.ascontiguousarray(array, dtype=np.float64)
    return contiguous, contiguous.ctypes.data_as(ctypes.POINTER(ctypes.c_double))


class CppMpcBackend:
    """Owns one C++ solver instance. Same solve_qp contract as the Python path."""

    def __init__(self, Ad, Bd, Q, R, S, horizon, dt, u_max, x_max, slack_penalty,
                 eps_abs=1e-3, eps_rel=1e-3, polishing=True, max_iter=4000):
        lib = _load()
        if lib is None:
            raise RuntimeError(unavailable_reason())

        # Keep every array alive for the lifetime of the handle.
        self._keepalive = []
        pointers = []
        for matrix in (Ad, Bd, Q, R, S):
            array, pointer = _as_c_double_ptr(matrix)
            self._keepalive.append(array)
            pointers.append(pointer)

        self._handle = lib.cartpole_mpc_create(
            *pointers,
            int(horizon),
            float(dt),
            float(u_max),
            -1.0 if x_max is None else float(x_max),  # negative disables it
            float(slack_penalty),
            float(eps_abs),
            float(eps_rel),
            1 if polishing else 0,
            int(max_iter),
        )
        if not self._handle:
            raise RuntimeError("cartpole_mpc_create failed (invalid arguments?)")

        self._lib = lib
        self._e0 = np.zeros(4, dtype=np.float64)
        self._u = np.zeros(1, dtype=np.float64)
        self._e0_ptr = self._e0.ctypes.data_as(ctypes.POINTER(ctypes.c_double))
        self._u_ptr = self._u.ctypes.data_as(ctypes.POINTER(ctypes.c_double))

    def solve(self, e0):
        """Return the first input, or None if the solve failed."""
        self._e0[:] = e0
        ok = self._lib.cartpole_mpc_solve(self._handle, self._e0_ptr, self._u_ptr)
        return float(self._u[0]) if ok else None

    def last_iterations(self):
        return self._lib.cartpole_mpc_last_iterations(self._handle)

    def last_status(self):
        """Raw OSQP status_val. 1 = solved; see OSQP_STATUS for the rest."""
        return self._lib.cartpole_mpc_last_status(self._handle)

    def __del__(self):
        handle = getattr(self, "_handle", None)
        if handle:
            self._lib.cartpole_mpc_destroy(handle)
            self._handle = None
