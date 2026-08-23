"""Swing-up trajectory via direct collocation.

Produces DATA -- a state trajectory and an input trajectory -- not a controller.
Keeping the optimizer's output as plain trajectories means it can be inspected,
asserted on, and plotted without building a diagram or running a simulation.

The program:

    decision variables : state at each knot (4 x N), input at each knot (1 x N),
                         and the time steps
    x(0)               : [0, 0, 0, 0]   cart centred, pole hanging, at rest
    x(T)               : [0, pi, 0, 0]  cart centred, pole upright, at rest
    dynamics           : collocation constraints, 4 per interval, written by Drake
    input              : |u| <= u_max at every knot
    cost               : integral of u^2 -- minimum effort

This is NONCONVEX. There is no global-optimum guarantee and no closed form; the
solver finds *a* local solution and which one depends on the initial guess.
"""

from pathlib import Path

import numpy as np
from pydrake.planning import DirectCollocation
from pydrake.solvers import Solve
from pydrake.trajectories import PiecewisePolynomial

from cartpole.model import HANGING, UPRIGHT, make_plant

# The solve takes minutes at 100 knots, and downstream work re-runs constantly,
# so the result is cached as dense samples and rebuilt as a first-order hold.
CACHE_PATH = Path(__file__).resolve().parents[1] / "data" / "swingup.npz"
CACHE_SAMPLES = 1000

# The force limit and the horizon are coupled, and the coupling is physical: the
# pendulum's natural period is 2*pi/sqrt(g/l) ~ 1.42 s, and at 20 N the cart can
# only manage ~1.8 m/s^2 (about 18% of g), so the pole has to be pumped up over
# many swings. Less force therefore demands a longer horizon, and enough knots to
# resolve it. Measured feasibility frontier (straight-line initial guess):
#
#     u_max    knots   duration
#      20 N     100     11.78 s
#      30 N      60      8.03 s
#      40 N      50      6.36 s
#     100 N      30      4.97 s
#
# Tightening u_max below ~20 N will need a longer horizon again.
U_MAX = 20.0        # N, force limit on the cart
NUM_KNOTS = 100
MIN_TIME_STEP = 0.01
MAX_TIME_STEP = 0.40
DURATION_BOUNDS = (1.0, 20.0)  # seconds, total trajectory duration

HANGING_STATE = np.array([0.0, HANGING, 0.0, 0.0])
UPRIGHT_STATE = np.array([0.0, UPRIGHT, 0.0, 0.0])


def solve_swingup(u_max=U_MAX, num_knots=NUM_KNOTS, min_time_step=MIN_TIME_STEP,
                  max_time_step=MAX_TIME_STEP, duration_bounds=DURATION_BOUNDS,
                  guess_duration=12.0):
    """Solve the swing-up trajectory optimization.

    Returns (state_trajectory, input_trajectory, result). The trajectories are
    None if the solver failed -- always check result.is_success().
    """
    plant = make_plant()
    context = plant.CreateDefaultContext()

    # Constructing this object writes the collocation constraints: 4 equality
    # constraints per interval, derived from the plant's dynamics. For 30 knots
    # that is 116 constraints -- the bulk of the program, and invisible here.
    dircol = DirectCollocation(
        plant,
        context,
        num_time_samples=num_knots,
        minimum_time_step=min_time_step,
        maximum_time_step=max_time_step,
        input_port_index=plant.get_actuation_input_port().get_index(),
    )
    prog = dircol.prog()

    # Free final time, but all intervals the same length. Without this the
    # optimizer can bunch knots up where the dynamics are easy and starve the
    # parts that matter.
    dircol.AddEqualTimeIntervalsConstraints()
    dircol.AddDurationBounds(*duration_bounds)

    # Boundary conditions. Pinning x(T) = 0 as well as theta(T) = pi is the
    # harder problem, but it hands off to LQR exactly where LQR wants to be.
    prog.AddBoundingBoxConstraint(HANGING_STATE, HANGING_STATE, dircol.initial_state())
    prog.AddBoundingBoxConstraint(UPRIGHT_STATE, UPRIGHT_STATE, dircol.final_state())

    # The force limit -- the constraint LQR structurally could not express, and
    # the reason it happily commanded 474 N.
    u = dircol.input()
    dircol.AddConstraintToAllKnotPoints(u[0] <= u_max)
    dircol.AddConstraintToAllKnotPoints(-u_max <= u[0])

    dircol.AddRunningCost(u[0] ** 2)

    # Initial guess: straight-line interpolation in state from hanging to
    # upright, no guess for the input. Standard starting point for cart-pole.
    # Because the problem is nonconvex, this choice decides which local optimum
    # (if any) the solver walks to.
    state_guess = PiecewisePolynomial.FirstOrderHold(
        [0.0, guess_duration], np.column_stack((HANGING_STATE, UPRIGHT_STATE))
    )
    dircol.SetInitialTrajectory(PiecewisePolynomial(), state_guess)

    result = Solve(prog)
    if not result.is_success():
        return None, None, result

    return (
        dircol.ReconstructStateTrajectory(result),
        dircol.ReconstructInputTrajectory(result),
        result,
    )


def save_trajectory(state_traj, input_traj, path=CACHE_PATH):
    """Cache dense samples of the solution."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    times = np.linspace(0.0, state_traj.end_time(), CACHE_SAMPLES)
    np.savez(
        path,
        t=times,
        x=np.hstack([state_traj.value(t) for t in times]),
        u=np.hstack([input_traj.value(t) for t in times]),
    )


def load_trajectory(path=CACHE_PATH):
    """Rebuild the cached trajectories, or (None, None) if there is no cache."""
    path = Path(path)
    if not path.exists():
        return None, None
    data = np.load(path)
    return (
        PiecewisePolynomial.FirstOrderHold(data["t"], data["x"]),
        PiecewisePolynomial.FirstOrderHold(data["t"], data["u"]),
    )


def get_swingup(resolve=False, **kwargs):
    """Load the cached swing-up, solving it first if needed.

    Raises if the solve fails -- a caller wanting to inspect the failure should
    call solve_swingup directly.
    """
    if not resolve:
        state_traj, input_traj = load_trajectory()
        if state_traj is not None:
            return state_traj, input_traj

    state_traj, input_traj, result = solve_swingup(**kwargs)
    if not result.is_success():
        raise RuntimeError(f"swing-up solve failed: {result.get_solution_result()}")
    save_trajectory(state_traj, input_traj)
    return state_traj, input_traj


def report_swingup(state_traj, input_traj, result, u_max=U_MAX):
    """Print what kind of solution came back, not merely that one did."""
    print(f"solver        : {result.get_solver_id().name()}")
    print(f"success       : {result.is_success()}")
    if not result.is_success():
        print(f"status        : {result.get_solution_result()}")
        return

    duration = state_traj.end_time()
    times = np.linspace(0.0, duration, 400)
    states = np.hstack([state_traj.value(t) for t in times])
    inputs = np.hstack([input_traj.value(t) for t in times])

    theta_dot = states[3, :]
    swings = int(np.sum(np.diff(np.sign(theta_dot)) != 0))

    print(f"duration      : {duration:.3f} s")
    print(f"peak |u|      : {np.max(np.abs(inputs)):.3f} N   (limit {u_max})")
    print(f"effort int u^2: {np.trapezoid(inputs[0] ** 2, times):.3f}")
    print(f"thetadot sign changes: {swings}   (pumping swings)")
    print(f"x(0) = {np.round(state_traj.value(0.0).flatten(), 6)}")
    print(f"x(T) = {np.round(state_traj.value(duration).flatten(), 6)}")
    print(f"peak |x|      : {np.max(np.abs(states[0, :])):.3f} m")
