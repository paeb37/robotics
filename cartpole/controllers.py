"""Controllers that plug into a diagram.

Anything here takes full plant state in and emits plant actuation out, so it
drops straight into build_cartpole(controller=...).
"""

import numpy as np
from pydrake.systems.framework import LeafSystem

from cartpole.model import UPRIGHT

UPRIGHT_STATE = np.array([0.0, UPRIGHT, 0.0, 0.0])

# The box scripts/tune_rho.py actually sampled. The sublevel set V <= rho is only
# meaningful inside it; outside, rho is validated by nothing.
THETA_MAX = 1.4      # rad
THETADOT_MAX = 4.0   # rad/s


def wrapped_error(x, x_star=UPRIGHT_STATE):
    """(x - x_star) with the angle component wrapped into [-pi, pi].

    Upright is theta = pi, so a pole at theta = -3.10 is 0.04 rad away, not 6.24.
    A swing-up rotates the pole through multiple revolutions, so without this the
    error term is nonsense and any basin test built on it never fires.
    """
    e = np.asarray(x, dtype=float) - x_star
    e[1] = (e[1] + np.pi) % (2 * np.pi) - np.pi
    return e


class StateFeedback(LeafSystem):
    """u = -K(x - x*), optionally clipped to +/- u_limit.

    Exists so LQR can be compared against MPC on equal terms, with and without
    saturation. Clipping is worth calling out: LQR's stability guarantee comes
    from a Riccati equation that assumes u is unbounded, so the moment the output
    is clipped that guarantee is void. It may still work -- but nothing says it
    has to, and that is precisely the gap MPC closes by planning within the limit
    instead of truncating afterwards.
    """

    def __init__(self, K, u_limit=None):
        LeafSystem.__init__(self)
        self._K = np.asarray(K).reshape(4)
        self._u_limit = u_limit
        self.DeclareVectorInputPort("state", 4)
        self.DeclareVectorOutputPort("actuation", 1, self._calc_actuation)

    def _calc_actuation(self, context, output):
        u = float(-self._K @ wrapped_error(self.get_input_port(0).Eval(context)))
        if self._u_limit is not None:
            u = float(np.clip(u, -self._u_limit, self._u_limit))
        output.SetFromVector([u])


class SwingUpAndBalance(LeafSystem):
    """Replay a planned swing-up, then hand off to LQR inside the basin.

    The switch criterion is the LQR cost-to-go, V(x) = e' S e <= rho, NOT an
    angle threshold. The region of attraction is a 4D region: a pole at the right
    angle but whipping past at speed is a different point in state space and
    generally outside it. Since a swing-up arrives with the pole moving, an
    angle-only trigger would fire in exactly the condition where it misleads.

    Note there is deliberately no latch. The output is a pure function of the
    context, which is Drake's contract -- outputs are cached and the integrator
    may retry steps, so a Python-side mutable flag would be silently wrong. In
    practice a latch is unnecessary: once LQR has the state inside the basin it
    keeps it there, so the test does not oscillate.
    """

    def __init__(self, input_traj, K, S, rho, theta_max=THETA_MAX,
                 thetadot_max=THETADOT_MAX, u_limit=None):
        LeafSystem.__init__(self)
        self._traj = input_traj
        self._K = np.asarray(K).reshape(4)  # flat, so -K @ e is a scalar
        self._S = np.asarray(S)
        self._rho = float(rho)
        self._theta_max = float(theta_max)
        self._thetadot_max = float(thetadot_max)
        self._u_limit = u_limit
        self._t_end = input_traj.end_time()

        self.DeclareVectorInputPort("state", 4)
        self.DeclareVectorOutputPort("actuation", 1, self._calc_actuation)

    def cost_to_go(self, x):
        e = wrapped_error(x)
        return float(e @ self._S @ e)

    def in_basin(self, x):
        """V(x) <= rho AND inside the box the sublevel set was validated over.

        The box is not belt-and-braces, it is load-bearing. S comes from the
        linearization about upright, so V is a LOCAL model -- evaluating it far
        from upright extrapolates a local model globally, which is the same
        mistake that made LQR command 474 N. Its large theta/thetadot cross-term
        means a badly-tilted, fast-moving pole can score a small V purely because
        it is moving "back towards" upright, and the ellipsoid V <= rho reaches
        well outside the region tune_rho actually sampled.
        """
        e = wrapped_error(x)
        if abs(e[1]) > self._theta_max or abs(e[3]) > self._thetadot_max:
            return False
        return self.cost_to_go(x) <= self._rho

    def _calc_actuation(self, context, output):
        x = self.get_input_port(0).Eval(context)

        if self.in_basin(x):
            u = float(-self._K @ wrapped_error(x))
        else:
            # Clamp past the end: if the plan runs out before the state enters
            # the basin, hold the final input rather than extrapolating.
            t = min(context.get_time(), self._t_end)
            u = float(np.asarray(self._traj.value(t)).flatten()[0])

        if self._u_limit is not None:
            u = float(np.clip(u, -self._u_limit, self._u_limit))

        output.SetFromVector([u])
