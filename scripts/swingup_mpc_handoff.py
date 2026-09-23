"""Experiment: the same swing-up, handing off to MPC instead of clipped LQR.

    python scripts/swingup_mpc_handoff.py

demo_swingup_lqr.py only balances because LQR's output is clipped to 20 N; at the
handoff it asks for far more. MPC plans within the limit, so does it catch the
same handoff without relying on clipping?
"""

import numpy as np
from demo_swingup_lqr import RHO
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder, LeafSystem

from cartpole.controllers import SwingUpAndBalance, wrapped_error
from cartpole.lqr import lqr_gain_and_cost_to_go
from cartpole.model import HANGING, UPRIGHT, read_state, set_state
from cartpole.mpc import make_mpc_controller
from cartpole.sim import build_cartpole
from cartpole.swingup import U_MAX, get_swingup

SAMPLE_DT = 0.001


class PlanThenBalance(LeafSystem):
    """Replay the plan until the state is in the basin, then pass through input 1.

    Same switch as SwingUpAndBalance (reused via `gate`), but the balancing command
    comes from another system, so MPC keeps its own 100 Hz zero-order hold.
    """

    def __init__(self, input_traj, gate):
        LeafSystem.__init__(self)
        self._traj = input_traj
        self._gate = gate
        self.DeclareVectorInputPort("state", 4)
        self.DeclareVectorInputPort("balance", 1)
        self.DeclareVectorOutputPort("actuation", 1, self._calc_actuation)

    def _calc_actuation(self, context, output):
        if self._gate.in_basin(self.get_input_port(0).Eval(context)):
            output.SetFromVector(self.get_input_port(1).Eval(context))
        else:
            t = min(context.get_time(), self._traj.end_time())
            output.SetFromVector(np.asarray(self._traj.value(t)).flatten())


def mpc_swingup(input_traj, gate):
    builder = DiagramBuilder()
    mpc = builder.AddSystem(make_mpc_controller())
    switch = builder.AddSystem(PlanThenBalance(input_traj, gate))
    state = builder.ExportInput(switch.get_input_port(0), "state")
    builder.ConnectInput(state, mpc.get_input_port(0))
    builder.Connect(mpc.get_output_port(0), switch.get_input_port(1))
    builder.ExportOutput(switch.get_output_port(0), "actuation")
    return builder.Build(), mpc


def trial(label, controller, gate, K, duration):
    """Run the swing-up; report only while the balancing controller is in charge."""
    diagram, plant = build_cartpole(controller=controller)
    context = diagram.CreateDefaultContext()
    plant_context = plant.GetMyContextFromRoot(context)
    controller_context = controller.GetMyContextFromRoot(context)
    set_state(plant, plant_context, theta=HANGING)

    simulator = Simulator(diagram, context)
    simulator.Initialize()

    times = np.arange(0.0, duration, SAMPLE_DT)
    inside, sent, demand, cart = [], [], [], []
    for t in times:
        simulator.AdvanceTo(t)
        x = read_state(plant, plant_context)
        inside.append(gate.in_basin(x))
        sent.append(controller.get_output_port(0).Eval(controller_context)[0])
        demand.append(float(-K @ wrapped_error(x)))  # what unclipped LQR would ask for
        cart.append(x[0])
    inside, sent, demand = np.array(inside), np.abs(sent), np.abs(demand)

    switches = times[1:][np.diff(inside.astype(int)) != 0]
    x, theta, _, _ = read_state(plant, plant_context)
    tilt = np.degrees((theta - UPRIGHT + np.pi) % (2 * np.pi) - np.pi)
    ok = abs(tilt) < 1.0 and abs(x) < 1.0
    print(f"\n{label}")
    print(f"  enters/leaves basin at : {np.round(switches, 3).tolist()} s")
    print(f"  peak |u| sent          : {sent[inside].max():6.1f} N   (limit {U_MAX} N)")
    print(f"  unclipped LQR demand   : peak {demand[inside].max():.1f} N, above {U_MAX} N "
          f"for {SAMPLE_DT * np.sum(demand[inside] > U_MAX):.3f} s")
    print(f"  peak |x| after handoff : {np.abs(cart)[np.argmax(inside):].max():6.2f} m")
    print(f"  final                  : x={x:.3f} m  tilt={tilt:.3f} deg  -> "
          + ("balanced" if ok else "FAILED"))
    return sent


def main():
    _state_traj, input_traj = get_swingup()
    K, S = lqr_gain_and_cost_to_go()
    K = np.asarray(K).reshape(4)
    gate = SwingUpAndBalance(input_traj, K, S, RHO)
    duration = input_traj.end_time() + 15.0

    lqr = SwingUpAndBalance(input_traj, K, S, RHO, u_limit=U_MAX)
    lqr_u = trial("swing-up + LQR clipped to 20 N (as in demo_swingup_lqr.py)",
                  lqr, gate, K, duration)

    controller, mpc = mpc_swingup(input_traj, gate)
    mpc_u = trial("swing-up + MPC", controller, gate, K, duration)
    print(f"  MPC latency            : {mpc.latency_report()}")
    print(f"\nlargest |u| difference, LQR vs MPC: {np.abs(lqr_u - mpc_u).max():.2f} N")


if __name__ == "__main__":
    main()
