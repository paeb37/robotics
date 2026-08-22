# Cart-Pole Control

Trajectory optimization and constrained model-predictive control for an underactuated
cart-pole, built on [Drake](https://drake.mit.edu).

> **Status: in progress.** This README currently documents setup only. The full write-up
> (system, approach, results, what I'd improve) lands once the controllers are working.

---

## Setup

### Requirements

| | |
|---|---|
| OS | macOS 15+ (arm64) or Ubuntu 24.04 / 26.04 |
| Python | 3.13 or 3.14 |
| Drake | 1.56.0 (pinned) |

On macOS you must use **Homebrew Python**, not Apple's system Python at `/usr/bin/python3` —
Drake does not support it. Install with `brew install python@3.14` if needed.

Drake's supported configurations are listed at
[drake.mit.edu/installation.html](https://drake.mit.edu/installation.html).

### Install

```bash
git clone https://github.com/brandon-pae/robotics.git
cd robotics

python3 -m venv .venv
source .venv/bin/activate

python -m pip install --upgrade pip
pip install -e ".[dev]"
```

`pip install -e ".[dev]"` installs this project in editable mode along with its pinned
dependencies (Drake 1.56.0) and the development tools (pytest, ruff). Editable mode means
source edits take effect without reinstalling.

### Verify

```bash
python -c "
import pydrake.all
from pydrake.solvers import OsqpSolver, SnoptSolver, IpoptSolver
print('pydrake.all import: OK')
print('OSQP  available:', OsqpSolver().available())
print('SNOPT available:', SnoptSolver().available())
print('IPOPT available:', IpoptSolver().available())
"
```

Expected output:

```
pydrake.all import: OK
OSQP  available: True
SNOPT available: True
IPOPT available: True
```

This checks more than that Drake imported. `pydrake.all` pulls in the full solver and
geometry stack, so it catches a partially-installed wheel that a bare `import pydrake`
would pass. The three solvers are each load-bearing here: **OSQP** solves the quadratic
program at every MPC step, and **SNOPT/IPOPT** solve the nonlinear program behind the
direct-collocation swing-up.

To check the installed Drake version, use `pip show drake` — `pydrake` does not expose a
`__version__` attribute.

### Visualization

Drake renders through [MeshCat](https://drake.mit.edu/doxygen_cxx/classdrake_1_1geometry_1_1_meshcat.html),
which serves a 3D view over a local HTTP URL. Scripts that visualize print that URL on
startup; open it in a browser. The server lives as long as the process holding it, so run
those scripts in a terminal you control rather than a transient shell.

---

## Repository layout

```
cartpole/        control and simulation code
tests/           pytest suite
notes/           NOTES.md — running technical notes
```

---

## License

MIT
