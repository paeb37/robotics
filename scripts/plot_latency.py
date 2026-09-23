"""Figure: per-step MPC cost, Python vs C++, against the 10 ms control period.

    python scripts/plot_latency.py          # writes docs/media/latency.png

The benchmark table reports percentiles; this shows the shape behind them -- a
C++ backend that is ~10x faster on a typical step, with rare spikes at
active-set transitions that land close to the deadline.
"""

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from bench_mpc import DEADLINE_MS, WARMUP, run

from cartpole.mpc import CONTROL_DT

KICK, X_MAX = 1.0, 4.0  # the bench_mpc.py case with the largest C++ spike
OUTPUT = Path(__file__).resolve().parents[1] / "docs" / "media" / "latency.png"


def main():
    fig, ax = plt.subplots(figsize=(8, 3.6))
    ax.axhline(DEADLINE_MS, color="#d62728", ls="--", lw=1.2)
    ax.text(0.2, DEADLINE_MS * 1.1, f"deadline ({DEADLINE_MS:.0f} ms)", fontsize=9,
            color="#d62728")

    typical = {}
    for name, color in [("python", "#ff7f0e"), ("cpp", "#1f77b4")]:
        _, times = run(KICK, X_MAX, backend=name)
        t = (WARMUP + np.arange(len(times))) * CONTROL_DT
        label = "Python" if name == "python" else "C++"
        ax.scatter(t, times, s=6, color=color, label=label)
        typical[label] = np.median(times)
        print(f"{label:<6} p50={np.median(times):.3f} ms  max={times.max():.3f} ms  "
              f"misses={int((times > DEADLINE_MS).sum())}/{len(times)}")
        if name == "cpp":
            i = int(np.argmax(times))
            ax.annotate(f"{times[i]:.1f} ms", (t[i], times[i]), xytext=(8, -4),
                        textcoords="offset points", fontsize=9)

    ax.set_yscale("log")
    ax.set_ylim(0.05, 20)
    ax.set_yticks([0.1, 1, 10], ["0.1", "1", "10"])
    ax.set_xlabel("time (s)")
    ax.set_ylabel("time per step (ms)")
    speedup = typical["Python"] / typical["C++"]
    ax.set_title(f"{speedup:.0f}× faster on a typical step. Almost missed the deadline.")
    ax.legend(loc="upper right", bbox_to_anchor=(1, 0.72), ncol=2, fontsize=9, markerscale=2)
    ax.grid(alpha=0.25)
    fig.tight_layout()

    fig.savefig(OUTPUT, dpi=140, bbox_inches="tight")
    print(f"wrote {OUTPUT}")


if __name__ == "__main__":
    main()
