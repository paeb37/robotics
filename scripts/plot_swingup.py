"""Figure: pole angle over the planned swing-up.

    python scripts/plot_swingup.py          # writes docs/media/swingup_plan.png

Neither swing-up clip shows the whole plan -- feedback takes over partway, and
open-loop falls before the end -- so this is the only view of the pumping.
"""

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from cartpole.swingup import U_MAX, get_swingup

OUTPUT = Path(__file__).resolve().parents[1] / "docs" / "media" / "swingup_plan.png"


def main():
    state_traj, input_traj = get_swingup()
    t = np.linspace(0.0, state_traj.end_time(), 2000)
    angle = np.degrees([state_traj.value(s)[1, 0] for s in t])
    force = np.array([input_traj.value(s)[0, 0] for s in t])

    # Swing peaks: where the pole reverses before it first reaches upright.
    peaks = [i for i in np.flatnonzero(np.diff(np.sign(np.diff(angle))) != 0) + 1
             if abs(angle[i]) < 170]

    fig, ax = plt.subplots(figsize=(8, 3.6))
    ax.axhline(180, color="0.5", ls="--", lw=1.2)
    ax.text(t[-1], 184, "upright", ha="right", va="bottom", fontsize=9, color="0.4")
    ax.axhline(0, color="0.8", lw=0.8)
    ax.plot(t, angle, color="#1f77b4", lw=2)
    for i in peaks:
        ax.annotate(f"{abs(angle[i]):.0f}°", (t[i], angle[i]), fontsize=9,
                    xytext=(0, 8 if angle[i] > 0 else -14), textcoords="offset points",
                    ha="center")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("pole angle from hanging (deg)")
    ax.set_ylim(-150, 205)
    ax.set_title(f"{len(peaks)} swings, each bigger than the last. "
                 f"Never more than {U_MAX:.0f} N.")
    ax.grid(alpha=0.25)
    fig.tight_layout()

    fig.savefig(OUTPUT, dpi=140, bbox_inches="tight")
    print(f"wrote {OUTPUT}")
    print(f"  swings = {len(peaks)}   peak |u| = {abs(force).max():.2f} N   "
          f"plan = {t[-1]:.2f} s")


if __name__ == "__main__":
    main()
