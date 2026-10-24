"""Measure isolated 1 m position steps without camera-rendering overhead."""
from __future__ import annotations

import csv
from datetime import datetime, timezone
from pathlib import Path
import sys

import matplotlib.pyplot as plt
import numpy as np
import pybullet as p

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from config import DT
from simulation import LandingSimulation


def measure(axis: int, seed: int = 100, step_time: float = 1.0, duration: float = 10.0):
    sim = LandingSimulation(gui=False, seed=seed)
    rows = []
    try:
        start = sim.drone.state()[0]
        target = start.copy()
        final_target = start.copy()
        final_target[axis] += 1.0
        for i in range(round(duration / DT)):
            now = i * DT
            command = start if now < step_time else final_target
            sim.drone.control(command)
            p.stepSimulation(physicsClientId=sim.client)
            pos, _, vel, _ = sim.drone.state()
            rows.append((now, float(pos[axis]), float(vel[axis])))
        return rows, float(start[axis]), float(final_target[axis]), step_time
    finally:
        sim.close()


def metrics(rows, initial, final, step_time):
    data = np.asarray(rows, dtype=float)
    after = data[data[:, 0] >= step_time]
    response = (after[:, 1] - initial) / (final - initial)
    crossings_10 = after[response >= .1, 0]
    crossings_90 = after[response >= .9, 0]
    rise = float(crossings_90[0] - crossings_10[0]) if len(crossings_10) and len(crossings_90) else None
    overshoot = max(0.0, float((np.max(after[:, 1]) - final) / abs(final - initial) * 100.0))
    band = .02 * abs(final - initial)
    settling = None
    for i, row in enumerate(after):
        if np.all(np.abs(after[i:, 1] - final) <= band):
            settling = float(row[0] - step_time)
            break
    return {"rise_time_10_90_s": rise, "overshoot_pct": overshoot,
            "settling_time_2pct_s": settling,
            "final_error_m": float(abs(data[-1, 1] - final)),
            "peak_abs_velocity_m_s": float(np.max(np.abs(data[:, 2])))}


def main():
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    outdir = Path(__file__).resolve().parents[1] / "results" / "dynamics"
    outdir.mkdir(parents=True, exist_ok=True)
    all_rows = []
    summaries = []
    fig, axes = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
    for axis, name in ((0, "x"), (2, "z")):
        rows, initial, final, step_time = measure(axis)
        all_rows.extend((name, *row) for row in rows)
        summary = {"axis": name, **metrics(rows, initial, final, step_time)}
        summaries.append(summary)
        axes[0 if name == "x" else 1].plot([r[0] for r in rows], [r[1] for r in rows], label=name)
        ax = axes[0 if name == "x" else 1]
        ax.axvline(step_time, color="gray", linestyle="--", linewidth=1)
        ax.axhline(final, color="black", linestyle=":", linewidth=1)
        ax.set_ylabel(f"{name} position (m)")
        ax.grid(alpha=.25)
        ax.set_title(f"{name.upper()} 1 m step — rise {summary['rise_time_10_90_s']} s; "
                     f"overshoot {summary['overshoot_pct']:.1f}%; settle {summary['settling_time_2pct_s']} s")
    axes[-1].set_xlabel("Simulation time (s)")
    fig.tight_layout()
    image_path = outdir / f"step_response_{stamp}.png"
    csv_path = outdir / f"step_response_{stamp}.csv"
    fig.savefig(image_path, dpi=150)
    plt.close(fig)
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(("axis", "time_s", "position_m", "velocity_m_s"))
        writer.writerows(all_rows)
    print({"csv": str(csv_path), "plot": str(image_path), "metrics": summaries})


if __name__ == "__main__":
    main()
