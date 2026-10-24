"""Generate publication-quality static plots from a saved mission CSV.

Usage:
    python plot_results.py results/mission_5_telemetry.csv
    python plot_results.py results/mission_3_telemetry.csv --save
"""
from __future__ import annotations
import argparse
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt

# ── Dark theme colours ───────────────────────────────────────────────────────
BG       = "#181a1f"
PANEL    = "#20232a"
GRID     = "#2e3138"
CYAN     = "#00e5ff"
MAGENTA  = "#ff4081"
LIME     = "#76ff03"
AMBER    = "#ffc107"
WHITE    = "#e8eaed"
ORANGE   = "#ff9100"
BLUE     = "#448aff"
RED      = "#ff5252"


def load_csv(path: str) -> pd.DataFrame:
    df = pd.read_csv(path)
    return df


def make_plots(df: pd.DataFrame, title_prefix: str = "", save_dir: Path | None = None):
    plt.rcParams.update({
        "figure.facecolor": BG, "axes.facecolor": PANEL,
        "axes.edgecolor": GRID, "axes.labelcolor": WHITE,
        "text.color": WHITE, "xtick.color": WHITE, "ytick.color": WHITE,
        "grid.color": GRID, "grid.alpha": 0.45,
        "font.size": 9, "legend.facecolor": PANEL, "legend.edgecolor": GRID,
    })

    t   = df["time_s"].values
    x   = df["x_m"].values
    y   = df["y_m"].values
    z   = df["z_m"].values
    px  = df["pad_x_m"].values
    py  = df["pad_y_m"].values
    pz  = df["pad_z_m"].values
    herr= df["horizontal_error_m"].values
    vx  = df["vel_x_m_s"].values
    vy  = df["vel_y_m_s"].values
    vz  = df["vel_z_m_s"].values
    spd = df["speed_m_s"].values

    # ── Figure 1 : overview (2×2) ────────────────────────────────────────
    fig1, axes1 = plt.subplots(2, 2, figsize=(13, 9))
    fig1.suptitle(f"{title_prefix}Mission Overview", fontsize=13, fontweight="bold")
    fig1.subplots_adjust(hspace=0.38, wspace=0.28,
                         left=0.07, right=0.97, top=0.92, bottom=0.07)

    # 1a) XY track
    ax = axes1[0, 0]
    ax.plot(x, y, color=CYAN, lw=1.2, label="Drone")
    ax.plot(px, py, "--", color=MAGENTA, lw=1.0, alpha=0.7, label="Pad")
    ax.plot(x[0], y[0], "o", color=LIME, ms=8, label="Start")
    ax.plot(x[-1], y[-1], "s", color=RED, ms=8, label="End")
    ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)")
    ax.set_title("Ground Track"); ax.set_aspect("equal", "datalim")
    ax.legend(fontsize=8); ax.grid(True, lw=0.4)

    # 1b) Altitude profile
    ax = axes1[0, 1]
    ax.plot(t, z, color=LIME, lw=1.2, label="Drone Z")
    ax.plot(t, pz, "--", color=AMBER, lw=1.0, alpha=0.7, label="Pad Z")
    ax.set_xlabel("Time (s)"); ax.set_ylabel("Altitude (m)")
    ax.set_title("Altitude Profile"); ax.legend(fontsize=8); ax.grid(True, lw=0.4)

    # 1c) Horizontal error
    ax = axes1[1, 0]
    ax.plot(t, herr, color=MAGENTA, lw=1.2)
    ax.axhline(0.25, ls=":", color=RED, alpha=0.6, label="25 cm threshold")
    ax.set_xlabel("Time (s)"); ax.set_ylabel("Error (m)")
    ax.set_title("Horizontal Error to Pad"); ax.legend(fontsize=8); ax.grid(True, lw=0.4)

    # 1d) Speed
    ax = axes1[1, 1]
    ax.plot(t, spd, color=ORANGE, lw=1.2)
    ax.set_xlabel("Time (s)"); ax.set_ylabel("Speed (m/s)")
    ax.set_title("Speed Magnitude"); ax.grid(True, lw=0.4)

    if save_dir:
        fig1.savefig(save_dir / "overview.png", dpi=180)

    # ── Figure 2 : velocity components (3×1) ─────────────────────────────
    fig2, axes2 = plt.subplots(3, 1, figsize=(12, 7), sharex=True)
    fig2.suptitle(f"{title_prefix}Velocity Components", fontsize=13, fontweight="bold")
    fig2.subplots_adjust(hspace=0.25, left=0.08, right=0.97, top=0.92, bottom=0.08)

    for ax, data, label, color in zip(
            axes2, [vx, vy, vz], ["Vx", "Vy", "Vz"], [CYAN, AMBER, LIME]):
        ax.plot(t, data, color=color, lw=1.1)
        ax.set_ylabel(f"{label} (m/s)"); ax.grid(True, lw=0.4)
        ax.axhline(0, ls="-", color=GRID, lw=0.6)
    axes2[-1].set_xlabel("Time (s)")

    if save_dir:
        fig2.savefig(save_dir / "velocity_components.png", dpi=180)

    # ── Figure 3 : 3-D trajectory ────────────────────────────────────────
    fig3 = plt.figure(figsize=(9, 7))
    ax3d = fig3.add_subplot(111, projection="3d")
    ax3d.set_facecolor(PANEL); fig3.patch.set_facecolor(BG)
    ax3d.plot(x, y, z, color=CYAN, lw=1.3, label="Drone")
    ax3d.plot(px, py, pz, "--", color=MAGENTA, lw=1.0, alpha=0.6, label="Pad")
    ax3d.scatter([x[0]], [y[0]], [z[0]], color=LIME, s=60, label="Start", depthshade=False)
    ax3d.scatter([x[-1]], [y[-1]], [z[-1]], color=RED, s=60, marker="s", label="End", depthshade=False)
    ax3d.set_xlabel("X (m)"); ax3d.set_ylabel("Y (m)"); ax3d.set_zlabel("Z (m)")
    ax3d.set_title(f"{title_prefix}3-D Flight Trajectory", fontsize=12, fontweight="bold")
    ax3d.legend(fontsize=8)

    if save_dir:
        fig3.savefig(save_dir / "trajectory_3d.png", dpi=180)

    # ── Summary stats ────────────────────────────────────────────────────
    duration = t[-1] - t[0]
    final_err = herr[-1]
    max_speed = spd.max()
    mean_err  = herr.mean()
    final_phase = df["phase"].iloc[-1]

    print(f"\n{'─'*50}")
    print(f"  Mission Summary")
    print(f"{'─'*50}")
    print(f"  Duration        : {duration:.1f} s")
    print(f"  Final phase     : {final_phase}")
    print(f"  Final horiz err : {final_err:.3f} m")
    print(f"  Mean horiz err  : {mean_err:.3f} m")
    print(f"  Max speed       : {max_speed:.2f} m/s")
    print(f"{'─'*50}\n")

    plt.show()


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", help="Path to mission telemetry CSV")
    ap.add_argument("--save", action="store_true",
                    help="Save figures to figures/ directory")
    args = ap.parse_args()

    csv_path = Path(args.csv)
    if not csv_path.exists():
        print(f"File not found: {csv_path}"); sys.exit(1)

    df = load_csv(str(csv_path))
    save_dir = None
    if args.save:
        save_dir = Path("figures")
        save_dir.mkdir(exist_ok=True)
        print(f"Figures will be saved to {save_dir.resolve()}")

    addr = int(df["address"].iloc[0]) if "address" in df.columns else 0
    make_plots(df, title_prefix=f"Addr {addr} – ", save_dir=save_dir)


if __name__ == "__main__":
    main()
