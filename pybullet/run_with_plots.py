"""Run a drone landing mission with smooth real-time telemetry plots.

Usage:
    python run_with_plots.py                   # interactive – defaults to address 1
    python run_with_plots.py 5                 # fly to address 5 (moving service rover)
    python run_with_plots.py 3 5 8             # sequential missions to addresses 3, 5, 8
    python run_with_plots.py --headless 1      # no PyBullet GUI, just plots + CSV
    python run_with_plots.py --no-camera 5     # skip onboard camera window

Data is saved to results/mission_<address>_telemetry.csv after each mission.
"""
from __future__ import annotations
import argparse
import time
import sys
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

from config import ADDRESSES
from simulation import LandingSimulation
from live_plotter import LivePlotter


def run_mission(address: int, *, gui: bool = True, show_camera: bool = True,
                max_steps: int = 18000, seed: int = 4):
    """Fly one mission with live plots, save CSV when done."""
    print(f"\n{'='*60}")
    print(f"  MISSION → Address {address}: {ADDRESSES[address].name}")
    print(f"  Type: {ADDRESSES[address].kind.upper()}  |  Motion: {ADDRESSES[address].motion}")
    print(f"{'='*60}\n")

    sim = LandingSimulation(gui=gui, seed=seed)
    plotter = LivePlotter(window_title=f"Telemetry – Addr {address}: {ADDRESSES[address].name}")

    sim.select(address)
    start_wall = time.perf_counter()

    try:
        for i in range(max_steps):
            snap = sim.step()
            pos = snap["position"]
            vel = snap["velocity"]

            # Compute horizontal error to the current pad position
            pad_pos = sim.world.target_position(address, snap["time"])
            horiz_err = float(np.linalg.norm(pos[:2] - pad_pos[:2]))

            # Feed live plotter (every control step is fine; plotter
            # internally decimates the visual refresh rate)
            plotter.update(snap["time"], pos, vel, horiz_err, snap["phase"])

            # Console print every ~2.5 s of sim time
            if i % 300 == 0:
                elapsed = time.perf_counter() - start_wall
                spd = float(np.linalg.norm(vel))
                print(f"  t={snap['time']:6.1f}s  {snap['phase']:14s}  "
                      f"pos=[{pos[0]:+6.2f},{pos[1]:+6.2f},{pos[2]:5.2f}]  "
                      f"err={horiz_err:5.2f}m  spd={spd:4.2f}m/s  "
                      f"[wall {elapsed:.1f}s]")

            # Show onboard camera feed if available
            if show_camera and snap["frame"] is not None:
                frame = snap["frame"].copy()
                for d in snap["detections"]:
                    match = address in (d.tag_id, d.tag_id - 10)
                    color = (75, 220, 105) if match else (40, 160, 240)
                    cv2.polylines(frame, [d.corners.astype("int32")], True, color, 2)
                    cv2.putText(frame, f"TAG {d.tag_id}",
                                tuple(np.asarray(d.center_px, dtype="int32")),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
                cv2.imshow("Onboard Camera", frame)
                cv2.waitKey(1)

            # End on landing or abort
            if snap["phase"] in ("LANDED", "ABORT"):
                status = "LANDED SUCCESSFULLY" if snap["phase"] == "LANDED" else "ABORTED"
                print(f"\n  ✓ {status} at t={snap['time']:.1f}s\n")
                break

            # Small sleep to keep real-time pacing when GUI is on
            if gui:
                time.sleep(1 / 240)

        # ── Save CSV ─────────────────────────────────────────────────
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_path = Path("results") / f"mission_{address}_{stamp}_telemetry.csv"
        sim.save_csv(str(csv_path))
        print(f"  CSV saved → {csv_path.resolve()}")

        # Keep plots visible until user closes the window
        plotter.finish()
        print("  Close the plot window to continue...")
        import matplotlib.pyplot as plt
        plt.show()

    finally:
        sim.close()
        plotter.close()
        cv2.destroyAllWindows()


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("addresses", nargs="*", type=int, default=[1],
                    help="Landing-pad addresses 1-10 (default: 1)")
    ap.add_argument("--headless", action="store_true",
                    help="Run PyBullet without 3-D GUI window")
    ap.add_argument("--no-camera", action="store_true",
                    help="Suppress onboard camera OpenCV window")
    ap.add_argument("--steps", type=int, default=18000,
                    help="Max simulation steps per mission (default 18000)")
    ap.add_argument("--seed", type=int, default=4,
                    help="Random seed (default 4)")
    args = ap.parse_args()

    for addr in args.addresses:
        if addr not in ADDRESSES:
            print(f"ERROR: address {addr} not in 1..10"); sys.exit(1)

    for addr in args.addresses:
        run_mission(addr, gui=not args.headless,
                    show_camera=not args.no_camera,
                    max_steps=args.steps, seed=args.seed)

    print("\nAll missions complete.")


if __name__ == "__main__":
    main()
