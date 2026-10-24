"""Quick headless smoke test to verify simulation + CSV saving."""
from simulation import LandingSimulation
from datetime import datetime, timezone
import numpy as np
from pathlib import Path

sim = LandingSimulation(gui=False, seed=4)
sim.select(1)
print("Running 600 steps headless...")
for i in range(600):
    snap = sim.step()
    if i % 200 == 0:
        pos = snap["position"]
        vel = snap["velocity"]
        spd = np.linalg.norm(vel)
        phase = snap["phase"]
        print(f"  t={snap['time']:5.1f}s  phase={phase:14s}  z={pos[2]:.2f}m  speed={spd:.2f}m/s")

stamp=datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
output=Path("results")/"smoke_tests"/f"smoke_{stamp}.csv"
sim.save_csv(output)
sim.close()

import pandas as pd
df = pd.read_csv(output)
print(f"\nCSV saved: {output} ({len(df)} rows)")
print(f"Columns: {list(df.columns)}")
has_vel = len(df)>0 and all(c in df.columns for c in ["vel_x_m_s","vel_y_m_s","vel_z_m_s","speed_m_s"])
print(f"Velocity columns present: {has_vel}")
print(f"\nSample (last row):")
print(df.iloc[-1].to_string())
print("\nSmoke test PASSED" if has_vel else "\nFAILED - missing velocity columns")
if not has_vel:
    raise SystemExit(1)
