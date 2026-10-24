"""Run the interactive dashboard or a selected address mission."""
import argparse
import time
import cv2
from config import ADDRESSES
from simulation import LandingSimulation

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument("addresses",nargs="*",type=int,help="address numbers 1-10; omit for interactive dashboard")
    ap.add_argument("--headless",action="store_true")
    ap.add_argument("--paper",action="store_true",help="disable moving-pad feed-forward and altitude gain schedule")
    ap.add_argument("--steps",type=int,default=18000)
    ap.add_argument("--seed",type=int,default=4,help="repeatable NumPy RNG seed")
    args=ap.parse_args()
    if not args.addresses and not args.headless:
        from panel import Dashboard
        Dashboard().run(); return
    sim=LandingSimulation(gui=not args.headless,seed=args.seed)
    try:
        for n in args.addresses or [1]:
            if n not in ADDRESSES: raise SystemExit("Address must be between 1 and 10")
            sim.select(n)
            if args.paper:
                sim.drone.enable_feedforward=False; sim.drone.enable_gain_schedule=False; sim.guidance.altitude_schedule=False
            print(f"Flying to {n}: {ADDRESSES[n].name}")
            for i in range(args.steps):
                snap=sim.step()
                if i%300==0:
                    print(f"t={snap['time']:6.1f}s {snap['phase']:12s} drone={snap['position'].round(2)} detections={len(snap['detections'])}")
                if not args.headless and snap["frame"] is not None:
                    cv2.imshow("Onboard downward camera",snap["frame"])
                    cv2.waitKey(1)
                if snap["phase"] in ("LANDED","ABORT"): break
                if not args.headless: time.sleep(1/120)
            sim.save_csv(f"results/mission_{n}.csv")
    finally:
        sim.close()
        if not args.headless: cv2.destroyAllWindows()

if __name__=="__main__": main()
