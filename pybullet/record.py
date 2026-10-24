"""Record one rendered mission as an animated GIF (world view and camera view)."""
import argparse
from pathlib import Path
from PIL import Image,ImageDraw
import cv2
from simulation import LandingSimulation

def record(address, seconds=40, output=None):
    sim=LandingSimulation(gui=False)
    sim.select(address); frames=[]
    try:
        for i in range(int(seconds*120)):
            snap=sim.step()
            if i%6==0:
                world=sim.render_world(640,400)
                camera=sim.frame if sim.frame is not None else world
                camera=cv2.resize(camera,(320,240))
                strip=Image.new("RGB",(960,440),(18,24,32))
                strip.paste(Image.fromarray(cv2.cvtColor(world,cv2.COLOR_BGR2RGB)),(0,24))
                strip.paste(Image.fromarray(cv2.cvtColor(camera,cv2.COLOR_BGR2RGB)),(640,100))
                ImageDraw.Draw(strip).text((12,5),f"Address {address} | {snap['phase']} | t={snap['time']:.1f}s",fill="white")
                frames.append(strip)
            if snap["phase"] in ("LANDED","ABORT"): break
        dest=Path(output or f"results/mission_{address}.gif"); dest.parent.mkdir(parents=True,exist_ok=True)
        if frames: frames[0].save(dest,save_all=True,append_images=frames[1:],duration=50,loop=0)
        sim.save_csv(dest.with_suffix(".csv"))
        print(f"Saved {dest} ({len(frames)} frames, final phase {sim.mission.phase.value})")
    finally: sim.close()

if __name__=="__main__":
    ap=argparse.ArgumentParser(); ap.add_argument("address",type=int); ap.add_argument("--seconds",type=float,default=40); ap.add_argument("--output")
    a=ap.parse_args(); record(a.address,a.seconds,a.output)
