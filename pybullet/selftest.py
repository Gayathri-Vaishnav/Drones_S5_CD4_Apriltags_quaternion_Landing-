"""Environment health check; run with `run.bat check`."""
import importlib
mods=("pybullet","numpy","cv2","pupil_apriltags","matplotlib")
for name in mods:
    module=importlib.import_module(name)
    print(name,getattr(module,"__version__","available"))
from world import tag_texture
from pupil_apriltags import Detector
import cv2
marker=tag_texture(1)
image=cv2.imread(marker,cv2.IMREAD_GRAYSCALE)
found=[int(item.tag_id) for item in Detector(families="tag36h11",nthreads=1).detect(image)]
if 1 not in found: raise SystemExit("AprilTag self-check failed: generated tag 1 did not decode")
print("AprilTag decode: id 1 OK")
from simulation import LandingSimulation
import pybullet as p
sim=LandingSimulation(gui=False)
try:
    sim.select(1)
    p.resetBasePositionAndOrientation(sim.drone.body,(-8.,-1.,5.),(0.,0.,0.,1.),physicsClientId=sim.client)
    position,orientation,_,_=sim.drone.state()
    frame,_=sim.camera.render(position,orientation)
    detections=sim.camera.detect(frame,position,orientation)
    if not any(d.tag_id==1 for d in detections): raise SystemExit("Rendered-scene self-check failed: address 1 tag was not detected")
    print(f"PyBullet world, rendered camera, and PnP: OK ({frame.shape[1]}x{frame.shape[0]}; tag 1 detected)")
    snap=sim.step(12)
    if not all(__import__("numpy").isfinite(snap["position"])): raise SystemExit("Rigid-body controller self-check failed: non-finite state")
    print("Closed-loop rigid-body step: OK")
finally:
    sim.close()
print("Ready to open the dashboard with run.bat")
