"""Common PyBullet simulation loop used by the dashboard, recorder and evaluator."""
from __future__ import annotations
import csv, json, math, platform, subprocess, time
from dataclasses import asdict
from datetime import datetime, timezone
from importlib import metadata
from pathlib import Path
import numpy as np
import pybullet as p
import config
from config import DT, CTRL_PERIOD, CAMERA_PERIOD, ADDRESSES, APPROACH_ALTITUDE_M
from world import UrbanWorld
from physics import Quadrotor
from mission import Mission, Phase
from perception.vision import OnboardCamera
from control.tracker import KalmanTargetTracker
from control.guidance import Guidance
from perception.beacon import approximate_fix

TELEMETRY_FIELDS=("time_s","address","phase","x_m","y_m","z_m",
                  "pad_x_m","pad_y_m","pad_z_m","horizontal_error_m","tag_id","tag_pose_error_m",
                  "vel_x_m_s","vel_y_m_s","vel_z_m_s","speed_m_s",
                  "roll_deg","pitch_deg","yaw_deg",
                  "angular_vel_x_rad_s","angular_vel_y_rad_s","angular_vel_z_rad_s",
                  "thrust_n","command_x_m","command_y_m","command_z_m",
                  "target_vel_x_m_s","target_vel_y_m_s","target_vel_z_m_s","pad_contact",
                  "tracker_innovation_x_m","tracker_innovation_y_m","tracker_innovation_z_m",
                  "tracker_mahalanobis","tracker_update_accepted","tracker_position_std_m",
                  "tracker_measurement_age_s")

class LandingSimulation:
    def __init__(self, gui=False, seed=4, start_position=None):
        self.client=p.connect(p.GUI if gui else p.DIRECT)
        if self.client < 0:
            raise RuntimeError("PyBullet could not create the requested physics client")
        p.resetSimulation(physicsClientId=self.client)
        p.setRealTimeSimulation(0,physicsClientId=self.client)
        p.setTimeStep(DT,physicsClientId=self.client)
        p.setPhysicsEngineParameter(numSolverIterations=config.SOLVER_ITERATIONS,
                                    numSubSteps=config.NUM_SUBSTEPS,
                                    deterministicOverlappingPairs=1,
                                    physicsClientId=self.client)
        self.seed=int(seed)
        self.gui=bool(gui)
        self.started_utc=datetime.now(timezone.utc).isoformat()
        self.rng=np.random.default_rng(seed)
        self.world=UrbanWorld(self.client)
        drone_start=(0,-7,.35) if start_position is None else tuple(map(float,start_position))
        self.drone=Quadrotor(self.client,drone_start,rng=self.rng)
        self.camera=OnboardCamera(self.client)
        self.mission=Mission(); self.tracker=KalmanTargetTracker(); self.guidance=Guidance()
        self.time=0.; self._camera_clock=0.; self._ctrl_clock=0.
        self.frame=None; self.detections=[]; self.position_estimate=None; self.velocity_estimate=np.zeros(3)
        self._last_tag_time=None
        self.beacon=None; self.rows=[]; self.last_phase=Phase.IDLE
        self.command_target=None; self.command_velocity=np.zeros(3)
        self.manifest_path=None
        self.scenario=None
        self.timing_samples={"camera_render_s":[],"tag_detection_s":[],"controller_s":[],"physics_step_s":[]}
        if gui:
            p.resetDebugVisualizerCamera(25,45,-35,(2,7,1),physicsClientId=self.client)

    def select(self, number):
        self.mission.select(int(number),self.time)
        self.world.set_active(int(number))
        self.tracker=KalmanTargetTracker()
        # Beacon fix is an approximate initial waypoint; camera pose takes over after decode.
        self.beacon=approximate_fix(int(number),self.rng).position
        self.command_target=self.beacon+np.array((0.,0.,APPROACH_ALTITUDE_M))
        self.command_velocity=np.zeros(3)
        self.scenario={"address":int(number),"target_code":ADDRESSES[int(number)].code,
                       "target_name":ADDRESSES[int(number)].name,"target_type":ADDRESSES[int(number)].kind}
        stamp=datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
        manifest_dir=Path(__file__).resolve().parent/"results"/"runs"
        self.manifest_path=manifest_dir/f"run_{stamp}_a{int(number):02d}_seed{self.seed}.manifest.json"
        self.write_manifest(self.manifest_path)

    @staticmethod
    def _git_revision():
        try:
            return subprocess.run(["git","rev-parse","HEAD"],cwd=Path(__file__).resolve().parent,
                                  check=True,capture_output=True,text=True,timeout=2).stdout.strip()
        except (OSError,subprocess.SubprocessError):
            return None

    @staticmethod
    def _package_version(distribution):
        try:
            return metadata.version(distribution)
        except metadata.PackageNotFoundError:
            return None

    def manifest(self, scenario=None):
        config_snapshot={key:value for key,value in vars(config).items()
                         if key.isupper() and key not in ("ADDRESSES",)}
        config_snapshot["ADDRESSES"]={str(key):asdict(value) for key,value in ADDRESSES.items()}
        return {"schema_version":1,"created_utc":self.started_utc,
                "git_revision":self._git_revision(),"seed":self.seed,
                "scenario":scenario if scenario is not None else self.scenario,
                "physics":{"mode":"GUI" if self.gui else "DIRECT","time_step_s":DT,
                           "solver_iterations":config.SOLVER_ITERATIONS,
                           "substeps":config.NUM_SUBSTEPS,"real_time":False},
                "renderer":{"onboard":"BULLET_HARDWARE_OPENGL" if self.gui else "TINY_RENDERER"},
                "config":config_snapshot,
                "runtime":{"python":platform.python_version(),"platform":platform.platform(),
                           "packages":{"pybullet":self._package_version("pybullet") or self._package_version("pybullet-arm64"),
                                       "numpy":self._package_version("numpy"),
                                       "opencv-contrib-python":self._package_version("opencv-contrib-python"),
                                       "pupil-apriltags":self._package_version("pupil-apriltags"),
                                       "matplotlib":self._package_version("matplotlib"),
                                       "pandas":self._package_version("pandas"),
                                       "pillow":self._package_version("pillow")}}}

    def write_manifest(self, path, scenario=None):
        path=Path(path)
        path.parent.mkdir(parents=True,exist_ok=True)
        path.write_text(json.dumps(self.manifest(scenario),indent=2,default=str),encoding="utf-8")
        return path

    def step(self, count=1, capture=True):
        for _ in range(count):
            self.time += DT
            self.world.update(self.time)
            pos,orn,vel,omega=self.drone.state()
            if self.mission.address is not None and self.time-self._camera_clock >= CAMERA_PERIOD:
                camera_dt=max(self.time-self._camera_clock,CAMERA_PERIOD)
                self._camera_clock=self.time
                timer=time.perf_counter()
                self.frame,_=self.camera.render(pos,orn)
                self.timing_samples["camera_render_s"].append(time.perf_counter()-timer)
                timer=time.perf_counter()
                self.detections=self.camera.detect(self.frame,pos,orn)
                self.timing_samples["tag_detection_s"].append(time.perf_counter()-timer)
                number=self.mission.address
                matching=next((d for d in self.detections if d.tag_id==number+10),None)
                if matching is None:
                    matching=next((d for d in self.detections if d.tag_id==number),None)
                if matching is not None and matching.reprojection_px<4:
                    self.position_estimate,self.velocity_estimate=self.tracker.update(matching.world_xyz,camera_dt)
                    self._last_tag_time=self.time
            if self.mission.address is not None and self.time-self._ctrl_clock >= CTRL_PERIOD:
                self._ctrl_clock=self.time
                number=self.mission.address
                target=self.world.target_position(number,self.time)
                target_vel=self.world.target_velocity(number,self.time)
                matched=next((d for d in self.detections if d.tag_id==number+10),None)
                if matched is None:
                    matched=next((d for d in self.detections if d.tag_id==number),None)
                estimate=self.beacon
                observed_velocity=np.zeros(3)
                if self.position_estimate is not None and self._last_tag_time is not None:
                    age=max(0.,self.time-self._last_tag_time)
                    estimate=self.position_estimate+self.velocity_estimate*min(age,.5)
                    if age<.5: observed_velocity=self.velocity_estimate
                self.drone.surface_height=target[2]
                # Tags guide the approach; physical contact confirms touchdown.
                contact=bool(p.getContactPoints(bodyA=self.drone.body,
                    bodyB=self.world.pad_ids[number],physicsClientId=self.client))
                phase=self.mission.update(self.time,pos,target,matched.tag_id if matched else None,
                                          observed_velocity,vel,contact=contact)
                if phase not in (Phase.LANDED,Phase.ABORT):
                    moving=ADDRESSES[number].kind=="vehicle"
                    self.command_target=self.guidance.waypoint(pos,estimate,observed_velocity,phase.value,moving=moving)
                    self.command_velocity=observed_velocity
                elif phase==Phase.LANDED:
                    self.command_target=np.array((target[0],target[1],target[2]+.16))
                    self.command_velocity=np.zeros(3)
                estimate_error=float(np.linalg.norm(self.position_estimate-target)) if matched is not None and self.position_estimate is not None else -1.0
                # Record simulation data including drone velocity and speed
                vel_est = self.velocity_estimate if self.velocity_estimate is not None else np.zeros(3)
                speed = float(np.linalg.norm(vel))
                euler = p.getEulerFromQuaternion(orn)
                tracker_std=float(np.sqrt(np.trace(self.tracker.covariance[:3,:3]))) if self.tracker.ready else -1.0
                tracker_age=float(self.time-self._last_tag_time) if self._last_tag_time is not None else -1.0
                self.rows.append((self.time, number, phase.value,
                                  *pos, *target,
                                  float(np.linalg.norm(pos[:2]-target[:2])),
                                  matched.tag_id if matched else -1,
                                  estimate_error,
                                  *vel, speed, *np.degrees(euler), *omega,
                                  self.drone.thrust, *self.command_target,
                                  *self.command_velocity,
                                  int(contact),*self.tracker.last_innovation,
                                  self.tracker.last_mahalanobis,int(self.tracker.last_measurement_accepted),
                                  tracker_std,tracker_age))
            if self.mission.address is not None and self.command_target is not None:
                # Bullet clears external forces after each integration step.
                timer=time.perf_counter()
                self.drone.control(self.command_target,self.command_velocity)
                self.timing_samples["controller_s"].append(time.perf_counter()-timer)
            timer=time.perf_counter()
            p.stepSimulation(physicsClientId=self.client)
            self.timing_samples["physics_step_s"].append(time.perf_counter()-timer)
        return self.snapshot()

    def snapshot(self):
        pos,orn,vel,omega=self.drone.state()
        contact=False
        if self.mission.address is not None:
            contact=bool(p.getContactPoints(bodyA=self.drone.body,
                bodyB=self.world.pad_ids[self.mission.address],physicsClientId=self.client))
        return {"time":self.time,"position":pos,"orientation":orn,"velocity":vel,
                "phase":self.mission.phase.value,"target":self.mission.address,
                "frame":self.frame,"detections":self.detections,"contact":contact}

    def render_world(self,width=1280,height=800,*,yaw=18,pitch=-18,distance=25,target=(2.5,7,1.5)):
        view=p.computeViewMatrixFromYawPitchRoll(target,distance,yaw,pitch,0,2)
        proj=p.computeProjectionMatrixFOV(55,width/height,.1,70)
        _,_,rgba,_,_=p.getCameraImage(width,height,view,proj,renderer=p.ER_TINY_RENDERER,physicsClientId=self.client)
        import cv2
        return cv2.cvtColor(np.asarray(rgba,dtype=np.uint8).reshape(height,width,4)[:,:,:3],cv2.COLOR_RGB2BGR)

    def save_csv(self,path):
        path=Path(path); path.parent.mkdir(parents=True,exist_ok=True)
        with path.open("w",newline="",encoding="utf-8") as f:
            w=csv.writer(f)
            w.writerow(TELEMETRY_FIELDS)
            w.writerows(self.rows)
        if self.manifest_path is not None:
            manifest=self.manifest(self.scenario)
            manifest["outputs"]={"telemetry_csv":str(path.resolve())}
            manifest["timing"] = self.timing_summary()
            self.manifest_path.write_text(json.dumps(manifest,indent=2,default=str),encoding="utf-8")

    def timing_summary(self):
        summary={}
        for name, samples in self.timing_samples.items():
            values=np.asarray(samples,dtype=float)
            if not values.size:
                summary[name]={"samples":0,"mean_s":None,"median_s":None,"p95_s":None,"max_s":None}
                continue
            summary[name]={"samples":int(values.size),"mean_s":float(values.mean()),
                           "median_s":float(np.median(values)),"p95_s":float(np.percentile(values,95)),
                           "max_s":float(values.max())}
        physics=np.asarray(self.timing_samples["physics_step_s"],dtype=float)
        summary["physics_cycle_overruns"]={"count":int(np.count_nonzero(physics>DT)),
                                           "fraction":float(np.mean(physics>DT)) if physics.size else None,
                                           "budget_s":DT}
        return summary

    def close(self):
        if p.isConnected(self.client): p.disconnect(self.client)
