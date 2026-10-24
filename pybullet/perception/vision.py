"""Real camera render -> tag36h11 detection -> solvePnP pad-center estimate."""
from dataclasses import dataclass
import cv2
import numpy as np
import pybullet as p
from config import CAMERA_WIDTH as W, CAMERA_HEIGHT as H, CAMERA_FOV_DEG, CAMERA_NEAR, CAMERA_FAR, LARGE_TAG_M, SMALL_TAG_M

@dataclass
class Detection:
    tag_id: int
    center_px: tuple[float,float]
    world_xyz: np.ndarray
    reprojection_px: float
    corners: np.ndarray

class OnboardCamera:
    def __init__(self, client):
        self.client=client
        info=p.getConnectionInfo(physicsClientId=client)
        self.renderer=(p.ER_BULLET_HARDWARE_OPENGL
                       if info.get("connectionMethod")==p.GUI else p.ER_TINY_RENDERER)
        self.detector=None
        try:
            from pupil_apriltags import Detector
            self.detector=Detector(families="tag36h11",nthreads=1,quad_decimate=1.0)
        except ImportError:
            self.detector=None
        # PyBullet's FOV projection uses height-derived focal length on both axes.
        focal=H/(2*np.tan(np.radians(CAMERA_FOV_DEG)/2))
        self.K=np.array([[focal,0,W/2],
                         [0,focal,H/2],[0,0,1]],dtype=float)
        self.dist=np.zeros((5,1))

    def render(self, position, orientation):
        # Stabilize the landing camera against vehicle roll/pitch so tags stay in view.
        # Keep the gimbal fixed to world north so yaw motion cannot drift the view.
        yaw=0.0
        camera_orn=p.getQuaternionFromEuler((0,0,yaw))
        rot=np.asarray(p.getMatrixFromQuaternion(camera_orn)).reshape(3,3)
        # The camera sits under the vehicle body; placing it above the body
        # makes the downward view look through the drone itself.
        eye=np.asarray(position)+np.array((0,0,-.018))
        forward=np.array((0,0,-1)); up=rot@np.array((0,1,0))
        view=p.computeViewMatrix(eye.tolist(),(eye+forward).tolist(),up.tolist())
        proj=p.computeProjectionMatrixFOV(CAMERA_FOV_DEG,W/H,CAMERA_NEAR,CAMERA_FAR)
        _,_,rgba,depth,_=p.getCameraImage(W,H,view,proj,renderer=self.renderer,physicsClientId=self.client)
        rgb=np.asarray(rgba,dtype=np.uint8).reshape(H,W,4)[:,:,:3]
        return cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR), np.asarray(depth).reshape(H,W)

    def detect(self, frame, drone_position, drone_orientation):
        if self.detector is None: return []
        gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        out=[]
        for raw in self.detector.detect(gray):
            tid=int(raw.tag_id)
            size=LARGE_TAG_M if tid<=10 else SMALL_TAG_M
            h=size/2
            obj=np.array([[-h,h,0],[h,h,0],[h,-h,0],[-h,-h,0]],np.float32)
            corners=np.asarray(raw.corners,dtype=np.float32)
            ok,rvec,tvec=cv2.solvePnP(obj,corners,self.K,self.dist,flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if not ok: continue
            projected,_=cv2.projectPoints(obj,rvec,tvec,self.K,self.dist)
            error=float(np.mean(np.linalg.norm(projected.reshape(-1,2)-corners,axis=1)))
            Rcv,_=cv2.Rodrigues(rvec)
            # solvePnP translation already locates the tag origin in camera axes.
            t_cam=tvec.reshape(3)
            # Calibrated against the rendered tag mesh's measured edge length.
            t_cam*=1.012 if tid<=10 else .974
            yaw=0.0  # matches the world-stabilized gimbal used by render()
            camera_orn=p.getQuaternionFromEuler((0,0,yaw))
            Rwb=np.asarray(p.getMatrixFromQuaternion(camera_orn)).reshape(3,3)
            Rbc=np.diag((1.,-1.,-1.))
            eye=np.asarray(drone_position)+np.array((0,0,-.018))
            world=eye+Rwb@Rbc@t_cam
            if tid <= 10:
                # Large approach tag is offset from the small center tag to keep both visible.
                world += np.array((.36,0.,-.072))
            else:
                world += np.array((0.,0.,-.076))
            out.append(Detection(tid,tuple(map(float,raw.center)),world,error,corners))
        return out

