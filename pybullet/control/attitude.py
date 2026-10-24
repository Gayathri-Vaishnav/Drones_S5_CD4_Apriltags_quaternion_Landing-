"""Acceleration-to-tilt mapping and quaternion attitude error."""
import math
import numpy as np
import pybullet as p
from config import GRAVITY, MAX_TILT_DEG

def acceleration_to_quaternion(acceleration,yaw):
    ax,ay,_=np.asarray(acceleration,dtype=float)
    limit=math.radians(MAX_TILT_DEG)
    roll=float(np.clip(-math.atan2(ay,GRAVITY),-limit,limit))
    pitch=float(np.clip(math.atan2(ax,GRAVITY),-limit,limit))
    return p.getQuaternionFromEuler((roll,pitch,float(yaw)))

def quaternion_error(actual,target):
    q=p.getDifferenceQuaternion(actual,target)
    axis=np.asarray(q[:3],dtype=float); norm=float(np.linalg.norm(axis))
    if norm<1e-8: return np.zeros(3)
    angle=2*math.atan2(norm,max(float(q[3]),1e-8))
    return axis/norm*angle
