"""Altitude-aware landing gates and velocity-aware pad guidance."""
import numpy as np
from config import APPROACH_ALTITUDE_M, ALTITUDE_SWITCH_M, TOUCHDOWN_HEIGHT_M

class Guidance:
    def __init__(self, altitude_schedule=True): self.altitude_schedule=altitude_schedule
    def waypoint(self, current, target, target_velocity, phase, *, moving=False):
        target=np.asarray(target,dtype=float); current=np.asarray(current,dtype=float)
        target_velocity=np.asarray(target_velocity,dtype=float)
        if moving and phase in ("APPROACH","DESCEND_HIGH"):
            distance=float(np.linalg.norm(target[:2]-current[:2]))
            intercept_horizon=float(np.clip(distance/2.0,.20,1.25))
            target=target+target_velocity*intercept_horizon
        height=target[2]
        if phase in ("TAKEOFF","SEARCH","APPROACH"):
            return np.array((target[0],target[1],height+APPROACH_ALTITUDE_M))
        if phase=="DESCEND_HIGH": return np.array((target[0],target[1],height+(ALTITUDE_SWITCH_M if self.altitude_schedule else .65)))
        if phase=="DESCEND_LOW": return np.array((target[0],target[1],height+.65))
        if phase in ("FLARE","TOUCHDOWN"): return np.array((target[0],target[1],height+TOUCHDOWN_HEIGHT_M))
        return target

