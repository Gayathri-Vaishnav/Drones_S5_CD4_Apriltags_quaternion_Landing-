"""Address selection, divert handling, descent interlocks and touchdown."""
from enum import Enum
import numpy as np
from config import ADDRESSES, ALTITUDE_SWITCH_M, TOUCHDOWN_HEIGHT_M

class Phase(str,Enum):
    IDLE="IDLE"; TAKEOFF="TAKEOFF"; SEARCH="SEARCH"; APPROACH="APPROACH"
    DESCEND_HIGH="DESCEND_HIGH"; DESCEND_LOW="DESCEND_LOW"; FLARE="FLARE"
    TOUCHDOWN="TOUCHDOWN"; LANDED="LANDED"; ABORT="ABORT"

class Mission:
    def __init__(self): self.address=None; self.phase=Phase.IDLE; self.stable=0; self.start_time=0; self.last_lock_time=None
    def select(self, number, now):
        if number not in ADDRESSES: raise ValueError("Address must be 1..10")
        self.address=number; self.phase=Phase.TAKEOFF; self.start_time=now; self.stable=0; self.last_lock_time=None
    def update(self, now, pos, target, detected_id=None, target_velocity=None, drone_velocity=None, contact=False):
        if self.address is None: return self.phase
        if now-self.start_time>180: self.phase=Phase.ABORT; return self.phase
        pos=np.asarray(pos); target=np.asarray(target); alt=pos[2]-target[2]
        horizontal=float(np.linalg.norm(pos[:2]-target[:2]))
        id_ok=detected_id in (self.address,self.address+10)
        moving=ADDRESSES[self.address].kind=="vehicle"
        if id_ok: self.last_lock_time=now
        landing_lock=id_ok or (moving and self.last_lock_time is not None and now-self.last_lock_time<.20) or not moving
        target_velocity=np.asarray(target_velocity if target_velocity is not None else (0,0,0),dtype=float)
        drone_velocity=np.asarray(drone_velocity if drone_velocity is not None else (0,0,0),dtype=float)
        relative_speed=float(np.linalg.norm((drone_velocity-target_velocity)[:2]))
        approach_speed_ok=not moving or relative_speed<.75
        descent_speed_ok=not moving or relative_speed<.55
        # For moving targets, losing visual lock during descent triggers a safe climb.
        if moving and not landing_lock and self.phase in (Phase.DESCEND_HIGH,Phase.DESCEND_LOW,Phase.FLARE):
            self.phase=Phase.APPROACH; self.stable=0
        if self.phase==Phase.TAKEOFF and alt>3.6: self.phase=Phase.SEARCH
        elif self.phase==Phase.SEARCH and (id_ok or horizontal<3.5): self.phase=Phase.APPROACH
        elif self.phase==Phase.APPROACH and horizontal<.75 and alt<5.6 and landing_lock and approach_speed_ok: self.phase=Phase.DESCEND_HIGH
        elif self.phase==Phase.DESCEND_HIGH and horizontal<.5 and alt<ALTITUDE_SWITCH_M+.35 and landing_lock and descent_speed_ok: self.phase=Phase.DESCEND_LOW
        elif self.phase==Phase.DESCEND_LOW and horizontal<.32 and alt<.8 and landing_lock and descent_speed_ok: self.phase=Phase.FLARE
        # Keep the aircraft above the tag until it is low, locked, and centered.
        elif self.phase==Phase.FLARE and alt<.3 and horizontal<.25 and landing_lock and relative_speed<.35: self.phase=Phase.TOUCHDOWN
        elif self.phase==Phase.TOUCHDOWN:
            speed = np.linalg.norm(np.asarray(target_velocity if target_velocity is not None else (0,0,0)))
            # Require measured pad contact; a transient image lock alone is not touchdown.
            if contact and horizontal<.20 and 0<=alt<TOUCHDOWN_HEIGHT_M+.10 and speed<.8 and relative_speed<.25: self.stable+=1
            else: self.stable=0
            if self.stable>=45: self.phase=Phase.LANDED
        return self.phase
