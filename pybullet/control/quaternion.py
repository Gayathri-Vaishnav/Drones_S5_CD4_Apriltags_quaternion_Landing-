"""Quaternion utilities used for attitude interpolation and filtering (x,y,z,w)."""
import numpy as np

def normalize(q):
    q=np.asarray(q,dtype=float); norm=np.linalg.norm(q)
    return q/norm if norm>1e-12 else np.array((0.,0.,0.,1.))

def multiply(a,b):
    ax,ay,az,aw=normalize(a); bx,by,bz,bw=normalize(b)
    return np.array((aw*bx+ax*bw+ay*bz-az*by,
                     aw*by-ax*bz+ay*bw+az*bx,
                     aw*bz+az*bw+ax*by-ay*bx,
                     aw*bw-ax*bx-ay*by-az*bz))

def slerp(q0,q1,fraction):
    a=normalize(q0); b=normalize(q1); dot=float(np.dot(a,b))
    if dot<0: b=-b; dot=-dot
    dot=float(np.clip(dot,-1,1)); t=float(np.clip(fraction,0,1))
    if dot>.9995: return normalize(a+t*(b-a))
    theta=np.arccos(dot)
    return normalize(np.sin((1-t)*theta)/np.sin(theta)*a+np.sin(t*theta)/np.sin(theta)*b)

class OrientationFilter:
    """Small complementary orientation filter for noisy quaternion measurements."""
    def __init__(self, gain=.18): self.gain=gain; self.value=None
    def update(self, measurement):
        self.value=normalize(measurement) if self.value is None else slerp(self.value,measurement,self.gain)
        return self.value.copy()
