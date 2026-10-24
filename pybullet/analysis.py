"""Small linear controllability/observability and quaternion interpolation study."""
import numpy as np

def analyze():
    # Discrete double integrator in one axis, extended to 3D by Kronecker products.
    dt=.02; a=np.array([[1.,dt],[0.,1.]]); b=np.array([[.5*dt*dt],[dt]]); c=np.array([[1.,0.]])
    ctr=np.hstack((b,a@b)); obs=np.vstack((c,c@a))
    print(f"1D double-integrator controllability rank: {np.linalg.matrix_rank(ctr)}/2")
    print(f"1D double-integrator observability rank: {np.linalg.matrix_rank(obs)}/2")
    q0=np.array([0.,0.,0.,1.]); q1=np.array([0.,0.,1.,0.])
    lerp=(q0+q1)/2; lerp/=np.linalg.norm(lerp)
    dot=np.dot(q0,q1); theta=np.arccos(dot)
    slerp=np.sin(theta/2)/np.sin(theta)*q0+np.sin(theta/2)/np.sin(theta)*q1
    print("Normalized midpoint LERP:",np.round(lerp,4))
    print("SLERP midpoint:            ",np.round(slerp,4))
    print("SLERP keeps constant angular speed; normalized LERP does not.")

if __name__=="__main__": analyze()
