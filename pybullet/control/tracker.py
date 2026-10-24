"""Alpha-beta-gamma constant-acceleration target tracker."""
import numpy as np

class ABGTracker:
    def __init__(self, alpha=.72, beta=.22, gamma=.035):
        self.alpha,self.beta,self.gamma=alpha,beta,gamma
        self.x=np.zeros(3); self.v=np.zeros(3); self.a=np.zeros(3); self.ready=False
    def update(self, measurement, dt):
        z=np.asarray(measurement,dtype=float)
        if not self.ready: self.x=z.copy(); self.v[:]=0; self.a[:]=0; self.ready=True; return self.x.copy(),self.v.copy()
        dt=max(float(dt),1e-3); pred=self.x+self.v*dt+.5*self.a*dt*dt; vp=self.v+self.a*dt
        residual=z-pred
        self.x=pred+self.alpha*residual
        self.v=vp+(self.beta/dt)*residual
        self.a=self.a+(2*self.gamma/(dt*dt))*residual
        return self.x.copy(),self.v.copy()


class KalmanTargetTracker:
    """Constant-velocity position/velocity Kalman filter with innovation gating."""
    def __init__(self, acceleration_noise=.8, gate_threshold=16.27):
        self.acceleration_noise=float(acceleration_noise)
        self.gate_threshold=float(gate_threshold)
        self.state=np.zeros(6,dtype=float)
        self.covariance=np.diag((.25,.25,.25,2.,2.,2.))
        self.ready=False
        self.last_innovation=np.zeros(3,dtype=float)
        self.last_mahalanobis=0.0
        self.last_measurement_accepted=False

    def reset(self):
        self.state[:]=0
        self.covariance=np.diag((.25,.25,.25,2.,2.,2.))
        self.ready=False
        self.last_innovation[:]=0
        self.last_mahalanobis=0.0
        self.last_measurement_accepted=False

    def predict(self,dt):
        dt=max(float(dt),1e-3)
        f=np.eye(6)
        f[:3,3:]=np.eye(3)*dt
        q=np.zeros((6,6))
        q[:3,:3]=np.eye(3)*(dt**3/3)*self.acceleration_noise**2
        q[:3,3:]=np.eye(3)*(dt**2/2)*self.acceleration_noise**2
        q[3:,:3]=q[:3,3:]
        q[3:,3:]=np.eye(3)*dt*self.acceleration_noise**2
        self.state=f@self.state
        self.covariance=f@self.covariance@f.T+q
        self.covariance=(self.covariance+self.covariance.T)*.5
        return self.state[:3].copy(),self.state[3:].copy()

    def update(self,measurement,dt,measurement_std=.10):
        z=np.asarray(measurement,dtype=float).reshape(3)
        if not self.ready:
            self.state[:3]=z
            self.covariance[:3,:3]=np.eye(3)*float(measurement_std)**2
            self.ready=True
            self.last_measurement_accepted=True
            return self.state[:3].copy(),self.state[3:].copy()
        self.predict(dt)
        h=np.zeros((3,6)); h[:,:3]=np.eye(3)
        r=np.eye(3)*float(measurement_std)**2
        innovation=z-h@self.state
        s=h@self.covariance@h.T+r
        mahal=float(innovation@np.linalg.solve(s,innovation))
        self.last_innovation=innovation.copy()
        self.last_mahalanobis=mahal
        self.last_measurement_accepted=bool(np.isfinite(mahal) and mahal<=self.gate_threshold)
        if self.last_measurement_accepted:
            gain=np.linalg.solve(s,h@self.covariance).T
            self.state=self.state+gain@innovation
            identity=np.eye(6)
            residual=identity-gain@h
            self.covariance=residual@self.covariance@residual.T+gain@r@gain.T
            self.covariance=(self.covariance+self.covariance.T)*.5
        return self.state[:3].copy(),self.state[3:].copy()

    def extrapolate(self,horizon_s):
        horizon=max(0.0,float(horizon_s))
        return self.state[:3]+self.state[3:]*horizon
