from collections import deque
import math
from .geometry import Pose, integrate, wrap
from .protocol import delta32


class Odometry:
    def __init__(self,cfg):
        self.cfg=cfg
        self.scale=[math.pi*d/n for d,n in zip(cfg['wheel_diameter_m'],cfg['counts_per_rev'])]
        self.pose=Pose(); self.v=self.w=0.0; self.previous=None
        self.history=deque(maxlen=160)

    def update(self,p):
        if self.previous is None:
            self.previous=p; self.history.append((p.received,self.pose)); return False
        elapsed=delta32(p.ms,self.previous.ms)
        if elapsed==0: return False
        if elapsed<0 or elapsed>1000:
            raise RuntimeError('Mega reset or telemetry discontinuity; relocalization required')
        dt=elapsed/1000
        ds=[delta32(a,b)*s for a,b,s in zip(p.counts,self.previous.counts,self.scale)]
        if max(map(abs,ds))>dt+4*max(self.scale):
            raise RuntimeError('Implausible encoder change')
        left,right=(ds[0]+ds[1])/2,(ds[2]+ds[3])/2
        self.v=(left+right)/(2*dt); self.w=(right-left)/(self.cfg['track_width_m']*dt)
        self.pose=integrate(self.pose,self.v,self.w,dt)
        self.previous=p; self.history.append((p.received,self.pose)); return True

    def pose_at(self,t):
        if not self.history or t<self.history[0][0] or t>self.history[-1][0]+.12: return None
        if t>=self.history[-1][0]:
            return integrate(self.pose,self.v,self.w,t-self.history[-1][0])
        for (ta,a),(tb,b) in zip(self.history,list(self.history)[1:]):
            if ta<=t<=tb and tb>ta:
                f=(t-ta)/(tb-ta)
                return Pose(a.x+f*(b.x-a.x),a.y+f*(b.y-a.y),wrap(a.yaw+f*wrap(b.yaw-a.yaw)))
        return None
