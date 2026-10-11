import math
import numpy as np
from .geometry import Pose,integrate,distance,wrap


class LiveObstacleLayer:
    def __init__(self,grid,cfg):
        self.grid=grid; self.cfg=cfg
        self.last_seen=np.full((grid.n,grid.n),-math.inf)

    def update(self,scan,pose,now):
        from .geometry import transform
        cells=np.floor((transform(scan.points,pose)-self.grid.origin)/self.grid.res).astype(int)
        keep=np.all((cells>=0)&(cells<self.grid.n),axis=1); cells=cells[keep]
        if len(cells): self.last_seen[cells[:,1],cells[:,0]]=now

    def mask(self,now):
        return now-self.last_seen<=self.cfg['live_obstacle_ttl_s']


def track(pose,path,measured_v,cfg):
    if not path: return 0.,0.
    # Discard only reached translation nodes; preserve planned in-place rotations.
    target=path[-1]
    for point in path:
        if distance(point,pose)>.04 or abs(wrap(point.yaw-pose.yaw))>.12:
            target=point; break
    d=distance(pose,target)
    if d<.045:
        error=wrap(target.yaw-pose.yaw); v=0.
    else:
        bearing=math.atan2(target.y-pose.y,target.x-pose.x)
        forward=abs(wrap(bearing-target.yaw))<math.pi/2
        error=wrap(bearing-pose.yaw-(0 if forward else math.pi))
        v=(1 if forward else -1)*min(cfg['cruise_mps'],.8*d)*max(0,math.cos(error)) if abs(error)<.25 else 0.
    w=max(-cfg['max_yaw_radps'],min(cfg['max_yaw_radps'],1.5*error))
    left,right=v-w*cfg['track_width_m']/2,v+w*cfg['track_width_m']/2
    scale=max(1,abs(left)/cfg['max_wheel_mps'],abs(right)/cfg['max_wheel_mps'])
    return left/scale,right/scale


def points_in_body(points,pose,cfg,extra=0):
    from .geometry import transform,inverse
    local=transform(points,inverse(pose))
    return (np.abs(local[:,0])<=cfg['body_length_m']/2+cfg['margin_m']+extra)&(
            np.abs(local[:,1])<=cfg['body_width_m']/2+cfg['margin_m']+extra)


def safe_command(pose,left,right,scan,grid,cfg,measured_v=0,layers=(),scan_pose=None,measured_w=0):
    from .geometry import transform
    if not all(math.isfinite(v) for v in (left,right,measured_v,measured_w)) or scan is None: return False
    world=transform(scan.points,scan_pose if scan_pose is not None else pose)
    v,w=(left+right)/2,(right-left)/cfg['track_width_m']
    speed=max(abs(v),abs(measured_v))+max(abs(w),abs(measured_w))*cfg['track_width_m']/2
    braking_time=speed/cfg['braking_mps2']
    horizon=cfg['reaction_s']+braking_time
    braking_motions={(bv,bw) for bv in (v,measured_v,(v+measured_v)/2)
                     for bw in (w,measured_w,(w+measured_w)/2)}
    for dt in np.linspace(0,horizon,max(2,math.ceil(horizon/.035)+1)):
        # Requested trajectory and independently measured motion both need stopping clearance.
        # Integrate reaction at measured speed, followed by bounded wheel deceleration.
        after=max(0,float(dt)-cfg['reaction_s'])
        elapsed=min(float(dt),cfg['reaction_s'])
        if braking_time>0: elapsed+=after-after*after/(2*braking_time)
        # Include residual translation during a commanded turn (and residual yaw
        # during a translation), rather than checking only two disjoint motions.
        poses=[integrate(pose,v,w,float(dt))]
        poses.extend(integrate(pose,bv,bw,elapsed) for bv,bw in braking_motions)
        for p in poses:
            if not grid.footprint_clear(p,cfg,layers) or np.any(points_in_body(world,p,cfg)): return False
    return True
