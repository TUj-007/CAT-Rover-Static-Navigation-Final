"""Keyframe scan constraints, conservative loop validation and bounded pose-graph rebuild."""
import numpy as np
from scipy.optimize import least_squares
from .models import Pose,MatchResult
from .geometry import compose,inverse,distance,wrap
from .grid import OccupancyGrid
from .localization import ScanMatcher,reference_cloud


class Mapper:
    def __init__(self,cfg):
        self.cfg=cfg; self.grid=OccupancyGrid(cfg); self.matcher=ScanMatcher(cfg)
        self.map_from_odom=Pose(); self.keyframes=[]; self.edges=[]; self.loop_closures=0; self.reference=None

    def observe(self,scan,odom_pose):
        predicted=compose(self.map_from_odom,odom_pose)
        if not self.keyframes:
            candidate=OccupancyGrid(self.cfg); candidate.update(predicted,scan)
            result=self.matcher.match(scan,predicted,candidate)
            if not result.accepted: return result
            candidate.certify_start(predicted); self.grid=candidate
            result=MatchResult(predicted,True,result.score,result.overlap,'Survey origin established')
        else:
            result=self.matcher.match(scan,predicted,self.grid,self.reference)
            if not result.accepted: return result
            self.grid.update(result.pose,scan)
        self.map_from_odom=compose(result.pose,inverse(odom_pose))
        if (not self.keyframes or distance(result.pose,self.keyframes[-1]['pose'])>=self.cfg['keyframe_distance_m']
            or abs(wrap(result.pose.yaw-self.keyframes[-1]['pose'].yaw))>=self.cfg['keyframe_rotation_rad']):
            previous=self.keyframes[-1] if self.keyframes else None
            self.keyframes.append({'pose':result.pose,'odom':odom_pose,'scan':scan})
            n=len(self.keyframes)-1
            if previous:
                relative=compose(inverse(previous['pose']),result.pose)
                self.edges.append((n-1,n,relative,.06))
            self._try_loop(n)
            self.reference=reference_cloud(self.keyframes,self.grid.res)
        return result

    def _try_loop(self,n):
        if n<12: return
        frame=self.keyframes[n]
        for i in range(n-10):
            old=self.keyframes[i]
            if distance(frame['pose'],old['pose'])>self.cfg['loop_closure_radius_m']: continue
            reference=OccupancyGrid(self.cfg)
            for k in self.keyframes[max(0,i-2):i+3]: reference.update(k['pose'],k['scan'])
            cloud=reference_cloud(self.keyframes[max(0,i-2):i+3],reference.res)
            r=ScanMatcher(self.cfg).match(frame['scan'],frame['pose'],reference,cloud)
            if not r.accepted or r.score>.065 or r.overlap<.85: continue
            delta=compose(inverse(old['pose']),r.pose)
            self.edges.append((i,n,delta,.04))
            if self.optimize(): self.loop_closures+=1
            else: self.edges.pop()
            break

    def optimize(self):
        if len(self.keyframes)<3 or not self.edges: return False
        old=np.array([k['pose'].array() for k in self.keyframes]); anchor=old[0]
        def unpack(x): return np.vstack([anchor,x.reshape(-1,3)])
        def residual(x):
            poses=unpack(x); errors=[]
            for i,j,want,sigma in self.edges:
                got=compose(inverse(Pose(*poses[i])),Pose(*poses[j]))
                errors.extend([(got.x-want.x)/sigma,(got.y-want.y)/sigma,wrap(got.yaw-want.yaw)/sigma])
            return np.asarray(errors)
        x0=old[1:].ravel(); before=float(np.sum(residual(x0)**2))
        fit=least_squares(residual,x0,loss='soft_l1',f_scale=1,max_nfev=40)
        poses=unpack(fit.x)
        if (not fit.success or not np.isfinite(poses).all() or np.sum(residual(fit.x)**2)>before+1e-6
            or np.max(np.linalg.norm(poses[:,:2]-old[:,:2],axis=1))>.35
            or max(abs(wrap(a-b)) for a,b in zip(poses[:,2],old[:,2]))>.25): return False
        rebuilt=OccupancyGrid(self.cfg)
        for k,p in zip(self.keyframes,poses):
            k['pose']=Pose(*p); rebuilt.update(k['pose'],k['scan'])
        rebuilt.certify_start(self.keyframes[0]['pose']); self.grid=rebuilt
        last=self.keyframes[-1]
        self.map_from_odom=compose(last['pose'],inverse(last['odom']))
        self.reference=reference_cloud(self.keyframes,self.grid.res)
        return True
