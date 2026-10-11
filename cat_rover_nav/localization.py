"""Bounded scan matching with held-out residual and geometric observability gates."""
import math
import numpy as np
from scipy.spatial import cKDTree
from scipy.optimize import least_squares
from .models import Pose,MatchResult
from .geometry import compose,inverse,transform,wrap,distance


class ScanMatcher:
    def __init__(self,cfg):
        self.cfg=cfg; self.cache_key=None; self.tree=None; self.points=None; self.normals=None; self.reference=None

    def _prepare(self,grid,reference=None):
        key=(id(grid),grid.revision) if reference is None else ('raw',id(reference))
        if key==self.cache_key: return
        self.reference=reference
        if reference is None:
            yy,xx=np.nonzero(grid.state()==1)
            self.points=np.c_[grid.origin+(xx+.5)*grid.res,grid.origin+(yy+.5)*grid.res]
        else:
            self.points=reference
        self.cache_key=key; self.tree=None
        if len(self.points)<20: return
        self.tree=cKDTree(self.points)
        _,idx=self.tree.query(self.points,k=min(9,len(self.points)))
        near=self.points[idx]; centred=near-near.mean(axis=1,keepdims=True)
        cov=np.einsum('nki,nkj->nij',centred,centred)
        _,vectors=np.linalg.eigh(cov); self.normals=vectors[:,:,0]

    def match(self,scan,predicted,grid,reference_points=None):
        self._prepare(grid,reference_points)
        reject=lambda reason,score=math.inf,overlap=0: MatchResult(predicted,False,score,overlap,reason)
        if self.tree is None or len(scan.points)<30: return reject('Insufficient geometry')
        pts=scan.points[::max(1,len(scan.points)//180)]
        if not np.isfinite(pts).all(): return reject('Nonfinite scan')
        extent=self.cfg['scan_match_translation_m']; angle=self.cfg['scan_match_rotation_rad']
        samples=pts[::2]; seeds=[]
        for da in np.linspace(-angle,angle,5):
            for dx in np.linspace(-extent,extent,5):
                for dy in np.linspace(-extent,extent,5):
                    pose=Pose(predicted.x+dx,predicted.y+dy,wrap(predicted.yaw+da))
                    ds,_=self.tree.query(transform(samples,pose))
                    seeds.append((float(np.mean(np.minimum(ds,.3)**2)),pose))
        seeds.sort(key=lambda item:item[0]); fits=[]
        lower=np.array([predicted.x-extent,predicted.y-extent,predicted.yaw-angle])
        upper=np.array([predicted.x+extent,predicted.y+extent,predicted.yaw+angle])
        for _,seed in seeds[:3]:
            initial=np.array([seed.x,seed.y,predicted.yaw+wrap(seed.yaw-predicted.yaw)])
            def residual(x):
                world=transform(pts[::2],Pose(*x)); ds,idx=self.tree.query(world)
                signed=np.sum((world-self.points[idx])*self.normals[idx],axis=1)
                # Far correspondences contribute bounded distance rather than a false plane.
                return np.where(ds<.25,signed,np.minimum(ds,.3))
            fit=least_squares(residual,np.clip(initial,lower+1e-9,upper-1e-9),bounds=(lower,upper),
                              loss='soft_l1',f_scale=.035,max_nfev=30)
            pose=Pose(float(fit.x[0]),float(fit.x[1]),wrap(float(fit.x[2])))
            ds,idx=self.tree.query(transform(pts,pose))
            overlap=float(np.mean(ds<.16)); score=float(np.sqrt(np.mean(np.minimum(ds,.3)**2)))
            fits.append((score,overlap,pose,ds,idx))
        fits.sort(key=lambda f:f[0]); score,overlap,pose,ds,idx=fits[0]
        if overlap<self.cfg['scan_match_min_overlap'] or score>self.cfg['scan_match_max_error_m']:
            return reject('Scan/map disagreement',score,overlap)
        for other in fits[1:]:
            separated=distance(other[2],pose)>.08 or abs(wrap(other[2].yaw-pose.yaw))>.08
            if separated and other[0]-score<self.cfg['scan_match_ambiguity_m']:
                return reject('Competing pose hypotheses',score,overlap)
        keep=ds<.16
        n=self.normals[idx[keep]]; world=transform(pts[keep],Pose(0,0,pose.yaw))
        jac=np.c_[n[:,0],n[:,1],-n[:,0]*world[:,1]+n[:,1]*world[:,0]]
        # Rescale yaw by scene range so axes are comparable. Eigenvalues detect unobservable DOF.
        jac[:,2]/=max(.5,float(np.median(np.linalg.norm(world,axis=1))))
        ev=np.linalg.eigvalsh(jac.T@jac/max(1,len(jac)))
        if ev[0]/max(ev[-1],1e-9)<self.cfg['scan_match_min_observability']:
            return reject('Unobservable translation/yaw',score,overlap)
        return MatchResult(pose,True,score,overlap,'LiDAR alignment accepted')


def reference_cloud(frames,resolution):
    """Mean measured endpoints in fine voxels, never snapped to voxel centres."""
    if not frames: return None
    raw=np.vstack([transform(k['scan'].points,k['pose']) for k in frames])
    if not len(raw): return None
    voxels=np.floor(raw/(resolution/2)).astype(np.int64)
    _,indices=np.unique(voxels,axis=0,return_inverse=True)
    count=np.bincount(indices)
    return np.c_[np.bincount(indices,weights=raw[:,0])/count,np.bincount(indices,weights=raw[:,1])/count]


class Localizer:
    def __init__(self,grid,cfg,initial_pose,keyframes=()):
        self.grid=grid; self.matcher=ScanMatcher(cfg); self.map_from_odom=initial_pose
        self.reference=reference_cloud(keyframes,grid.res)

    def observe(self,scan,odom_pose):
        predicted=compose(self.map_from_odom,odom_pose)
        result=self.matcher.match(scan,predicted,self.grid,self.reference)
        if result.accepted: self.map_from_odom=compose(result.pose,inverse(odom_pose))
        return result
