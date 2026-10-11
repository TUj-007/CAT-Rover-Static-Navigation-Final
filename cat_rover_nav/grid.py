"""Observed occupancy; -1 unknown, 0 free, 1 occupied. Array rows increase with +y."""
import math
import numpy as np
from .geometry import Pose, transform


class OccupancyGrid:
    def __init__(self,cfg):
        self.cfg=dict(cfg); self.res=cfg['grid_resolution_m']
        self.n=int(math.ceil(cfg['map_size_m']/self.res)); self.origin=-self.n*self.res/2
        self.logodds=np.zeros((self.n,self.n),dtype=np.float32)
        self.observed=np.zeros((self.n,self.n),dtype=bool)
        self.revision=0; self.clipped_rays=0; self._state=None

    def cell(self,x,y):
        return math.floor((x-self.origin)/self.res),math.floor((y-self.origin)/self.res)

    def world(self,cell):
        return self.origin+(cell[0]+.5)*self.res,self.origin+(cell[1]+.5)*self.res

    def inside(self,cell):
        return 0<=cell[0]<self.n and 0<=cell[1]<self.n

    def in_survey(self,x,y):
        a,b,c,d=self.cfg['survey_bounds']
        return a<=x<=b and c<=y<=d

    def state(self):
        if self._state is None:
            out=np.full((self.n,self.n),-1,dtype=np.int8)
            out[self.observed & (self.logodds<=-.45)]=0
            out[self.observed & (self.logodds>=.65)]=1
            self._state=out
        return self._state

    def changed(self):
        self.revision+=1; self._state=None

    def value(self,x,y):
        p=self.cell(x,y)
        return int(self.state()[p[1],p[0]]) if self.inside(p) else -1

    def footprint_cells(self,pose,cfg,margin=None):
        margin=cfg['margin_m'] if margin is None else margin
        hx=cfg['body_length_m']/2+margin; hy=cfg['body_width_m']/2+margin
        r=math.hypot(hx,hy)+self.res
        x0,y0=self.cell(pose.x-r,pose.y-r); x1,y1=self.cell(pose.x+r,pose.y+r)
        xx,yy=np.meshgrid(np.arange(x0,x1+1),np.arange(y0,y1+1))
        wx=self.origin+(xx+.5)*self.res-pose.x; wy=self.origin+(yy+.5)*self.res-pose.y
        c,s=math.cos(pose.yaw),math.sin(pose.yaw)
        # Project each cell square onto the body axes for conservative intersection.
        padding=self.res*(abs(c)+abs(s))/2
        mask=(np.abs(c*wx+s*wy)<=hx+padding)&(np.abs(-s*wx+c*wy)<=hy+padding)
        return xx[mask],yy[mask]

    def footprint_clear(self,pose,cfg,layers=()):
        xx,yy=self.footprint_cells(pose,cfg)
        if not len(xx) or np.any((xx<0)|(yy<0)|(xx>=self.n)|(yy>=self.n)): return False
        wx=self.origin+(xx+.5)*self.res; wy=self.origin+(yy+.5)*self.res
        a,b,c,d=cfg['survey_bounds']
        if np.any((wx<a)|(wx>b)|(wy<c)|(wy>d)): return False
        if np.any(self.state()[yy,xx]!=0): return False
        for layer in layers:
            if np.any(layer[yy,xx]): return False
        return True

    def certify_start(self,pose):
        xx,yy=self.footprint_cells(pose,self.cfg,margin=0)
        valid=(xx>=0)&(yy>=0)&(xx<self.n)&(yy<self.n)
        if not valid.all(): raise RuntimeError('Launch footprint outside map')
        if np.any(self.state()[yy,xx]==1): raise RuntimeError('Launch footprint intersects LiDAR obstacle')
        self.observed[yy,xx]=True; self.logodds[yy,xx]=np.minimum(self.logodds[yy,xx],-.6)
        self.changed()

    def update(self,pose,scan):
        origin=transform([[self.cfg['lidar_x_m'],self.cfg['lidar_y_m']]],pose)[0]
        if not self.inside(self.cell(*origin)): raise RuntimeError('LiDAR origin outside map')
        points=transform(scan.points,pose)
        free=np.zeros_like(self.observed); hits=np.zeros_like(self.observed)
        for p in points:
            steps=max(1,int(np.max(np.abs(p-origin))/self.res*2))
            line=origin+(p-origin)*np.linspace(0,1,steps+1)[:,None]
            cells=np.floor((line-self.origin)/self.res).astype(int)
            valid=np.all((cells>=0)&(cells<self.n),axis=1)
            q=cells[:-1][valid[:-1]]
            if len(q): free[q[:,1],q[:,0]]=True
            if valid[-1]: hits[cells[-1,1],cells[-1,0]]=True
            else: self.clipped_rays+=1
        # Fill tiny angular gaps within a measured surface. Never bridge depth discontinuities.
        for i in range(len(points)):
            j=(i+1)%len(points)
            if len(points)<2: break
            gap=(scan.angles[j]-scan.angles[i])%(2*math.pi)
            if gap>math.radians(self.cfg['ray_interpolation_gap_deg']): continue
            ra=np.linalg.norm(points[i]-origin); rb=np.linalg.norm(points[j]-origin)
            if abs(ra-rb)>.20: continue
            self._triangle_free(origin,points[i],points[j],free)
        free &= ~hits
        self.logodds[free]=np.maximum(-4,self.logodds[free]-.6)
        self.logodds[hits]=np.minimum(4,self.logodds[hits]+1.6)
        self.observed |= free|hits
        self.changed()

    def _triangle_free(self,a,b,c,free):
        lower=np.floor((np.minimum(b,c)-self.origin)/self.res).astype(int)
        upper=np.floor((np.maximum(b,c)-self.origin)/self.res).astype(int)
        oa=np.floor((a-self.origin)/self.res).astype(int)
        lower=np.maximum(0,np.minimum(lower,oa)); upper=np.minimum(self.n-1,np.maximum(upper,oa))
        if np.any(lower>upper): return
        # Broadcast the bounded wedge box; avoid thousands of Python row iterations per scan.
        v=b-a; w=c-a; det=v[0]*w[1]-v[1]*w[0]
        if abs(det)<1e-10: return
        qx=self.origin+(np.arange(lower[0],upper[0]+1)[None,:]+.5)*self.res-a[0]
        qy=self.origin+(np.arange(lower[1],upper[1]+1)[:,None]+.5)*self.res-a[1]
        u=(qx*w[1]-qy*w[0])/det; t=(v[0]*qy-v[1]*qx)/det
        free[lower[1]:upper[1]+1,lower[0]:upper[0]+1]|=(u>=0)&(t>=0)&(u+t<.99)
