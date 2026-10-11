"""Frontier targets are observation poses in known free space, never unknown cells."""
import math
import numpy as np
from scipy.ndimage import binary_dilation,label,distance_transform_edt
from .models import Pose
from .geometry import distance


class Explorer:
    def __init__(self,grid,cfg):
        self.grid=grid; self.cfg=cfg; self.failures={}; self.unresolved=[]
        self.last_frontiers=0; self.unreachable=0; self.ready_path=[]

    def frontiers(self):
        g=self.grid; state=g.state(); unknown=~g.observed
        frontier=(state==0)&binary_dilation(unknown,structure=np.array([[0,1,0],[1,1,1],[0,1,0]]))
        yy,xx=np.indices(state.shape); a,b,c,d=self.cfg['survey_bounds']
        wx=g.origin+(xx+.5)*g.res; wy=g.origin+(yy+.5)*g.res
        frontier &= (wx>a+g.res)&(wx<b-g.res)&(wy>c+g.res)&(wy<d-g.res)
        self.last_frontiers=int(frontier.sum())
        return frontier

    def visible_gain(self,pose):
        """Unknown cells visible before the first known obstacle, in the survey area.

        One-cell obstacle expansion closes sampling pinholes in mapped surfaces for
        information estimates only. Collision planning still uses the exact footprint.
        """
        g=self.grid; c,s=math.cos(pose.yaw),math.sin(pose.yaw)
        origin=np.array([pose.x+c*self.cfg['lidar_x_m']-s*self.cfg['lidar_y_m'],
                         pose.y+s*self.cfg['lidar_x_m']+c*self.cfg['lidar_y_m']])
        a=np.linspace(0,2*np.pi,360,endpoint=False)
        r=np.arange(g.res/2,self.cfg['lidar_max_m'],g.res/2)
        points=origin+np.c_[np.cos(a),np.sin(a)][:,None,:]*r[None,:,None]
        cells=np.floor((points-g.origin)/g.res).astype(int)
        valid=np.all((cells>=0)&(cells<g.n),axis=2)
        x,y=np.clip(cells[:,:,0],0,g.n-1),np.clip(cells[:,:,1],0,g.n-1)
        blocked=binary_dilation(g.state()==1,structure=np.ones((3,3),bool))
        stop=(~valid)|blocked[y,x]
        visible=(np.cumsum(stop,axis=1)==0)&valid&(~g.observed[y,x])
        lo,hi,bot,top=self.cfg['survey_bounds']
        visible&=(points[:,:,0]>=lo)&(points[:,:,0]<=hi)&(points[:,:,1]>=bot)&(points[:,:,1]<=top)
        return len(np.unique(y[visible]*g.n+x[visible]))

    def choose(self,pose,planner):
        g=self.grid; cfg=self.cfg; frontier=self.frontiers(); groups,num=label(frontier)
        self.ready_path=[]; self.unreachable=0
        free=g.state()==0; connected,count=label(free); cell=g.cell(pose.x,pose.y)
        region=connected[cell[1],cell[0]] if g.inside(cell) else 0
        reach=(connected==region)&free if region else np.zeros_like(free)
        ys,xs=np.nonzero(reach); candidates=[]
        if not len(xs): return None
        xy=np.c_[g.origin+(xs+.5)*g.res,g.origin+(ys+.5)*g.res]
        setback=math.hypot(cfg['body_length_m']/2,cfg['body_width_m']/2)+cfg['margin_m']+2*g.res
        anchors=[]
        for group in range(1,num+1):
            fy,fx=np.nonzero(groups==group)
            if len(fx)<cfg['frontier_min_cells']: continue
            frontier_xy=np.c_[g.origin+(fx+.5)*g.res,g.origin+(fy+.5)*g.res]
            tiles=np.floor(frontier_xy/.5).astype(int)
            for tile in np.unique(tiles,axis=0):
                anchors.append(np.mean(frontier_xy[np.all(tiles==tile,axis=1)],axis=0))
        for centre in anchors:
            d=np.linalg.norm(xy-centre,axis=1)
            possible=np.flatnonzero((d>=setback)&(d<=setback+.40))
            possible=sorted(possible,key=lambda i:d[i]+.3*np.linalg.norm(xy[i]-[pose.x,pose.y]))
            found=False; alternatives=0
            for i in possible[:60]:
                x,y=xy[i]; yaw=math.atan2(centre[1]-y,centre[0]-x)
                # Prefer current heading when valid; orientation can change later on the checked path.
                target=Pose(float(x),float(y),round(yaw/(math.pi/8))*math.pi/8)
                key=g.cell(x,y)
                if self.failures.get(key,0)>=cfg['frontier_max_failures']: continue
                if not g.footprint_clear(target,cfg): continue
                gain=self.visible_gain(target)
                if gain<cfg['frontier_min_gain_cells']: continue
                score=gain/(.5+distance(target,pose))
                candidates.append((score/(1+alternatives*.1),target)); found=True; alternatives+=1
                if alternatives>=4: break
            # A mapped wall/pinhole can have an adjacency frontier but no observable gain.
            if not found and not len(possible): self.unreachable+=1
        candidates.sort(key=lambda item:item[0],reverse=True)
        for _,target in candidates:
            path=planner.plan(pose,target)
            if path:
                self.ready_path=path; return target
            self.unreachable+=1
        return None

    def report_target_failure(self,target,reason):
        key=self.grid.cell(target.x,target.y)
        self.failures[key]=self.failures.get(key,0)+1
        if self.failures[key]>=self.cfg['frontier_max_failures']:
            self.unresolved.append({'target':[target.x,target.y,target.yaw],'reason':reason})

    def coverage(self):
        state=self.grid.state(); self.frontiers()
        return {'observed_m2':float(self.grid.observed.sum()*self.grid.res**2),
                'free_m2':float((state==0).sum()*self.grid.res**2),
                'unknown_m2':float((state==-1).sum()*self.grid.res**2),
                'frontier_cells':self.last_frontiers,'unreachable_frontier_regions':self.unreachable,
                'unresolved_failures':len(self.unresolved),'clipped_rays':self.grid.clipped_rays,
                'survey_bounds':self.cfg['survey_bounds']}

    def completion(self):
        self.unresolved=[r for r in self.unresolved if self.visible_gain(Pose(*r['target']))>=self.cfg['frontier_min_gain_cells']]
        if self.unresolved: return 'partial','Exploration targets exhausted their retries'
        if self.grid.clipped_rays: return 'partial','LiDAR hits exceed map extent; enlarge map and survey again'
        if self.unreachable: return 'partial','Unresolved frontiers remain; footprint/path search could not reach observation poses'
        return 'complete','No remaining reachable observation frontier; inaccessible/unknown space reported separately'
