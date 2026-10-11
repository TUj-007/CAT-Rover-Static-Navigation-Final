"""A* on a 16-heading lattice with checked rectangular motion primitives."""
import heapq
import math
import numpy as np
from scipy.ndimage import distance_transform_edt
from .models import Pose
from .geometry import wrap,distance


class Planner:
    def __init__(self,grid,cfg):
        self.grid=grid; self.cfg=cfg

    def _volumes(self,layers):
        g=self.grid; blocked=g.state()!=0
        for layer in layers: blocked=blocked|layer
        yy,xx=np.indices(blocked.shape)
        xmin,xmax,ymin,ymax=self.cfg['survey_bounds']
        wx=g.origin+(xx+.5)*g.res; wy=g.origin+(yy+.5)*g.res
        blocked=blocked|(wx<xmin)|(wx>xmax)|(wy<ymin)|(wy>ymax)
        return distance_transform_edt(~blocked)*g.res

    def plan(self,start,goal,layers=()):
        g=self.grid; cfg=self.cfg
        if not g.footprint_clear(start,cfg,layers) or not g.footprint_clear(goal,cfg,layers): return []
        clearance=self._volumes(layers)
        collision_cache={}
        def key(p):
            return (*g.cell(p.x,p.y),int(round(p.yaw/(math.pi/8)))%16)
        def allowed(p):
            x,y,h=key(p)
            if not g.inside((x,y)): return False
            # Cell-centre dilation can falsely block a real sub-cell pose in a tight corridor.
            # The exact continuous footprint is also needed between lattice headings.
            pk=(round(p.x,7),round(p.y,7),round(p.yaw,7))
            if pk not in collision_cache:
                collision_cache[pk]=g.footprint_clear(p,cfg,layers)
            return collision_cache[pk]
        def primitive(a,b):
            length=distance(a,b); turn=abs(wrap(b.yaw-a.yaw))
            samples=max(2,math.ceil(length/(g.res*.45)),math.ceil(turn/.06))
            for t in np.linspace(0,1,samples+1):
                p=Pose(a.x+t*(b.x-a.x),a.y+t*(b.y-a.y),wrap(a.yaw+t*wrap(b.yaw-a.yaw)))
                if not allowed(p): return False
            return True
        startkey=key(start); costs={startkey:0.}; parents={startkey:None}; poses={startkey:start}
        h=lambda p: distance(p,goal)+.08*abs(wrap(goal.yaw-p.yaw))
        queue=[(h(start),0.,0,startkey)]; serial=0; expansions=0; finish=None
        step=2*g.res
        while queue and expansions<cfg['planning_max_expansions']:
            _,cost,_,pk=heapq.heappop(queue)
            if cost!=costs[pk]: continue
            p=poses[pk]; expansions+=1
            if distance(p,goal)<=cfg['goal_tolerance_m'] and abs(wrap(p.yaw-goal.yaw))<=.20:
                if primitive(p,goal): finish=pk; break
            candidates=[]
            heading=round(p.yaw/(math.pi/8))*math.pi/8
            for sign in (1,-1):
                q=Pose(p.x+sign*step*math.cos(heading),p.y+sign*step*math.sin(heading),wrap(heading))
                candidates.append((q,step*(1 if sign>0 else 1.35)))
            for sign in (-1,1):
                candidates.append((Pose(p.x,p.y,wrap(heading+sign*math.pi/8)),.07))
            for q,move_cost in candidates:
                qk=key(q)
                if qk==pk or not allowed(q) or not primitive(p,q): continue
                x,y,_=qk
                nc=cost+move_cost+.002/(clearance[y,x]+.05)
                if nc>=costs.get(qk,math.inf): continue
                costs[qk]=nc; parents[qk]=pk; poses[qk]=q; serial+=1
                heapq.heappush(queue,(nc+h(q),nc,serial,qk))
        if finish is None: return []
        path=[]
        while finish is not None: path.append(poses[finish]); finish=parents[finish]
        path.reverse()
        if distance(path[-1],goal)>1e-6 or abs(wrap(path[-1].yaw-goal.yaw))>1e-6: path.append(goal)
        # Lattice speedup is conservative but the final exact footprint is authoritative.
        if not all(g.footprint_clear(p,cfg,layers) for p in path): return []
        return path
