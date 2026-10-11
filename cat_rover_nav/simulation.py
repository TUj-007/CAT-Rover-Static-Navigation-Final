"""Deterministic ray/encoder world exercising the production runtime, without hardware."""
import json
import math
from dataclasses import asdict
from pathlib import Path
import numpy as np
from .config import defaults
from .models import Pose,Scan,Telemetry
from .geometry import integrate,distance,wrap
from .motion import points_in_body
from .runtime import run_exploration,run_navigation
from .storage import load_map,map_identity
from .routes import validate_route


class World:
    def __init__(self,cfg,obstacles=False,dropout=False):
        self.cfg=cfg; self.t=0.; self.pose=Pose(); self.left=self.right=0.
        self.counts=np.zeros(4); self.latest=None; self.scan=None; self.scan_time=-1.
        self.collisions=0; self.max_stop_delay=0.; self.dropout=dropout; self.synchronous_planning=True
        self.closed=False; self.commands=[]; self.travel=0.
        self.bounds=(-2.,2.5,-1.8,2.2)
        self.boxes=obstacles if isinstance(obstacles,list) else ([(.65,.95,-.25,.25)] if obstacles else [])
        self.ambiguous=False
        x0,x1,y0,y1=self.bounds
        self.segments=[((x0,y0),(x1,y0)),((x1,y0),(x1,y1)),((x1,y1),(x0,y1)),((x0,y1),(x0,y0))]
        for a,b,c,d in self.boxes:
            self.segments.extend([((a,c),(b,c)),((b,c),(b,d)),((b,d),(a,d)),((a,d),(a,c))])
        self.a=np.linspace(0,2*np.pi,360,endpoint=False)
    def identity(self): return {'protocol':2,'encoder_edges':1,'motor_layout':1}
    def get(self):
        self.latest=Telemetry(round(self.t*1000)&0xffffffff,1,2,0,tuple(int(round(v))&0xffffffff for v in self.counts),self.t)
        return self.latest
    def send(self,p,left=0,right=0):
        self.left,self.right=left,right; self.commands.append((self.t,left,right))
    def close(self): self.left=self.right=0.; self.closed=True
    def tick(self,dt):
        v,w=(self.left+self.right)/2,(self.right-self.left)/self.cfg['track_width_m']
        before=self.pose; self.pose=integrate(before,v,w,dt); self.travel+=distance(before,self.pose)
        wheel=np.array([self.left,self.left,self.right,self.right])*dt
        # Realistic small unequal odometry scale error; matching must correct the drift.
        self.counts+=wheel*np.array([1.01,1.008,.997,.998])*np.array(self.cfg['counts_per_rev'])/(np.pi*np.array(self.cfg['wheel_diameter_m']))
        self.t+=dt
        a,b,c,d=self.bounds; p=self.pose; co,si=math.cos(p.yaw),math.sin(p.yaw)
        corners=np.array([[x,y] for x in (-self.cfg['body_length_m']/2,self.cfg['body_length_m']/2)
                         for y in (-self.cfg['body_width_m']/2,self.cfg['body_width_m']/2)])@np.array([[co,si],[-si,co]])+np.array([p.x,p.y])
        collision=bool((corners[:,0]<=a).any() or (corners[:,0]>=b).any() or (corners[:,1]<=c).any() or (corners[:,1]>=d).any())
        for a,b,c,d in self.boxes:
            # Segment samples check intersection, including corners wholly enclosing a small box.
            edge=np.vstack([np.c_[np.linspace(a,b,30),np.full(30,c)],np.c_[np.linspace(a,b,30),np.full(30,d)],
                            np.c_[np.full(30,a),np.linspace(c,d,30)],np.c_[np.full(30,b),np.linspace(c,d,30)]])
            collision|=bool(points_in_body(edge,p,dict(self.cfg,margin_m=0)).any())
        if collision: self.collisions+=1; raise RuntimeError('Simulator ground-truth collision')
    def scan_get(self):
        if self.dropout and self.t>=.5: return self.scan
        if self.t-self.scan_time>=.20-1e-8:
            self.scan_time=self.t; p=self.pose; angles=self.a+p.yaw
            if self.ambiguous:
                self.scan=Scan(np.c_[np.cos(self.a),np.sin(self.a)]*2.,self.a,self.t,self.t)
                return self.scan
            rays=np.c_[np.cos(angles),np.sin(angles)]; origin=np.array([p.x,p.y]); ranges=np.full(len(rays),np.inf)
            cross=lambda u,v:u[...,0]*v[...,1]-u[...,1]*v[...,0]
            for aa,bb in self.segments:
                q=np.asarray(aa)-origin; s=np.asarray(bb)-np.asarray(aa); den=cross(rays,s)
                with np.errstate(divide='ignore',invalid='ignore'):
                    t=cross(q,s)/den; u=cross(q,rays)/den
                good=(t>self.cfg['lidar_min_m'])&(u>=0)&(u<=1)&(np.abs(den)>1e-9)
                ranges=np.minimum(ranges,np.where(good,t,np.inf))
            keep=np.isfinite(ranges)&(ranges<=self.cfg['lidar_max_m'])
            ranges=ranges+.002*np.sin(3*self.a+self.t) # deterministic ±2 mm range noise
            # Synthetic scan is instantaneous; host device tests separately cover assembly age/distortion.
            self.scan=Scan(np.c_[np.cos(self.a[keep]),np.sin(self.a[keep])]*ranges[keep,None],self.a[keep],self.t,self.t)
        return self.scan


class Sensor:
    def __init__(self,world): self.world=world
    def get(self): return self.world.scan_get()
    def close(self): pass


def simulate(output,scenario='room'):
    if scenario not in ('room','obstacle','dropout','ambiguous','live-obstacle'): raise ValueError('Unknown simulation scenario')
    output=Path(output); output.mkdir(parents=True,exist_ok=True)
    cfg=defaults(); cfg['hardware_confirmed']=cfg['calibration_confirmed']=True
    cfg['lidar_angle_sign']=1; cfg['replan_interval_s']=1.5; cfg['mission_timeout_s']=240.
    cfg['exploration_timeout_s']=240.; cfg['checkpoint_interval_s']=30.
    world=World(cfg,obstacles=scenario in ('obstacle','dropout'),dropout=scenario=='dropout')
    world.ambiguous=scenario=='ambiguous'
    map_path=output/'map.npz'
    with (output/'survey.jsonl').open('w') as log:
        survey=run_exploration(cfg,(world,Sensor(world)),map_path,clock=lambda:world.t,sleep=world.tick,log=log)
    surveyed=load_map(map_path)[0]
    yy,xx=np.indices(surveyed.observed.shape)
    x=surveyed.origin+(xx+.5)*surveyed.res; y=surveyed.origin+(yy+.5)*surveyed.res
    lo,hi,bot,top=world.bounds; interior=(x>lo)&(x<hi)&(y>bot)&(y<top)
    for lo,hi,bot,top in world.boxes: interior&=~((x>=lo)&(x<=hi)&(y>=bot)&(y<=top))
    report={'scenario':scenario,'survey':asdict(survey),'survey_travel_m':world.travel,
            'ground_truth_interior_observed_fraction':float(np.mean(surveyed.observed[interior])),
            'nonzero_survey_commands':sum(l!=0 or r!=0 for _,l,r in world.commands),
            'survey_simulated_seconds':world.t,'collisions':world.collisions,
            'final_command_zero':world.left==world.right==0}
    if survey.status=='complete' and scenario not in ('dropout','ambiguous'):
        data=load_map(map_path); identity=map_identity(data[0])
        route={'version':1,'map_identity':identity,'start':[.15,-.2,.10],
               'goals':[{'pose':[.25,0,0],'dwell_s':.2},{'pose':[.25,.6,math.pi/2],'dwell_s':.2},
                        {'pose':[-.2,.6,math.pi],'dwell_s':0}]}
        if scenario=='live-obstacle': route['goals']=[{'pose':[1.5,0,0],'dwell_s':0}]
        validate_route(route,data[0],cfg); (output/'route.json').write_text(json.dumps(route,indent=2))
        obstacles=[(.8,1.,-.1,.1)] if scenario=='live-obstacle' else scenario=='obstacle'
        rover=World(cfg,obstacles=obstacles); rover.pose=Pose(*route['start'])
        with (output/'navigation.jsonl').open('w') as log:
            navigation=run_navigation(cfg,(rover,Sensor(rover)),data,route,clock=lambda:rover.t,sleep=rover.tick,log=log)
        report.update(navigation=asdict(navigation),navigation_simulated_seconds=rover.t,
                      collisions=world.collisions+rover.collisions,end_error_m=distance(rover.pose,Pose(*route['goals'][-1]['pose'])),
                      end_heading_error_rad=abs(wrap(rover.pose.yaw-route['goals'][-1]['pose'][2])),
                      static_map_unchanged=map_identity(data[0])==identity,
                      final_command_zero=rover.left==rover.right==0)
    (output/'report.json').write_text(json.dumps(report,indent=2))
    return report
