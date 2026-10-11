import math
import numpy as np
from .models import Pose


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def compose(a, b):
    c,s=math.cos(a.yaw),math.sin(a.yaw)
    return Pose(a.x+c*b.x-s*b.y, a.y+s*b.x+c*b.y, wrap(a.yaw+b.yaw))


def inverse(p):
    c,s=math.cos(p.yaw),math.sin(p.yaw)
    return Pose(-c*p.x-s*p.y, s*p.x-c*p.y, -p.yaw)


def transform(points, pose):
    p=np.asarray(points,dtype=float).reshape(-1,2)
    c,s=math.cos(pose.yaw),math.sin(pose.yaw)
    return p @ np.array([[c,s],[-s,c]]) + [pose.x,pose.y]


def distance(a,b):
    return math.hypot(a.x-b.x,a.y-b.y)


def integrate(pose,v,w,dt):
    if abs(w)<1e-8:
        return compose(pose,Pose(v*dt,0,0))
    return compose(pose,Pose(v*math.sin(w*dt)/w,v*(1-math.cos(w*dt))/w,w*dt))
