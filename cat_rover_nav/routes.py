"""Route files bind ordered observation poses to one immutable map."""
import json
import math
from pathlib import Path
from .models import Pose
from .config import number
from .storage import map_identity


def validate_route(route,grid,cfg=None):
    cfg=cfg or grid.cfg
    if (not isinstance(route,dict) or set(route)!={'version','map_identity','start','goals'}
        or route['version']!=1 or route['map_identity']!=map_identity(grid)):
        raise ValueError('Route version/map identity mismatch; select the correct map')
    def pose(value):
        if not isinstance(value,list) or len(value)!=3 or any(not number(v) for v in value):
            raise ValueError('Pose requires finite [x metres,y metres,yaw radians]')
        if abs(value[2])>math.pi: raise ValueError('Yaw must be between -pi and pi')
        if not grid.footprint_clear(Pose(*value),cfg):
            raise ValueError(f'Pose {value} overlaps occupied, unknown or out-of-bounds space')
    pose(route['start'])
    if not isinstance(route['goals'],list) or not 1<=len(route['goals'])<=100:
        raise ValueError('Route requires 1..100 ordered intermediate/end points')
    for goal in route['goals']:
        if not isinstance(goal,dict) or set(goal)!={'pose','dwell_s'}:
            raise ValueError('Each goal requires pose and dwell_s')
        pose(goal['pose'])
        if not number(goal['dwell_s']) or not 0<=goal['dwell_s']<=3600:
            raise ValueError('dwell_s must be between 0 and 3600 seconds')
    return route


def load_route(path,grid,cfg=None):
    return validate_route(json.loads(Path(path).read_text()),grid,cfg)


def compatible_config(cfg,saved):
    # Changing scan coordinates or the robot footprint invalidates map/route assumptions.
    keys=('grid_resolution_m','map_size_m','survey_bounds','body_length_m','body_width_m','margin_m',
          'track_width_m','wheel_diameter_m','counts_per_rev','lidar_angle_sign','lidar_yaw_deg',
          'lidar_x_m','lidar_y_m','lidar_self_filter','lidar_occlusion_sectors_deg')
    different=[k for k in keys if cfg[k]!=saved[k]]
    if different: raise ValueError('Map requires original calibrated geometry: '+', '.join(different))
