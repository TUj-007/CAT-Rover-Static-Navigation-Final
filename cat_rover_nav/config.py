"""Validate before device initialization. All distances are metres."""
import copy
import json
import math
from pathlib import Path

_DEFAULTS = {
    'hardware_confirmed': False, 'calibration_confirmed': False,
    'mega_port': '/dev/serial/by-id/usb-Arduino__www.arduino.cc__0042_5583834323335111D1A0-if00',
    'lidar_port': '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0',
    'lidar_baud': 115200, 'lidar_angle_sign': -1, 'lidar_yaw_deg': 0.0,
    'lidar_x_m': 0.0, 'lidar_y_m': 0.0, 'lidar_min_m': 0.12, 'lidar_max_m': 6.0,
    'lidar_max_age_s': 0.35, 'lidar_min_points': 100, 'lidar_max_gap_deg': 20.0,
    'lidar_occlusion_sectors_deg': [], 'lidar_self_filter': False,
    'wheel_diameter_m': [0.10]*4, 'counts_per_rev': [600.0]*4,
    'track_width_m': 0.4425, 'body_length_m': 0.565, 'body_width_m': 0.40,
    'margin_m': 0.03, 'cruise_mps': 0.08, 'max_wheel_mps': 0.25,
    'max_yaw_radps': 0.50, 'acceleration_mps2': 0.20, 'braking_mps2': 0.30,
    'reaction_s': 0.90, 'telemetry_max_age_s': 0.30,
    'grid_resolution_m': 0.05, 'map_size_m': 6.0,
    'survey_bounds': [-2.9,2.9,-2.9,2.9], 'goal_tolerance_m': 0.10,
    'heading_tolerance_rad': 0.12, 'startup_timeout_s': 12.0,
    'recovery_timeout_s': 4.0, 'mission_timeout_s': 600.0,
    'exploration_timeout_s': 1200.0, 'no_progress_timeout_s': 30.0,
    'checkpoint_interval_s': 15.0, 'replan_interval_s': 0.7,
    'scan_max_yaw_motion_rad': 0.20, 'scan_max_translation_m': 0.06,
    'scan_match_translation_m': 0.15, 'scan_match_rotation_rad': 0.20,
    'scan_match_max_error_m': 0.10, 'scan_match_min_overlap': 0.70,
    'scan_match_min_observability': 0.015, 'scan_match_ambiguity_m': 0.005,
    'keyframe_distance_m': 0.20, 'keyframe_rotation_rad': 0.30,
    'loop_closure_radius_m': 0.35, 'frontier_min_cells': 3,
    'frontier_min_gain_cells': 8, 'frontier_max_failures': 3,
    'planning_max_expansions': 18000, 'live_obstacle_ttl_s': 2.0,
    'ray_interpolation_gap_deg': 3.0,
}


def defaults():
    return copy.deepcopy(_DEFAULTS)


def number(v):
    return type(v) in (int,float) and math.isfinite(v)


def validate(c):
    if set(c)!=set(_DEFAULTS):
        raise ValueError(f'Configuration keys differ: {set(c)^set(_DEFAULTS)}')
    for k,example in _DEFAULTS.items():
        v=c[k]
        if type(example) is bool and type(v) is not bool:
            raise ValueError(f'{k} must be boolean')
        if type(example) in (int,float) and not number(v):
            raise ValueError(f'{k} must be a finite number')
        if type(example) is int and type(v) is not int:
            raise ValueError(f'{k} must be an integer')
        if isinstance(example,str) and (not isinstance(v,str) or not v.strip()):
            raise ValueError(f'{k} must be a nonempty string')
    positive=[k for k,v in _DEFAULTS.items() if type(v) in (int,float) and v>0
              and k not in ('lidar_angle_sign',)]
    for k in positive:
        if c[k]<=0: raise ValueError(f'{k} must be positive')
    for k in ('wheel_diameter_m','counts_per_rev'):
        if not isinstance(c[k],list) or len(c[k])!=4 or any(not number(v) or v<=0 for v in c[k]):
            raise ValueError(f'{k} requires four measured positive values')
    bounds=c['survey_bounds']; half=c['map_size_m']/2
    if (not isinstance(bounds,list) or len(bounds)!=4 or any(not number(v) for v in bounds)
        or not -half<=bounds[0]<bounds[1]<=half or not -half<=bounds[2]<bounds[3]<=half):
        raise ValueError('Survey bounds must fit the map: [xmin,xmax,ymin,ymax]')
    sectors=c['lidar_occlusion_sectors_deg']
    if not isinstance(sectors,list) or any(not isinstance(s,list) or len(s)!=2
        or any(not number(v) or not 0<=v<360 for v in s) for s in sectors):
        raise ValueError('Occlusion sectors require [start,end] degrees in [0,360)')
    if c['mega_port']==c['lidar_port']: raise ValueError('Mega and LiDAR ports must differ')
    if c['lidar_angle_sign'] not in (-1,1): raise ValueError('LiDAR sign must be -1 or 1')
    if c['lidar_baud']!=115200: raise ValueError('A1M8 adapter uses 115200 baud')
    if not c['lidar_min_m']<c['lidar_max_m']<=6: raise ValueError('Invalid A1M8 range')
    if not 0<c['cruise_mps']<=c['max_wheel_mps']<=.30: raise ValueError('Invalid wheel speed limits')
    if c['lidar_max_age_s']>.5 or c['telemetry_max_age_s']>.30 or c['lidar_max_gap_deg']>20:
        raise ValueError('Sensor freshness/coverage exceeds supported limits')
    if c['reaction_s']<c['lidar_max_age_s']+.30+.15: raise ValueError('Reaction budget too short')
    if not 20<=c['map_size_m']/c['grid_resolution_m']<=240: raise ValueError('Map needs 20..240 cells/side')
    if c['lidar_min_points']<30 or c['ray_interpolation_gap_deg']>5:
        raise ValueError('Insufficient ray coverage settings')
    if not 0<c['scan_match_min_overlap']<=1: raise ValueError('Invalid match overlap')
    return c


def load_config(path):
    supplied=json.loads(Path(path).read_text())
    if not isinstance(supplied,dict) or set(supplied)-set(_DEFAULTS):
        raise ValueError('Unknown configuration keys')
    c=defaults(); c.update(supplied)
    return validate(c)
