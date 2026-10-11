"""The runtime owns commands. Computation never supplies an independent motion heartbeat."""
import copy
import json
import math
import threading
import time
from dataclasses import asdict
import numpy as np
from .models import Pose,RunResult
from .geometry import compose,distance,wrap
from .devices import scan_usable
from .odometry import Odometry
from .slam import Mapper
from .localization import Localizer
from .exploration import Explorer
from .planning import Planner
from .motion import track,safe_command,LiveObstacleLayer
from .storage import save_map,export_map,map_identity


class CommandGuard:
    def __init__(self,cfg): self.cfg=cfg; self.epoch=None
    def note_zero(self,p):
        if p.mode==2: self.epoch=p.epoch
    def allowed(self,p,scan,now,localized,computing):
        return bool(p and p.mode==2 and p.fault==0 and self.epoch==p.epoch and localized
                    and not computing and 0<=now-p.received<=self.cfg['telemetry_max_age_s']
                    and scan_usable(scan,self.cfg,now))


class PlanJob:
    def __init__(self,work,synchronous=False):
        self.result=None; self.error=None; self.done=False
        def execute():
            try: self.result=work()
            except Exception as e: self.error=e
            finally: self.done=True
        if synchronous: execute()
        else: threading.Thread(target=execute,daemon=True).start()


def run_exploration(cfg,devices,output,*,clock=time.monotonic,sleep=time.sleep,log=None):
    return _run(cfg,devices,True,output,None,None,clock,sleep,log)


def run_navigation(cfg,devices,map_data,route,*,clock=time.monotonic,sleep=time.sleep,log=None):
    return _run(cfg,devices,False,None,map_data,route,clock,sleep,log)


def _run(cfg,devices,exploring,output,map_data,route,clock,sleep,log):
    link,lidar=devices; odom=Odometry(cfg); guard=CommandGuard(cfg)
    mapper=Mapper(cfg) if exploring else None
    grid=mapper.grid if mapper else map_data[0]
    localizer=None if exploring else Localizer(grid,cfg,Pose(*route['start']),map_data[1])
    explorer=Explorer(grid,cfg) if exploring else None
    live=LiveObstacleLayer(grid,cfg); started=clock(); now=started; last_scan=None
    pose=Pose(); scan_pose=None; path=[]; target=None; job=None; localized=False; ready=False
    progress_time=started; progress_pose=pose; last_plan=started; last_checkpoint=started
    recovery_since=None; goal_index=0; hold_until=None; survey_observed=0
    station_arrived=False; last_left=last_right=0.; last_loop=started; state='WAIT_SENSORS'
    result=RunResult('partial' if exploring else 'fault','Run interrupted')
    sync=bool(getattr(link,'synchronous_planning',False))
    try:
        while True:
            now=clock(); dt=max(.001,min(.15,now-last_loop)); last_loop=now
            deadline=cfg['exploration_timeout_s'] if exploring else cfg['mission_timeout_s']
            if now-started>deadline: raise RuntimeError('Autonomous run deadline reached')
            pkt=link.get(); scan=lidar.get()
            identity=link.identity()
            if identity and identity!={'protocol':2,'encoder_edges':1,'motor_layout':1}:
                raise RuntimeError('Incompatible firmware; upload the companion Mega sketch')
            if pkt:
                odom.update(pkt)
                if pkt.fault and pkt.fault!=8: raise RuntimeError(f'Mega fault {pkt.fault}; inspect calibration/wiring')
            usable=bool(pkt and identity and 0<=now-pkt.received<=cfg['telemetry_max_age_s']
                        and scan_usable(scan,cfg,now))
            if not usable:
                localized=False; state='RECOVERING_STOPPED' if ready else 'WAIT_SENSORS'
                if pkt:
                    link.send(pkt); guard.note_zero(pkt)
                last_left=last_right=0.
                if recovery_since is None: recovery_since=now
                limit=cfg['recovery_timeout_s'] if ready else cfg['startup_timeout_s']
                if now-recovery_since>limit: raise RuntimeError('Sensor startup/recovery timeout; no usable fresh scan/telemetry')
                sleep(.05); continue
            # Rehandshake on entry and after controller timeout; never jump straight to motion.
            if guard.epoch!=pkt.epoch or pkt.fault==8:
                link.send(pkt); guard.note_zero(pkt); last_left=last_right=0.
                sleep(.05); continue
            scan_key=(scan.generation,scan.acquired)
            if scan_key!=last_scan:
                scan_odom=odom.pose_at(scan.acquired)
                finish_odom=odom.pose_at(scan.completed)
                if scan_odom is None or finish_odom is None:
                    link.send(pkt); last_left=last_right=0.; localized=False
                    if recovery_since is None: recovery_since=now
                    limit=cfg['recovery_timeout_s'] if ready else cfg['startup_timeout_s']
                    if now-recovery_since>limit: raise RuntimeError('No timestamp-aligned odometry for LiDAR')
                    sleep(.05); continue
                if (distance(scan_odom,finish_odom)>cfg['scan_max_translation_m'] or
                    abs(wrap(scan_odom.yaw-finish_odom.yaw))>cfg['scan_max_yaw_motion_rad']):
                    link.send(pkt); localized=False; last_left=last_right=0.
                    if recovery_since is None: recovery_since=now
                    if now-recovery_since>cfg['recovery_timeout_s']: raise RuntimeError('Persistent scan motion distortion')
                    sleep(.05); continue
                estimate=mapper.observe(scan,scan_odom) if exploring else localizer.observe(scan,scan_odom)
                localized=estimate.accepted; last_scan=scan_key
                if not localized:
                    link.send(pkt); last_left=last_right=0.; state='LOCALIZING'
                    if recovery_since is None: recovery_since=now
                    if now-recovery_since>cfg['recovery_timeout_s']: raise RuntimeError(f'Localization lost: {estimate.reason}')
                    sleep(.05); continue
                ready=True; recovery_since=None
                grid=mapper.grid if exploring else grid
                if exploring: explorer.grid=grid
                pose=compose(mapper.map_from_odom if exploring else localizer.map_from_odom,odom.pose)
                scan_pose=compose(mapper.map_from_odom if exploring else localizer.map_from_odom,scan_odom)
                live.grid=grid; live.update(scan,scan_pose,now)
            if not localized:
                link.send(pkt); sleep(.05); continue
            pose=compose(mapper.map_from_odom if exploring else localizer.map_from_odom,odom.pose)
            if distance(pose,progress_pose)>.02 or abs(wrap(pose.yaw-progress_pose.yaw))>.15:
                progress_time=now; progress_pose=pose
            deadline=cfg['exploration_timeout_s'] if exploring else cfg['mission_timeout_s']
            if now-started>deadline: raise RuntimeError('Autonomous run deadline reached')
            if exploring and now-last_checkpoint>=cfg['checkpoint_interval_s']:
                link.send(pkt); last_left=last_right=0.
                save_map(output,grid,mapper.keyframes,{'status':'partial','reason':'Survey checkpoint','coverage':explorer.coverage()})
                last_checkpoint=now
            if not exploring and target is None:
                if goal_index>=len(route['goals']):
                    result=RunResult('complete','All ordered waypoints reached',goal_index); break
                goal=route['goals'][goal_index]; target=Pose(*goal['pose']); path=[]; station_arrived=False
            if target is not None and distance(pose,target)<=cfg['goal_tolerance_m'] and abs(wrap(pose.yaw-target.yaw))<=cfg['heading_tolerance_rad']:
                link.send(pkt); last_left=last_right=0.
                if hold_until is None:
                    hold_until=now+(1.0 if exploring else route['goals'][goal_index].get('dwell_s',0))
                if now>=hold_until:
                    if exploring:
                        gained=int(grid.observed.sum())-survey_observed
                        if gained<cfg['frontier_min_gain_cells']: explorer.report_target_failure(target,'No new observable cells at target')
                    else: goal_index+=1
                    target=None; path=[]; hold_until=None; job=None; progress_time=now
                sleep(.05); continue
            if job is None and (not path or now-last_plan>cfg['replan_interval_s']):
                # Always stop before copying or starting potentially slow work.
                link.send(pkt); last_left=last_right=0.
                snapshot=copy.deepcopy(grid); masks=(live.mask(now).copy(),) if not exploring else ()
                current_pose=pose; desired=target
                failures=copy.deepcopy(explorer.failures) if exploring else None
                def work():
                    planner=Planner(snapshot,cfg)
                    if exploring and desired is None:
                        e=Explorer(snapshot,cfg); e.failures=failures; e.unresolved=copy.deepcopy(explorer.unresolved)
                        choice=e.choose(current_pose,planner)
                        return choice,e.ready_path,e.unreachable,e.completion()
                    return desired,planner.plan(current_pose,desired,masks),0,None
                job=PlanJob(work,sync); last_plan=now; state='PLANNING_STOPPED'
            if job is not None:
                link.send(pkt); last_left=last_right=0.
                if job.done:
                    if job.error: raise RuntimeError(f'Planner error: {job.error}')
                    choice,newpath,unreachable,completion=job.result; job=None
                    if exploring and target is None and choice is None:
                        explorer.unreachable=unreachable
                        status,reason=completion
                        result=RunResult(status,reason,0,explorer.coverage()); break
                    if target is None: survey_observed=int(grid.observed.sum())
                    target=choice; path=newpath
                    if not path:
                        if now-progress_time>cfg['no_progress_timeout_s']:
                            if exploring:
                                explorer.report_target_failure(target,'No navigable path'); target=None; progress_time=now
                            else: raise RuntimeError('No navigable route; rover remains stopped')
                sleep(.05); continue
            if now-progress_time>cfg['no_progress_timeout_s']:
                if exploring:
                    explorer.report_target_failure(target,'No motion progress'); target=None; path=[]; progress_time=now
                    link.send(pkt); sleep(.05); continue
                raise RuntimeError('No motion progress; inspect course or localization')
            # Trim executed nodes while preserving outstanding pivot headings.
            while len(path)>1 and distance(pose,path[0])<.06 and abs(wrap(pose.yaw-path[0].yaw))<.15: path.pop(0)
            left,right=track(pose,path,odom.v,cfg)
            increment=cfg['acceleration_mps2']*dt
            left=max(last_left-increment,min(last_left+increment,left))
            right=max(last_right-increment,min(last_right+increment,right))
            latest=link.get(); current=clock(); current_scan=lidar.get()
            if (not guard.allowed(latest,current_scan,current,localized,False)
                or (current_scan.generation,current_scan.acquired)!=last_scan
                or not safe_command(pose,left,right,current_scan,grid,cfg,odom.v,
                                    (live.mask(current),) if not exploring else (),scan_pose=scan_pose,measured_w=odom.w)):
                left=right=0.; path=[]; state='BLOCKED'
            else: state='EXPLORING' if exploring else 'NAVIGATING'
            # Collision/localization work can consume the remaining age budget.
            # Recheck immediately before transmitting; a new unprocessed scan also stops us.
            current=clock(); newest_scan=lidar.get(); newest_packet=link.get()
            if (not guard.allowed(latest,current_scan,current,localized,False)
                or not newest_scan or (newest_scan.generation,newest_scan.acquired)!=last_scan
                or not newest_packet or newest_packet.epoch!=pkt.epoch or newest_packet.fault
                or newest_packet.mode!=2 or current-newest_packet.received>cfg['telemetry_max_age_s']):
                left=right=0.; path=[]; state='RECOVERING_STOPPED'
            latest=newest_packet
            if latest: link.send(latest,left,right)
            last_left,last_right=left,right
            if log:
                row={'t':current,'state':state,'pose':pose.array().tolist(),'wheel_command_mps':[left,right],
                     'goal_index':goal_index,'map_identity':map_identity(grid),'localized':localized}
                log.write(json.dumps(row)+'\n'); log.flush()
            sleep(.05)
    except (Exception,KeyboardInterrupt) as exc:
        result=RunResult('partial' if exploring else 'fault',str(exc) or 'Operator interrupted',goal_index,
                         explorer.coverage() if exploring else {})
    finally:
        packet=getattr(link,'latest',None)
        if packet:
            try: link.send(packet)
            except Exception: pass
        try:
            if exploring:
                save_map(output,mapper.grid,mapper.keyframes,{'status':result.status,'reason':result.reason,
                         'coverage':result.coverage,'loop_closures':mapper.loop_closures})
                export_map(output,mapper.grid)
        finally:
            try: lidar.close()
            finally: link.close()
    return result
