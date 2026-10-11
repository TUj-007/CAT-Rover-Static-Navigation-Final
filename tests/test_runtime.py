import math
import tempfile
import unittest
from pathlib import Path
import numpy as np
from cat_rover_nav.config import defaults
from cat_rover_nav.models import Pose,Telemetry,Scan
from cat_rover_nav.grid import OccupancyGrid
from cat_rover_nav.runtime import CommandGuard,run_exploration,run_navigation


class Guarding(unittest.TestCase):
    def test_expensive_collision_check_cannot_send_a_stale_command(self):
        from unittest.mock import patch
        from cat_rover_nav.simulation import World,Sensor
        c=defaults(); c['mission_timeout_s']=1.; w=World(c)
        grid=OccupancyGrid(c); grid.update(Pose(),w.scan_get())
        route={'start':[0,0,0],'goals':[{'pose':[.5,0,0],'dwell_s':0}]}
        def slow_check(*args,**kwargs): w.tick(.4); return True
        with patch('cat_rover_nav.runtime.safe_command',side_effect=slow_check):
            r=run_navigation(c,(w,Sensor(w)),(grid,[],{}),route,clock=lambda:w.t,sleep=w.tick)
        self.assertEqual(r.status,'fault')
        self.assertTrue(all(l==r==0 for _,l,r in w.commands))

    def test_alignment_and_planning_failures_have_deadlines(self):
        from unittest.mock import patch
        from cat_rover_nav.simulation import World,Sensor
        for failure in ('alignment','planning'):
            c=defaults(); c['startup_timeout_s']=.2; c['mission_timeout_s']=.3; c['exploration_timeout_s']=.3
            w=World(c,obstacles=True)
            with tempfile.TemporaryDirectory() as d:
                if failure=='alignment':
                    with patch('cat_rover_nav.runtime.Odometry.pose_at',return_value=None):
                        r=run_exploration(c,(w,Sensor(w)),Path(d)/'m.npz',clock=lambda:w.t,sleep=w.tick)
                else:
                    class StuckJob:
                        done=False
                        def __init__(self,*args): pass
                    with patch('cat_rover_nav.runtime.PlanJob',StuckJob):
                        r=run_exploration(c,(w,Sensor(w)),Path(d)/'m.npz',clock=lambda:w.t,sleep=w.tick)
                self.assertEqual(r.status,'partial'); self.assertLess(w.t,.5)
                self.assertTrue(all(l==r==0 for _,l,r in w.commands))

    def test_guard_rejects_expired_data_and_requires_zero(self):
        c=defaults(); guard=CommandGuard(c)
        p=Telemetry(100,1,2,0,(0,)*4,1)
        a=np.linspace(0,2*np.pi,360,endpoint=False)
        scan=Scan(np.c_[2*np.cos(a),2*np.sin(a)],a,.95,1)
        self.assertFalse(guard.allowed(p,scan,1,True,False))
        guard.note_zero(p)
        self.assertTrue(guard.allowed(p,scan,1.05,True,False))
        self.assertFalse(guard.allowed(p,scan,1.4,True,False))
        self.assertFalse(guard.allowed(p,scan,1.05,False,False))
        self.assertFalse(guard.allowed(p,scan,1.05,True,True))
        new=Telemetry(101,2,2,0,(0,)*4,1.05)
        self.assertFalse(guard.allowed(new,scan,1.05,True,False))

    def test_timeout_with_missing_scans_saves_partial_and_zero(self):
        c=defaults(); c['startup_timeout_s']=.3
        clock=[0.]; sent=[]
        class Link:
            latest=None
            def get(self):
                self.latest=Telemetry(round(clock[0]*1000),1,2,0,(0,)*4,clock[0]); return self.latest
            def identity(self): return {'protocol':2,'encoder_edges':1,'motor_layout':1}
            def send(self,p,l=0,r=0): sent.append((l,r))
            def close(self): pass
        class Sensor:
            def get(self): return None
            def close(self): pass
        def sleep(dt): clock[0]+=dt
        with tempfile.TemporaryDirectory() as d:
            result=run_exploration(c,(Link(),Sensor()),Path(d)/'map.npz',clock=lambda:clock[0],sleep=sleep)
            self.assertEqual(result.status,'partial')
            self.assertTrue((Path(d)/'map.npz').exists())
            self.assertTrue(sent); self.assertTrue(all(x==(0,0) for x in sent))

    def test_identity_mismatch_prohibits_motion(self):
        c=defaults(); clock=[0.]; sent=[]
        class Link:
            latest=Telemetry(1,1,2,0,(0,)*4,0)
            def get(self): return self.latest
            def identity(self): return {'protocol':1}
            def send(self,p,l=0,r=0): sent.append((l,r))
            def close(self): pass
        class Sensor:
            def get(self): return None
            def close(self): pass
        with tempfile.TemporaryDirectory() as d:
            result=run_exploration(c,(Link(),Sensor()),Path(d)/'map.npz',clock=lambda:0,sleep=lambda _:None)
            self.assertEqual(result.status,'partial'); self.assertIn('firmware',result.reason.lower())
            self.assertTrue(all(pair==(0,0) for pair in sent))


if __name__=='__main__': unittest.main()
