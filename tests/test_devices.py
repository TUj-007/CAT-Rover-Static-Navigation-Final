import math
import unittest
import numpy as np
from cat_rover_nav.config import defaults
from cat_rover_nav.models import Telemetry
from cat_rover_nav.devices import convert_scan, scan_usable, Lidar
from cat_rover_nav.odometry import Odometry


class Sensors(unittest.TestCase):
    def test_coverage_and_true_close_obstacles(self):
        c=defaults(); c['lidar_angle_sign']=1
        raw=[(15,a,2000) for a in range(360)]
        raw[0]=(15,0,310)
        s=convert_scan(raw,c,1,1.1,0)
        self.assertTrue(scan_usable(s,c,1.15))
        self.assertAlmostEqual(s.points[0,0],.31)
        self.assertFalse(scan_usable(s,c,2))
        self.assertFalse(scan_usable(convert_scan(raw[:180],c,1,1.1,0),c,1.1))
        c['lidar_self_filter']=True
        raw=[(15,a,100) for a in range(360)]
        self.assertFalse(scan_usable(convert_scan(raw,c,1,1.1,0),c,1.1))

    def test_mask_does_not_create_free_rays(self):
        c=defaults(); c['lidar_angle_sign']=1; c['lidar_occlusion_sectors_deg']=[[350,10]]
        s=convert_scan([(15,a,2000) for a in range(360)],c,0,.1,0)
        self.assertEqual(len(s.points),339)
        self.assertFalse(scan_usable(s,c,.1))

    def test_recovery_invalidates_cached_scan(self):
        sensor=Lidar.__new__(Lidar)
        import threading
        sensor.lock=threading.Lock(); sensor.latest=object(); sensor.generation=0
        sensor.invalidate()
        self.assertIsNone(sensor.latest); self.assertEqual(sensor.generation,1)

    def test_signed_counts_duplicates_reset_and_history(self):
        c=defaults(); o=Odometry(c)
        packet=lambda ms,n,t: Telemetry(ms,1,2,0,(n,)*4,t)
        o.update(packet(100,0,1)); o.update(packet(200,60,1.1))
        self.assertAlmostEqual(o.pose.x,math.pi*.01)
        self.assertAlmostEqual(o.pose_at(1.05).x,math.pi*.005)
        self.assertFalse(o.update(packet(200,60,1.2)))
        o.update(packet(300,0,1.2)); self.assertAlmostEqual(o.pose.x,0)
        o.update(packet(400,0xffffffc4,1.3)); self.assertLess(o.pose.x,0)
        with self.assertRaises(RuntimeError): o.update(packet(10,0,1.4))
        self.assertIsNone(o.pose_at(.5))

    def test_encoder_jump_rejected(self):
        o=Odometry(defaults())
        o.update(Telemetry(1,1,2,0,(0,)*4,1))
        with self.assertRaises(RuntimeError): o.update(Telemetry(51,1,2,0,(10000,)*4,1.05))


if __name__=='__main__': unittest.main()
