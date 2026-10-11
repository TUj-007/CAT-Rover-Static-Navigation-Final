import json
import math
import tempfile
import unittest
from pathlib import Path

from cat_rover_nav.config import defaults, validate, load_config
from cat_rover_nav.geometry import compose, inverse, transform
from cat_rover_nav.models import Pose, Telemetry
from cat_rover_nav.protocol import frame, unframe, parse_telemetry, velocity_command, delta32


class Contracts(unittest.TestCase):
    def test_crc_and_bounds(self):
        self.assertEqual(frame('123456789'), b'123456789*29B1\n')
        p = parse_telemetry(frame('T,100,7,2,0,0,4294967295,0,0,1500,1500,2000,2000'), 1.0)
        self.assertEqual(p.counts, (0, 4294967295, 0, 0))
        self.assertEqual(unframe(velocity_command(p, 3, -.1, .2)), ['C','7','3','100','-100','200'])
        for raw in (b'T,1*0000\n', b'abc*zzzz\n', frame('hi')[:-1]):
            with self.assertRaises(ValueError): unframe(raw)
        with self.assertRaises(ValueError): velocity_command(p, 1, math.nan, 0)
        with self.assertRaises(ValueError): velocity_command(p, 1, .301, 0)
        with self.assertRaises(ValueError): parse_telemetry(frame('T,1,1,3,0,0,0,0,0,0,0,0,0'),0)
        self.assertEqual(delta32(2, 0xffffffff), 3)
        self.assertEqual(delta32(0xffffffff, 2), -3)

    def test_config_rejects_unsafe_types_and_bounds(self):
        for key,value in [('cruise_mps',True),('cruise_mps','0.1'),('cruise_mps',float('nan')),
                          ('max_wheel_mps',.31),('survey_bounds',[-4,4,-1,1]),
                          ('wheel_diameter_m',[.1]),('telemetry_max_age_s',1)]:
            c=defaults(); c[key]=value
            with self.subTest(key=key,value=value), self.assertRaises(ValueError): validate(c)
        c=defaults(); c['lidar_port']=c['mega_port']
        with self.assertRaises(ValueError): validate(c)
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'config.json'; path.write_text(json.dumps({'nonsense':1}))
            with self.assertRaises(ValueError): load_config(path)

    def test_coordinate_conventions(self):
        p=Pose(1,2,math.pi/2)
        q=compose(p,Pose(1,0,0))
        self.assertAlmostEqual(q.x,1); self.assertAlmostEqual(q.y,3)
        z=compose(p,inverse(p))
        self.assertAlmostEqual(z.x,0); self.assertAlmostEqual(z.y,0); self.assertAlmostEqual(z.yaw,0)
        self.assertAlmostEqual(transform([[1,0]],p)[0,1],3)


if __name__=='__main__': unittest.main()
