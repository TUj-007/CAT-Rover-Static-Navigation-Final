import math
import unittest
import numpy as np
from cat_rover_nav.config import defaults
from cat_rover_nav.models import Pose,Scan
from cat_rover_nav.grid import OccupancyGrid
from cat_rover_nav.planning import Planner
from cat_rover_nav.motion import safe_command,track,LiveObstacleLayer


def clear_grid():
    c=defaults(); g=OccupancyGrid(c); g.observed[:]=True; g.logodds[:]=-2; g.changed()
    return c,g


class Planning(unittest.TestCase):
    def test_static_detour_and_unknown_block(self):
        c,g=clear_grid()
        for y in np.arange(-.35,.36,g.res):
            x,z=g.cell(.8,y); g.logodds[z,x]=3
        g.changed(); path=Planner(g,c).plan(Pose(),Pose(1.6,0,0))
        self.assertTrue(path); self.assertGreater(max(abs(p.y) for p in path),.5)
        self.assertTrue(all(g.footprint_clear(p,c) for p in path))
        blank=OccupancyGrid(c)
        self.assertEqual(Planner(blank,c).plan(Pose(),Pose(1,0,0)),[])

    def test_narrow_straight_clear_but_pivot_blocked(self):
        c,g=clear_grid()
        for y in (-.30,.30):
            for x in np.arange(-2,2,g.res):
                a,b=g.cell(x,y); g.logodds[b,a]=3
        g.changed()
        self.assertTrue(g.footprint_clear(Pose(),c))
        self.assertFalse(g.footprint_clear(Pose(0,0,math.pi/2),c))
        self.assertTrue(Planner(g,c).plan(Pose(-.5,0,0),Pose(.5,0,0)))
        scan=Scan(np.array([[0,.30],[0,-.30],[2,0]]),np.array([0,1,2]),0,.1)
        self.assertTrue(safe_command(Pose(),.08,.08,scan,g,c,0))
        self.assertFalse(safe_command(Pose(),-.1,.1,scan,g,c,0))

    def test_live_obstacle_and_measured_braking_veto(self):
        c,g=clear_grid()
        s=Scan(np.array([[.38,0]]),np.array([0.]),0,.1)
        self.assertFalse(safe_command(Pose(),.08,.08,s,g,c,.15))
        layer=LiveObstacleLayer(g,c); before=g.logodds.copy(); layer.update(s,Pose(),1)
        self.assertTrue(layer.mask(1)[g.cell(.38,0)[1],g.cell(.38,0)[0]])
        self.assertTrue(np.array_equal(before,g.logodds))
        self.assertFalse(layer.mask(4).any())

    def test_tracker_turn_sign_and_limits(self):
        c,g=clear_grid(); l,r=track(Pose(),[Pose(),Pose(0,1,math.pi/2)],0,c)
        self.assertLess(l,0); self.assertGreater(r,0)
        self.assertLessEqual(max(abs(l),abs(r)),c['max_wheel_mps'])

    def test_full_measured_braking_travel_and_acquisition_pose(self):
        c,g=clear_grid(); scan=Scan(np.array([[.60,0]]),np.array([0.]),0,.1)
        self.assertFalse(safe_command(Pose(),.05,.05,scan,g,c,.25))
        scan=Scan(np.array([[.41,0]]),np.array([0.]),0,.1)
        self.assertFalse(safe_command(Pose(.1,0,0),.05,.05,scan,g,c,0,scan_pose=Pose()))

    def test_turn_command_includes_residual_forward_motion(self):
        c,g=clear_grid(); scan=Scan(np.array([[.40,.30]]),np.array([.5]),0,.1)
        self.assertFalse(safe_command(Pose(),-.10,.10,scan,g,c,.20))


if __name__=='__main__': unittest.main()
