import math
import unittest
import numpy as np
from cat_rover_nav.models import Pose,Scan
from cat_rover_nav.config import defaults
from cat_rover_nav.geometry import transform,inverse
from cat_rover_nav.grid import OccupancyGrid
from cat_rover_nav.localization import ScanMatcher,Localizer
from cat_rover_nav.slam import Mapper


def walls():
    t=np.linspace(-1.5,1.5,100)
    return np.vstack([np.c_[t,np.full(100,1.5)],np.c_[t,np.full(100,-1.5)],
                      np.c_[np.full(100,2.),t],np.c_[np.full(100,-1.5),t]])


def observation(points,pose):
    local=transform(points,inverse(pose)); a=np.arctan2(local[:,1],local[:,0])%(2*np.pi)
    idx=np.argsort(a)
    return Scan(local[idx],a[idx],0,.1)


def occupied_fixture(points):
    g=OccupancyGrid(defaults()); g.observed[:]=True; g.logodds[:]=-2
    for x,y in points:
        cx,cy=g.cell(x,y); g.logodds[cy,cx]=3
    g.changed(); return g


class Localization(unittest.TestCase):
    def test_repeated_stationary_scans_do_not_walk_the_map_by_cell_centres(self):
        from cat_rover_nav.simulation import World
        c=defaults(); w=World(c); m=Mapper(c)
        for i in range(20):
            w.t=i*.2; result=m.observe(w.scan_get(),Pose())
            self.assertTrue(result.accepted,result.reason)
        self.assertLess(math.hypot(result.pose.x,result.pose.y),.015)

    def test_corrects_translation_yaw_from_real_geometry(self):
        p=walls(); g=occupied_fixture(p); true=Pose(.3,.2,.12)
        result=ScanMatcher(defaults()).match(observation(p,true),Pose(.38,.14,.18),g)
        self.assertTrue(result.accepted,result.reason)
        self.assertLess(math.hypot(result.pose.x-.3,result.pose.y-.2),.065)
        self.assertLess(abs(result.pose.yaw-.12),.03)

    def test_bad_pose_and_low_overlap_rejected(self):
        p=walls(); g=occupied_fixture(p)
        result=ScanMatcher(defaults()).match(observation(p,Pose()),Pose(1,1,1),g)
        self.assertFalse(result.accepted)
        sparse=Scan(np.array([[.3,.3]]),np.array([0.]),0,.1)
        self.assertFalse(ScanMatcher(defaults()).match(sparse,Pose(),g).accepted)

    def test_corridor_and_circle_degeneracy_rejected(self):
        y=np.linspace(-2,2,150)
        p=np.vstack([np.c_[np.full(150,-1.),y],np.c_[np.full(150,1.),y]])
        self.assertFalse(ScanMatcher(defaults()).match(observation(p,Pose()),Pose(),occupied_fixture(p)).accepted)
        a=np.linspace(0,2*np.pi,360,endpoint=False); p=np.c_[2*np.cos(a),2*np.sin(a)]
        self.assertFalse(ScanMatcher(defaults()).match(observation(p,Pose()),Pose(),occupied_fixture(p)).accepted)

    def test_localizer_does_not_mutate_map(self):
        p=walls(); g=occupied_fixture(p); before=g.logodds.copy()
        loc=Localizer(g,defaults(),Pose(.3,.2,.12))
        r=loc.observe(observation(p,Pose(.3,.2,.12)),Pose())
        self.assertTrue(r.accepted,r.reason); self.assertTrue(np.array_equal(before,g.logodds))

    def test_mapper_rejects_bad_scan_without_painting(self):
        m=Mapper(defaults()); m.observe(observation(walls(),Pose()),Pose())
        rev=m.grid.revision
        r=m.observe(Scan(np.array([[.2,.2]]),np.array([0.]),0,.1),Pose(2,2,0))
        self.assertFalse(r.accepted); self.assertEqual(m.grid.revision,rev)

    def test_pose_graph_rebuild_reduces_constraint_error_and_rejects_false_loop(self):
        m=Mapper(defaults()); points=walls()
        m.keyframes=[{'pose':Pose(x*1.04,x*.06,x*.03),'odom':Pose(x*1.04,0,0),
                      'scan':observation(points,Pose(x,0,0))} for x in (0,.5,1.)]
        m.edges=[(0,1,Pose(.5,0,0),.06),(1,2,Pose(.5,0,0),.06),(0,2,Pose(1,0,0),.04)]
        self.assertTrue(m.optimize()); self.assertLess(abs(m.keyframes[-1]['pose'].x-1),.005)
        self.assertLess(abs(m.keyframes[-1]['pose'].y),.005)
        self.assertEqual(m.grid.value(2,0),1)
        old=[k['pose'] for k in m.keyframes]; identity=id(m.grid)
        m.edges.append((0,2,Pose(5,2,1),.001))
        self.assertFalse(m.optimize()); self.assertEqual(old,[k['pose'] for k in m.keyframes])
        self.assertEqual(identity,id(m.grid))


if __name__=='__main__': unittest.main()
