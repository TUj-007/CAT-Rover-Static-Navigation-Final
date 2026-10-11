import unittest
import numpy as np
from cat_rover_nav.config import defaults
from cat_rover_nav.grid import OccupancyGrid
from cat_rover_nav.models import Pose
from cat_rover_nav.planning import Planner
from cat_rover_nav.exploration import Explorer


class Exploration(unittest.TestCase):
    def test_frontier_observation_target_is_known_and_clear(self):
        c=defaults(); g=OccupancyGrid(c)
        g.observed[35:85,35:75]=True; g.logodds[g.observed]=-2; g.changed()
        e=Explorer(g,c); target=e.choose(Pose(),Planner(g,c))
        self.assertIsNotNone(target)
        self.assertTrue(g.footprint_clear(target,c))
        self.assertGreater(e.coverage()['frontier_cells'],0)

    def test_complete_and_failed_are_distinct(self):
        c=defaults(); g=OccupancyGrid(c); g.observed[:]=True; g.logodds[:]=-2; g.changed()
        e=Explorer(g,c)
        self.assertIsNone(e.choose(Pose(),Planner(g,c)))
        self.assertEqual(e.completion()[0],'complete')
        g.observed[60:80,90:110]=False; g.changed()
        target=e.choose(Pose(),Planner(g,c)); self.assertIsNotNone(target)
        for _ in range(c['frontier_max_failures']): e.report_target_failure(target,'no progress')
        self.assertEqual(e.completion()[0],'partial')

    def test_partial_when_survey_exceeds_map(self):
        c=defaults(); g=OccupancyGrid(c); g.clipped_rays=3
        e=Explorer(g,c)
        self.assertEqual(e.completion()[0],'partial')

    def test_all_candidate_regions_examined_and_exhaustion_partial(self):
        c=defaults(); g=OccupancyGrid(c); g.observed[:]=True; g.logodds[:]=-2; g.changed()
        e=Explorer(g,c); mask=np.zeros((g.n,g.n),bool)
        for y in (25,45,65,85):
            for x in (25,85): mask[y,x:x+4]=True
        e.frontiers=lambda:mask; e.visible_gain=lambda pose:100
        class LaterPath:
            calls=0
            def plan(self,a,b):
                self.calls+=1; return [a,b] if self.calls==7 else []
        planner=LaterPath(); self.assertIsNotNone(e.choose(Pose(),planner))
        class NoPath:
            def plan(self,a,b): return []
        self.assertIsNone(e.choose(Pose(),NoPath()))
        self.assertEqual(e.completion()[0],'partial')

    def test_uncertain_observed_wall_is_not_unexplored_space(self):
        c=defaults(); g=OccupancyGrid(c); g.observed[:]=True; g.logodds[:]=-2
        g.logodds[80:82,40:80]=0; g.changed(); e=Explorer(g,c)
        self.assertEqual(e.frontiers().sum(),0)

    def test_large_ring_uses_actual_frontier_not_group_centroid(self):
        c=defaults(); c['map_size_m']=20.; c['grid_resolution_m']=.1; c['survey_bounds']=[-9.9,9.9,-9.9,9.9]
        g=OccupancyGrid(c); g.observed[30:170,30:170]=True; g.logodds[g.observed]=-2; g.changed()
        e=Explorer(g,c)
        class ClearPath:
            def plan(self,a,b): return [a,b]
        target=e.choose(Pose(),ClearPath()); self.assertIsNotNone(target)
        self.assertGreater(max(abs(target.x),abs(target.y)),5.)


if __name__=='__main__': unittest.main()
