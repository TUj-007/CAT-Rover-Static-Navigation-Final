import unittest
from cat_rover_nav.config import defaults
from cat_rover_nav.grid import OccupancyGrid
from cat_rover_nav.storage import map_identity
from cat_rover_nav.routes import validate_route

class Routes(unittest.TestCase):
    def setUp(self):
        self.g=OccupancyGrid(defaults()); self.g.observed[:]=True; self.g.logodds[:]=-3; self.g.changed()
        self.route={'version':1,'map_identity':map_identity(self.g),'start':[0,0,0],
                    'goals':[{'pose':[.5,0,0],'dwell_s':0}]}
    def test_ordered_route_accepted(self):
        self.assertEqual(validate_route(self.route,self.g)['goals'][0]['pose'],[.5,0,0])
    def test_invalid_identity_numbers_and_blocked_pose_rejected(self):
        self.route['map_identity']='wrong'
        with self.assertRaises(ValueError): validate_route(self.route,self.g)
        self.route['map_identity']=map_identity(self.g); self.route['start'][0]=float('nan')
        with self.assertRaises(ValueError): validate_route(self.route,self.g)
        self.route['start']=[0,0,0]; self.g.logodds[60,70]=3; self.g.changed()
        self.route['map_identity']=map_identity(self.g)
        with self.assertRaises(ValueError): validate_route(self.route,self.g)
    def test_empty_route_and_unbounded_dwell_rejected(self):
        self.route['goals']=[]
        with self.assertRaises(ValueError): validate_route(self.route,self.g)
        self.route['goals']=[{'pose':[.5,0,0],'dwell_s':-1}]
        with self.assertRaises(ValueError): validate_route(self.route,self.g)

if __name__=='__main__': unittest.main()
