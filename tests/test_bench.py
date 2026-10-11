import unittest
from cat_rover_nav.bench import validate_pulse

class Bench(unittest.TestCase):
    def test_bounded_finite_pulse(self):
        validate_pulse(.05,-.05,2)
        for values in ((.11,0,2),(0,0,6),(float('nan'),0,1),(0,0,-1)):
            with self.assertRaises(ValueError): validate_pulse(*values)
