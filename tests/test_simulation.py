import tempfile
import unittest
from pathlib import Path
from cat_rover_nav.simulation import simulate

class Simulation(unittest.TestCase):
    def test_first_run_map_and_saved_map_ordered_navigation(self):
        with tempfile.TemporaryDirectory() as d:
            report=simulate(Path(d),scenario='room')
            self.assertEqual(report['survey']['status'],'complete',report)
            self.assertEqual(report['navigation']['status'],'complete',report)
            self.assertEqual(report['navigation']['stations_reached'],3)
            self.assertEqual(report['collisions'],0)
            self.assertLess(report['end_error_m'],.14)
            self.assertLess(report['end_heading_error_rad'],.15)
            self.assertTrue(report['static_map_unchanged'])
    def test_dropout_stops_and_saves_partial(self):
        with tempfile.TemporaryDirectory() as d:
            report=simulate(Path(d),scenario='dropout')
            self.assertEqual(report['survey']['status'],'partial',report)
            self.assertEqual(report['collisions'],0)
            self.assertTrue(report['final_command_zero'])

    def test_ambiguous_geometry_never_moves(self):
        with tempfile.TemporaryDirectory() as d:
            report=simulate(Path(d),scenario='ambiguous')
            self.assertEqual(report['survey']['status'],'partial',report)
            self.assertEqual(report['nonzero_survey_commands'],0)
            self.assertTrue(report['final_command_zero'])

    def test_obstacle_added_after_mapping_detoured_without_map_change(self):
        with tempfile.TemporaryDirectory() as d:
            report=simulate(Path(d),scenario='live-obstacle')
            self.assertEqual(report['navigation']['status'],'complete',report)
            self.assertEqual(report['collisions'],0)
            self.assertTrue(report['static_map_unchanged'])

if __name__=='__main__': unittest.main()
