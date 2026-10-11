import contextlib
import io
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
from cat_rover_nav.__main__ import main
from cat_rover_nav.config import defaults
from cat_rover_nav.grid import OccupancyGrid
from cat_rover_nav.storage import save_map

class CLI(unittest.TestCase):
    def test_invalid_route_and_missing_confirmation_never_open_hardware(self):
        with tempfile.TemporaryDirectory() as d:
            d=Path(d); c=defaults(); cp=d/'config.json'; cp.write_text(json.dumps(c))
            g=OccupancyGrid(c); g.observed[:]=True; g.logodds[:]=-2; g.changed(); mp=d/'map.npz'
            save_map(mp,g,[],{'status':'complete'})
            rp=d/'route.json'; rp.write_text(json.dumps({'version':1,'map_identity':'wrong','start':[0,0,0],
                                                       'goals':[{'pose':[1,0,0],'dwell_s':0}]}))
            argv=['rover','navigate','--config',str(cp),'--map',str(mp),'--route',str(rp),'--allow-motion']
            for confirmed in (False,True):
                c['hardware_confirmed']=c['calibration_confirmed']=confirmed; cp.write_text(json.dumps(c))
                with patch('sys.argv',argv),patch('cat_rover_nav.__main__.MegaLink') as device,contextlib.redirect_stderr(io.StringIO()):
                    self.assertEqual(main(),1); device.assert_not_called()
