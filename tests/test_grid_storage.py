import tempfile
import json
import unittest
from pathlib import Path
import numpy as np
from cat_rover_nav.config import defaults
from cat_rover_nav.models import Pose, Scan
from cat_rover_nav.grid import OccupancyGrid
from cat_rover_nav.storage import save_map, load_map, map_identity


def ring(radius=2):
    a=np.linspace(0,2*np.pi,360,endpoint=False)
    return Scan(np.c_[radius*np.cos(a),radius*np.sin(a)],a,0,.1,0)


class Mapping(unittest.TestCase):
    def test_observed_free_and_hit_but_unknown_outside(self):
        g=OccupancyGrid(defaults()); g.update(Pose(),ring())
        self.assertEqual(g.value(.5,0),0)
        self.assertEqual(g.value(2,0),1)
        self.assertEqual(g.value(2.5,0),-1)
        self.assertTrue(g.footprint_clear(Pose(),defaults()))

    def test_self_footprint_does_not_clear_a_safety_bubble(self):
        g=OccupancyGrid(defaults()); g.certify_start(Pose())
        self.assertEqual(g.value(.40,0),-1)
        s=Scan(np.array([[.7,0]]),np.array([0.]),0,.1,0)
        g.update(Pose(),s)
        self.assertEqual(g.value(.5,.3),-1)

    def test_noise_can_be_cleared_and_outside_reported(self):
        g=OccupancyGrid(defaults()); s=Scan(np.array([[1.,0]]),np.array([0.]),0,.1,0)
        g.update(Pose(),s); self.assertEqual(g.value(1,0),1)
        for _ in range(5): g.update(Pose(),Scan(np.array([[2.,0]]),np.array([0.]),0,.1,0))
        self.assertEqual(g.value(1,0),0)
        g.update(Pose(),ring(4)); self.assertGreater(g.clipped_rays,0)

    def test_atomic_roundtrip_identity_and_bad_version(self):
        g=OccupancyGrid(defaults()); g.update(Pose(),ring())
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'map.npz'
            save_map(p,g,[],{'status':'complete'})
            h,frames,meta=load_map(p)
            self.assertEqual(map_identity(h),map_identity(g))
            self.assertEqual(meta['status'],'complete'); self.assertEqual(frames,[])
            self.assertTrue(np.array_equal(g.observed,h.observed))
            np.savez_compressed(p,metadata=np.array('{"version":99}'))
            with self.assertRaises(ValueError): load_map(p)

    def test_changed_evidence_and_nonfinite_metadata_rejected(self):
        g=OccupancyGrid(defaults()); g.update(Pose(),ring())
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'map.npz'; save_map(p,g,[],{'status':'complete'})
            with np.load(p,allow_pickle=False) as archive: arrays={k:archive[k] for k in archive.files}
            arrays['logodds']=arrays['logodds'].copy(); arrays['logodds'][60,60]-=.1
            np.savez_compressed(p,**arrays)
            with self.assertRaises(ValueError): load_map(p)
            save_map(p,g,[],{'status':'complete'})
            with np.load(p,allow_pickle=False) as archive: arrays={k:archive[k] for k in archive.files}
            meta=json.loads(str(arrays['metadata'])); meta['coverage']={'observed_m2':float('nan')}
            arrays['metadata']=np.array(json.dumps(meta)); np.savez_compressed(p,**arrays)
            with self.assertRaises(ValueError): load_map(p)


if __name__=='__main__': unittest.main()
