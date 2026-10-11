import hashlib
import json
import os
from pathlib import Path
import tempfile
import numpy as np
from .grid import OccupancyGrid
from .config import validate
from .models import Pose, Scan


def map_identity(grid):
    h=hashlib.sha256()
    h.update(np.asarray([grid.n,grid.res,grid.origin],dtype='<f8').tobytes())
    h.update(np.asarray(grid.logodds,dtype='<f4').tobytes()); h.update(grid.observed.tobytes())
    return h.hexdigest()


def save_map(path,grid,keyframes,metadata):
    path=Path(path); path.parent.mkdir(parents=True,exist_ok=True)
    meta=dict(metadata,version=1,identity=map_identity(grid),config=grid.cfg,
              frame='x-forward y-left yaw-CCW metres radians',clipped_rays=grid.clipped_rays)
    points=[]; angles=[]; poses=[]; odoms=[]; offsets=[0]
    for k in keyframes:
        points.append(k['scan'].points); angles.append(k['scan'].angles)
        poses.append(k['pose'].array()); odoms.append(k['odom'].array())
        offsets.append(offsets[-1]+len(k['scan'].points))
    text=json.dumps(meta,allow_nan=False)
    temp=None
    try:
        with tempfile.NamedTemporaryFile(dir=path.parent,suffix='.npz',delete=False) as f:
            temp=Path(f.name)
            np.savez_compressed(f,metadata=np.array(text),logodds=grid.logodds,observed=grid.observed,
                poses=np.asarray(poses).reshape(-1,3),odoms=np.asarray(odoms).reshape(-1,3),
                points=np.concatenate(points) if points else np.empty((0,2)),
                angles=np.concatenate(angles) if angles else np.empty(0),offsets=np.array(offsets))
            f.flush(); os.fsync(f.fileno())
        os.replace(temp,path); temp=None
    finally:
        if temp and temp.exists(): temp.unlink()


def load_map(path):
    with np.load(path,allow_pickle=False) as data:
        def bad_constant(value): raise ValueError('Nonfinite map metadata')
        meta=json.loads(str(data['metadata']),parse_constant=bad_constant)
        if meta.get('version')!=1: raise ValueError('Unsupported map version')
        cfg=validate(meta['config']); g=OccupancyGrid(cfg)
        lo=data['logodds']; observed=data['observed']
        if (lo.shape!=(g.n,g.n) or observed.shape!=lo.shape or observed.dtype!=np.bool_
            or not np.isfinite(lo).all() or np.any(np.abs(lo)>4)):
            raise ValueError('Corrupt map shape/evidence')
        g.logodds=lo.astype(np.float32); g.observed=observed.astype(bool)
        g.clipped_rays=int(meta.get('clipped_rays',0)); g.changed()
        if map_identity(g)!=meta['identity']: raise ValueError('Map identity mismatch/corruption')
        points=data['points']; angles=data['angles']; poses=data['poses']; odoms=data['odoms']; off=data['offsets']
        if (off.ndim!=1 or len(off)<1 or off.dtype.kind not in 'iu'
            or angles.ndim!=1 or points.ndim!=2 or points.shape[1]!=2 or len(angles)!=len(points) or poses.shape!=odoms.shape
            or poses.shape!=(len(off)-1,3) or off[0]!=0 or off[-1]!=len(points)
            or np.any(off>len(points)) or np.any(np.diff(off.astype(np.int64))<0)
            or any(not np.isfinite(a).all() for a in (points,angles,poses,odoms))):
            raise ValueError('Corrupt map keyframes')
        frames=[{'pose':Pose(*p),'odom':Pose(*o),'scan':Scan(points[off[i]:off[i+1]].copy(),
                    angles[off[i]:off[i+1]].copy(),0,0)} for i,(p,o) in enumerate(zip(poses,odoms))]
        return g,frames,meta


def export_map(path,grid):
    path=Path(path); state=grid.state()
    image=np.where(state==0,254,np.where(state==1,0,205)).astype(np.uint8)[::-1]
    path.with_suffix('.pgm').write_bytes(f'P5\n{grid.n} {grid.n}\n255\n'.encode()+image.tobytes())
    path.with_suffix('.json').write_text(json.dumps({'identity':map_identity(grid),'resolution':grid.res,
        'origin':[grid.origin,grid.origin,0],'width':grid.n,'height':grid.n,
        'image':path.with_suffix('.pgm').name,'frame':'x-forward y-left yaw-CCW'},indent=2))
