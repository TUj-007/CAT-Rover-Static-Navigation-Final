"""Hardware workers never fabricate readings or keep a motion heartbeat alive."""
import math
import threading
import time
import numpy as np
from .models import Scan
from .protocol import frame, unframe, parse_telemetry, velocity_command, delta32


def in_sector(deg,sector):
    a,b=sector
    return a<=deg<=b if a<=b else deg>=a or deg<=b


def convert_scan(raw,cfg,acquired,completed,generation):
    points=[]; angles=[]
    for quality,degrees,mm in raw:
        if quality<=0 or not math.isfinite(degrees) or not math.isfinite(mm): continue
        r=mm/1000
        if not cfg['lidar_min_m']<=r<=cfg['lidar_max_m']: continue
        deg=(cfg['lidar_angle_sign']*degrees+cfg['lidar_yaw_deg'])%360
        if any(in_sector(deg,s) for s in cfg['lidar_occlusion_sectors_deg']): continue
        a=math.radians(deg)
        px=cfg['lidar_x_m']+r*math.cos(a); py=cfg['lidar_y_m']+r*math.sin(a)
        if cfg['lidar_self_filter'] and abs(px)<=cfg['body_length_m']/2 and abs(py)<=cfg['body_width_m']/2:
            continue
        points.append((px,py)); angles.append(a)
    order=np.argsort(angles)
    return Scan(np.asarray(points,dtype=float).reshape(-1,2)[order],np.asarray(angles)[order],acquired,completed,generation)


def scan_usable(scan,cfg,now):
    if scan is None or not 0<=now-scan.acquired<=cfg['lidar_max_age_s']: return False
    if (len(scan.points)<cfg['lidar_min_points'] or len(scan.angles)!=len(scan.points)
        or not np.isfinite(scan.points).all() or not np.isfinite(scan.angles).all()
        or scan.completed<scan.acquired or scan.completed>now+.01): return False
    a=np.sort(scan.angles%(2*math.pi))
    gap=np.max(np.diff(np.r_[a,a[0]+2*math.pi]))
    return gap<=math.radians(cfg['lidar_max_gap_deg'])+1e-8


class MegaLink:
    def __init__(self,port):
        import serial
        self.ser=serial.Serial(port,115200,timeout=.05,write_timeout=.10,exclusive=True)
        self.latest=None; self.error=None; self.version=None; self.seq=0
        self.lock=threading.Lock(); self.write_lock=threading.Lock(); self.done=threading.Event()
        self.thread=threading.Thread(target=self._read,daemon=True); self.thread.start()

    def _read(self):
        pending=bytearray(); dropping=False
        try:
            while not self.done.is_set():
                for byte in self.ser.read(max(1,min(self.ser.in_waiting,1024))):
                    if byte==10:
                        if not dropping:
                            try:
                                raw=bytes(pending)+b'\n'; parts=unframe(raw)
                                if parts[0]=='V':
                                    if parts!=['V','2','1','1']: raise RuntimeError('Incompatible firmware identity')
                                    self.version={'protocol':2,'encoder_edges':1,'motor_layout':1}
                                else:
                                    pkt=parse_telemetry(raw,time.monotonic())
                                    with self.lock:
                                        if self.latest and pkt.ms==self.latest.ms:
                                            pass # Duplicates must not refresh telemetry age.
                                        else: self.latest=pkt
                            except (ValueError,UnicodeError): pass
                        pending.clear(); dropping=False
                    elif len(pending)>=179: dropping=True
                    elif not dropping: pending.append(byte)
        except Exception as e: self.error=e

    def identity(self):
        if self.error: raise RuntimeError(str(self.error))
        if self.version is None:
            with self.write_lock: self.ser.write(frame('Q'))
        return self.version

    def get(self):
        if self.error: raise RuntimeError(f'Mega serial: {self.error}')
        with self.lock: return self.latest

    def send(self,p,left=0,right=0):
        self.seq=(self.seq+1)&0xffffffff
        data=velocity_command(p,self.seq,left,right)
        with self.write_lock:
            if self.ser.write(data)!=len(data): raise RuntimeError('Incomplete Mega command')

    def close(self):
        try:
            if self.latest: self.send(self.latest)
        finally:
            self.done.set(); self.thread.join(timeout=.4); self.ser.close()


class Lidar:
    def __init__(self,cfg):
        self.cfg=cfg; self.latest=None; self.error=None; self.generation=0
        self.lock=threading.Lock(); self.done=threading.Event()
        self.thread=threading.Thread(target=self._read,daemon=True); self.thread.start()

    def invalidate(self):
        with self.lock: self.latest=None; self.generation+=1

    def _read(self):
        device=None; failures=0
        try:
            from rplidar import RPLidar
            while not self.done.is_set():
                try:
                    device=RPLidar(self.cfg['lidar_port'],baudrate=self.cfg['lidar_baud'],timeout=.2)
                    if device.get_health()[0]!='Good': raise RuntimeError('LiDAR health is not Good')
                    start=time.monotonic()
                    for raw in device.iter_scans(max_buf_meas=0,min_len=5):
                        if self.done.is_set(): break
                        # API tied to pinned 0.9.2; never silently discard a backlog as fresh.
                        if device._serial_port.in_waiting>1000: raise RuntimeError('LiDAR processing backlog')
                        now=time.monotonic()
                        scan=convert_scan(raw,self.cfg,start,now,self.generation); start=now
                        with self.lock: self.latest=scan
                        failures=0
                    if self.done.is_set(): break
                    raise RuntimeError('LiDAR scan stream ended')
                except Exception as e:
                    self.invalidate(); failures+=1
                    if failures>=5: self.error=e; break
                    if self.done.wait(.2): break
                finally:
                    if device:
                        for name in ('stop','stop_motor','disconnect'):
                            try: getattr(device,name)()
                            except Exception: pass
                        device=None
        except Exception as e: self.invalidate(); self.error=e

    def get(self):
        if self.error: raise RuntimeError(f'LiDAR failed: {self.error}')
        with self.lock: return self.latest

    def close(self):
        self.done.set(); self.thread.join(timeout=1.2)
