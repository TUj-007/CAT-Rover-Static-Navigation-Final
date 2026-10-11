"""Explicit raised-wheel calibration pulse. Never used by autonomous navigation."""
import argparse
import math
import signal
import time
from .config import load_config
from .devices import MegaLink
from .protocol import delta32


def validate_pulse(left,right,seconds):
    if not all(math.isfinite(v) for v in (left,right,seconds)) or max(abs(left),abs(right))>.10 or not 0<seconds<=5:
        raise ValueError('Bench pulse requires finite speeds <=0.10 m/s and duration (0,5] seconds')


def main():
    p=argparse.ArgumentParser(description='Wheels-raised calibration only; both motors on each side share a speed target')
    p.add_argument('--config',required=True); p.add_argument('--left',type=float,default=0)
    p.add_argument('--right',type=float,default=0); p.add_argument('--seconds',type=float,default=2)
    p.add_argument('--wheels-raised',action='store_true'); p.add_argument('--allow-motion',action='store_true')
    a=p.parse_args(); validate_pulse(a.left,a.right,a.seconds); c=load_config(a.config)
    if not a.wheels_raised or not a.allow_motion or not c['hardware_confirmed'] or not c['calibration_confirmed']:
        raise ValueError('Bench pulse requires confirmed config, --wheels-raised and --allow-motion')
    def stop(_sig,_frame): raise KeyboardInterrupt()
    signal.signal(signal.SIGTERM,stop)
    link=MegaLink(c['mega_port']); previous=None
    try:
        started=time.monotonic(); ready=None
        while True:
            now=time.monotonic(); identity=link.identity(); packet=link.get()
            if identity and identity!={'protocol':2,'encoder_edges':1,'motor_layout':1}: raise RuntimeError('Wrong firmware')
            if not packet or not identity:
                if now-started>c['startup_timeout_s']: raise RuntimeError('Telemetry startup timed out')
                time.sleep(.05);continue
            if now-packet.received>c['telemetry_max_age_s']: raise RuntimeError('Stale telemetry')
            if packet.fault not in (0,8): raise RuntimeError(f'Mega fault {packet.fault}')
            if ready is None:
                link.send(packet)
                if now-started>c['startup_timeout_s']: raise RuntimeError('Zero-handshake startup timed out')
                if packet.mode==2 and packet.fault==0 and now-started>=.25: ready=now
            elif now-ready>=a.seconds: break
            else:
                if packet.mode!=2 or packet.fault: raise RuntimeError('Controller session lost')
                link.send(packet,a.left,a.right)
            if previous and packet.ms!=previous.ms:
                dt=delta32(packet.ms,previous.ms)/1000
                if dt<=0 or dt>1: raise RuntimeError('Telemetry reset/discontinuity')
                speed=[delta32(x,y)*math.pi*d/n/dt for x,y,d,n in zip(packet.counts,previous.counts,c['wheel_diameter_m'],c['counts_per_rev'])]
                print('FL RL FR RR m/s:', ' '.join(f'{v:+.3f}' for v in speed),flush=True)
            previous=packet; time.sleep(.05)
    finally: link.close()


if __name__=='__main__':
    try: main()
    except (Exception,KeyboardInterrupt) as exc: raise SystemExit(str(exc) or 'Bench pulse interrupted; zero requested')
