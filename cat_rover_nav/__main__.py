import argparse
import json
import signal
import sys
import time
from dataclasses import asdict
from pathlib import Path
from .config import load_config
from .devices import MegaLink,Lidar,scan_usable
from .storage import load_map
from .routes import load_route,compatible_config
from .planning import Planner
from .models import Pose
from .runtime import run_exploration,run_navigation


def main():
    parser=argparse.ArgumentParser(description='CAT Rover: camera-free static mapping and autonomous routes')
    commands=parser.add_subparsers(dest='command',required=True)
    for name in ('explore','navigate','diagnose'):
        p=commands.add_parser(name); p.add_argument('--config',required=True)
        if name!='diagnose': p.add_argument('--allow-motion',action='store_true'); p.add_argument('--log',default='run.jsonl')
        if name=='explore': p.add_argument('--output',required=True)
        if name=='navigate': p.add_argument('--map',required=True); p.add_argument('--route',required=True)
        if name=='diagnose': p.add_argument('--seconds',type=float,default=10)
    p=commands.add_parser('view-map'); p.add_argument('--map',required=True); p.add_argument('--port',type=int,default=8765)
    p.add_argument('--no-browser',action='store_true')
    p=commands.add_parser('simulate'); p.add_argument('--scenario',choices=['room','obstacle','dropout','ambiguous','live-obstacle'],default='room')
    p.add_argument('--output',required=True)
    p=commands.add_parser('replay-telemetry'); p.add_argument('--archive',required=True)
    args=parser.parse_args()
    def interrupted(_sig,_frame): raise KeyboardInterrupt('Stopped by signal')
    if hasattr(signal,'SIGTERM'): signal.signal(signal.SIGTERM,interrupted)
    try:
        if args.command=='view-map':
            from .editor import serve
            serve(args.map,args.port,not args.no_browser); return 0
        if args.command=='simulate':
            from .simulation import simulate
            report=simulate(args.output,args.scenario); print(json.dumps(report,indent=2))
            expected='partial' if args.scenario in ('dropout','ambiguous') else 'complete'
            return 0 if report['survey']['status']==expected and report['collisions']==0 and report.get('navigation',{'status':'complete'})['status']=='complete' else 1
        if args.command=='replay-telemetry':
            from .replay import audit_archive
            print(json.dumps(audit_archive(args.archive),indent=2)); return 0
        cfg=load_config(args.config)
        data=route=None
        if args.command!='diagnose':
            if not args.allow_motion or not cfg['hardware_confirmed'] or not cfg['calibration_confirmed']:
                raise ValueError('Confirm wiring and calibration in config, then supply --allow-motion')
            if args.command=='explore' and Path(args.output).exists():
                raise ValueError('Output map already exists; choose a new filename for this survey')
            if args.command=='navigate':
                data=load_map(args.map); compatible_config(cfg,data[2]['config'])
                route=load_route(args.route,data[0],cfg)
                planner=Planner(data[0],cfg); previous=Pose(*route['start'])
                for i,goal in enumerate(route['goals']):
                    target=Pose(*goal['pose'])
                    if not planner.plan(previous,target): raise ValueError(f'No footprint-safe path to point {i+1}')
                    previous=target
        elif not 0<args.seconds<=60: raise ValueError('diagnose --seconds must be in (0,60]')
        # All input validation precedes device initialization.
        link=MegaLink(cfg['mega_port'])
        try: lidar=Lidar(cfg)
        except BaseException: link.close(); raise
        if args.command=='diagnose':
            try:
                end=time.monotonic()+args.seconds
                while time.monotonic()<end:
                    p=link.get(); scan=lidar.get(); identity=link.identity()
                    if p: link.send(p)
                    print(json.dumps({'identity':identity,'telemetry':asdict(p) if p else None,
                                      'scan_points':len(scan.points) if scan else 0,
                                      'scan_usable':scan_usable(scan,cfg,time.monotonic())}),flush=True)
                    time.sleep(.25)
            finally:
                try: lidar.close()
                finally: link.close()
            return 0
        try:
            with Path(args.log).open('w') as log:
                result=(run_exploration(cfg,(link,lidar),args.output,log=log) if args.command=='explore'
                        else run_navigation(cfg,(link,lidar),data,route,log=log))
        except BaseException:
            try: lidar.close()
            finally: link.close()
            raise
        print(json.dumps(asdict(result),indent=2)); return 0 if result.status=='complete' else 1
    except (Exception,KeyboardInterrupt) as e:
        print(f'Error: {str(e) or "Operator interrupted"}',file=sys.stderr); return 1


if __name__=='__main__': sys.exit(main())
