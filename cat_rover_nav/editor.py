"""Local browser map/route editor. This server never opens rover devices."""
import json
import threading
import webbrowser
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
from pathlib import Path
import numpy as np
from .storage import load_map,map_identity
from .routes import validate_route
from .models import Pose
from .planning import Planner


def serve(map_path,port=8765,open_browser=True):
    grid,_,meta=load_map(map_path)
    payload={'cells':grid.state()[::-1].tolist(),'resolution':grid.res,'origin':grid.origin,
             'identity':map_identity(grid),'status':meta.get('status','unknown'),
             'body':[grid.cfg['body_length_m'],grid.cfg['body_width_m']],'margin':grid.cfg['margin_m']}
    class Handler(BaseHTTPRequestHandler):
        def respond(self,status,data,kind='application/json'):
            body=data if isinstance(data,bytes) else json.dumps(data).encode()
            self.send_response(status); self.send_header('Content-Type',kind)
            self.send_header('Content-Length',str(len(body))); self.send_header('Cache-Control','no-store')
            self.end_headers(); self.wfile.write(body)
        def do_GET(self):
            if self.path=='/':
                self.respond(200,Path(__file__).with_name('map_editor.html').read_bytes(),'text/html; charset=utf-8')
            elif self.path=='/map': self.respond(200,payload)
            else: self.respond(404,{'error':'Not found'})
        def do_POST(self):
            if self.path!='/validate': return self.respond(404,{'error':'Not found'})
            try:
                size=int(self.headers.get('Content-Length','0'))
                if not 0<size<=65536: raise ValueError('Route too large or empty')
                route=validate_route(json.loads(self.rfile.read(size)),grid)
                paths=[]; previous=Pose(*route['start']); planner=Planner(grid,grid.cfg)
                for i,goal in enumerate(route['goals']):
                    target=Pose(*goal['pose']); path=planner.plan(previous,target)
                    if not path: raise ValueError(f'No footprint-safe path to point {i+1}')
                    paths.append([p.array().tolist() for p in path]); previous=target
                self.respond(200,{'valid':True,'paths':paths})
            except (ValueError,KeyError,TypeError) as e: self.respond(400,{'error':str(e)})
    server=ThreadingHTTPServer(('127.0.0.1',port),Handler)
    print(f'Map editor: http://127.0.0.1:{port} (Ctrl+C to close)',flush=True)
    if open_browser: threading.Timer(.3,lambda:webbrowser.open(f'http://127.0.0.1:{port}')).start()
    try: server.serve_forever()
    except KeyboardInterrupt: pass
    finally: server.server_close()
