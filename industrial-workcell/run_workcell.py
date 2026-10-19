"""Start the dashboard and the production MQTT bridge together. Ctrl+C stops both."""
import functools
import http.server
import os
from pathlib import Path
import subprocess
import sys
import threading

folder=Path(__file__).resolve().parent
build=folder.parent/'remote-robot-frontend'/'build'
if not (build/'index.html').exists():
    raise SystemExit('Build the React frontend first: npm run build')
handler=functools.partial(http.server.SimpleHTTPRequestHandler,directory=str(build))
try:
    server=http.server.ThreadingHTTPServer(('127.0.0.1',3000),handler)
except OSError:
    raise SystemExit('Port 3000 is in use. Stop the previous dashboard server first.')
with (folder/'bridge.log').open('a',encoding='utf-8') as log:
    child=subprocess.Popen([sys.executable,str(folder/'workcell_backend.py')],stdout=log,stderr=subprocess.STDOUT,
                           creationflags=subprocess.CREATE_NO_WINDOW if os.name=='nt' else 0)
    threading.Thread(target=server.serve_forever,daemon=True).start()
    print('Dashboard: http://127.0.0.1:3000',flush=True)
    print('Production bridge running. Open production_cell.ttt in CoppeliaSim. Press Start auto in the dashboard.',flush=True)
    print('Ctrl+C stops the dashboard and backend. Backend log: '+str(folder/'bridge.log'),flush=True)
    try:
        child.wait()
    except KeyboardInterrupt:
        child.terminate()
        child.wait(timeout=5)
    finally:
        server.shutdown(); server.server_close()
        if child.poll() is None: child.terminate()
