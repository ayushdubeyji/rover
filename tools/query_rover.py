import urllib.request
import json
import time

for path in ['/status', '/sensors', '/cfg', '/map', '/telem']:
    url = f"http://192.168.0.101{path}"
    req = urllib.request.Request(url, headers={'Connection': 'close'})
    try:
        with urllib.request.urlopen(req, timeout=2.5) as r:
            body = r.read().decode('utf-8')
            print(f"[{path}] -> {body}")
    except Exception as e:
        print(f"[{path}] -> ERROR: {e}")
    time.sleep(0.5)
