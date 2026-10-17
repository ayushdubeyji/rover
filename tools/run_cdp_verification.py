import subprocess
import time
import sys

def main():
    print("[RUNNER] Starting web_telemetry_server.py...")
    server = subprocess.Popen([sys.executable, "web_telemetry_server.py"], cwd=r"C:\Users\dell\Desktop\rover")
    time.sleep(3.0)

    try:
        print("[RUNNER] Executing node tools/test_cdp_local.js...")
        res = subprocess.run(["node", r"C:\Users\dell\Desktop\rover\tools\test_cdp_local.js"], cwd=r"C:\Users\dell\Desktop\rover", capture_output=True, text=True)
        print("STDOUT:\n", res.stdout)
        if res.stderr:
            print("STDERR:\n", res.stderr)
        return res.returncode == 0
    finally:
        print("[RUNNER] Stopping server...")
        server.terminate()
        try:
            server.wait(timeout=3)
        except Exception:
            server.kill()

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
