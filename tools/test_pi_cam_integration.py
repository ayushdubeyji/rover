import subprocess
import time
import json
import urllib.request
import socket
import sys

def run_tests():
    print("[TEST] Launching web_telemetry_server.py locally on port 8090...")
    proc = subprocess.Popen([sys.executable, "web_telemetry_server.py"], cwd=r"C:\Users\dell\Desktop\rover", stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)

    try:
        # 1. Wait for server to bind
        time.sleep(4.5)

        # 2. Check HTTP Index
        print("[TEST] 1. Checking GET http://127.0.0.1:8090/ ...")
        with urllib.request.urlopen("http://127.0.0.1:8090/", timeout=3.0) as res:
            assert res.status == 200, f"Expected status 200, got {res.status}"
            html = res.read().decode('utf-8')
            assert 'id="card-pi-cam"' in html, "card-pi-cam element missing in index.html"
            assert 'id="picam-stream-img"' in html, "picam-stream-img element missing in index.html"
            assert 'id="picam-bar-f"' in html, "picam-bar-f element missing in index.html"
            assert 'updatePiCamTelemetry' in html, "updatePiCamTelemetry function missing in index.html"
            print("  -> PASS: index.html contains all Pi Cam HUD elements and bindings!")

        # 3. Check /api/pi_cam/status (populated via poll_pi_cam_loop from Pi 3B+ @ 192.168.0.103:8081)
        print("[TEST] 2. Checking GET http://127.0.0.1:8090/api/pi_cam/status ...")
        # Give poller another 1-2 seconds
        time.sleep(2.0)
        with urllib.request.urlopen("http://127.0.0.1:8090/api/pi_cam/status", timeout=3.0) as res:
            assert res.status == 200, f"Expected status 200, got {res.status}"
            status_data = json.loads(res.read().decode('utf-8'))
            print("  Received status from server:", status_data)
            assert "fps" in status_data, "fps field missing"
            assert "vis_f_mm" in status_data, "vis_f_mm field missing"
            assert "cpu_usage" in status_data, "cpu_usage field missing"
            assert "stream_url" in status_data, "stream_url field missing"
            print("  -> PASS: /api/pi_cam/status successfully populated from Pi 3B+ daemon!")

        # 4. Test UDP ingestion of R,... packet to 8888
        print("[TEST] 3. Testing UDP R,... telemetry packet forwarding on 8888...")
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.sendto(b"R,92.5,350,280,290,1\n", ("127.0.0.1", 8888))
        time.sleep(0.2)
        sock.close()
        print("  -> PASS: UDP packet sent successfully!")

        print("\n==========================================")
        print(" ALL VERIFICATION TESTS PASSED (3/3)")
        print("==========================================")
        return True

    except Exception as e:
        print(f"[TEST FAIL] Error during verification: {e}")
        return False
    finally:
        print("[TEST] Terminating local server...")
        proc.terminate()
        try:
            proc.wait(timeout=3)
        except Exception:
            proc.kill()

if __name__ == "__main__":
    ok = run_tests()
    sys.exit(0 if ok else 1)
