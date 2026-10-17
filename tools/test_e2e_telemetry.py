import socket
import json
import asyncio
import websockets

async def test_bridge():
    uri = 'ws://192.168.0.111:8090/ws/telemetry'
    print("[TEST] Connecting to", uri)
    async with websockets.connect(uri) as ws:
        # Give connection 100ms
        await asyncio.sleep(0.1)

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        pkt = {
            'q': [0.9848, 0.1228, 0.0614, 0.1069],
            'g': [0.0512, 0.0234, -0.1245],
            'a': [0.512, -1.234, 9.712],
            'm': [12.4, -5.6, 33.1],
            'us': 250,
            'tof': [150, 160],
            'w': [350, -350, 350, -350],
            'pos': [12.5, 8.2],
            'hd': 45.0,
            'r': 12.0,
            'p': -8.5
        }
        # Send 3 UDP packets to ensure delivery over Wi-Fi/LAN
        for _ in range(3):
            sock.sendto(json.dumps(pkt).encode('utf-8'), ('192.168.0.111', 8888))
            await asyncio.sleep(0.02)
        sock.close()

        # Listen for broadcast
        for i in range(15):
            msg = await asyncio.wait_for(ws.recv(), timeout=2.0)
            data = json.loads(msg)
            if data.get('yaw_deg') == 45.0:
                print("[TEST PASS] Live UDP -> WebSocket telemetry verified!")
                print("  Yaw:", data.get('yaw_deg'), "deg")
                print("  Pitch:", data.get('pitch_deg'), "deg")
                print("  Roll:", data.get('roll_deg'), "deg")
                print("  Quaternion:", data.get('q'))
                print("  Gyro:", data.get('g'))
                print("  Accel:", data.get('a'))
                print("  Mag:", data.get('m'))
                print("  Wheel speeds:", data.get('w'))
                print("  Link:", data.get('link_type'))
                return True
        print("[TEST] Last received data:", data)
        return False

if __name__ == "__main__":
    success = asyncio.run(test_bridge())
    if not success:
        exit(1)
