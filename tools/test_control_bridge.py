import asyncio
import websockets
import json

async def test_control():
    uri = 'ws://192.168.0.111:8090/ws/telemetry'
    print(f"[TEST] Connecting to {uri}...")
    async with websockets.connect(uri) as ws:
        cmd = {
            "type": "control",
            "vx": 0.5,
            "vy": -0.2,
            "vz": 0.0,
            "speed": 75,
            "mode": 1
        }
        await ws.send(json.dumps(cmd))
        print("[TEST PASS] Sent control message successfully:", cmd)

if __name__ == "__main__":
    asyncio.run(test_control())
