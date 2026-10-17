import asyncio
import websockets
import json

async def main():
    uri = "ws://127.0.0.1:8090/ws/telemetry"
    try:
        async with websockets.connect(uri) as ws:
            print("Connected to WS telemetry!")
            for i in range(2):
                msg = await asyncio.wait_for(ws.recv(), timeout=3.0)
                print(f"RAW [{i}]: {msg}")
    except Exception as e:
        print(f"WS error: {e}")

asyncio.run(main())
