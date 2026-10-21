import asyncio
import json
import websockets

async def get_telem():
    async with websockets.connect('ws://192.168.0.111:8090/ws/telemetry') as ws:
        msg = await asyncio.wait_for(ws.recv(), timeout=2.0)
        data = json.loads(msg)
        print("Telemetry keys:", list(data.keys()))
        print("autoState:", data.get("auto_state"), data.get("autoState"))
        print("impact:", data.get("impact"), data.get("imp"))
        print("mode:", data.get("mode"), data.get("mode_name"))
        print("link_type:", data.get("link_type"))
        print("full telem:", json.dumps(data, indent=2))

asyncio.run(get_telem())
