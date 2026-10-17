#pragma once
#include <pgmspace.h>

static const char INDEX_HTML[] PROGMEM = R"rawliteral(<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Rover Core Gateway</title>
  <style>
    body { font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif; background: #0f172a; color: #f8fafc; margin: 0; padding: 24px; display: flex; flex-direction: column; align-items: center; justify-content: center; min-height: 100vh; box-sizing: border-box; }
    .card { background: #1e293b; border: 1px solid #334155; border-radius: 12px; padding: 24px; max-width: 480px; width: 100%; box-shadow: 0 10px 25px -5px rgba(0, 0, 0, 0.5); }
    h1 { font-size: 1.5rem; margin-top: 0; color: #38bdf8; display: flex; align-items: center; gap: 8px; }
    p { color: #94a3b8; font-size: 0.95rem; line-height: 1.5; }
    .btn { display: inline-block; background: #0284c7; color: white; text-decoration: none; padding: 10px 18px; border-radius: 8px; font-weight: 600; margin-top: 12px; transition: background 0.2s; }
    .btn:hover { background: #0369a1; }
    .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; margin-top: 16px; font-size: 0.85rem; }
    .item { background: #0f172a; padding: 10px; border-radius: 6px; border: 1px solid #334155; }
    .label { color: #64748b; font-size: 0.75rem; text-transform: uppercase; }
    .val { font-size: 1.1rem; font-weight: bold; color: #e2e8f0; margin-top: 2px; }
  </style>
</head>
<body>
  <div class="card">
    <h1>🚀 Rover Core Gateway</h1>
    <p>Autonomous Mecanum Rover onboard telemetry & sensory core.</p>
    <a class="btn" href="http://192.168.0.111:8090" target="_blank">Open Full ROS 2 3D Dashboard ↗</a>
    <div class="grid">
      <div class="item"><div class="label">Status</div><div class="val" id="st">ONLINE</div></div>
      <div class="item"><div class="label">Ultrasonic</div><div class="val" id="us">-- mm</div></div>
      <div class="item"><div class="label">ToF Left</div><div class="val" id="tof1">-- mm</div></div>
      <div class="item"><div class="label">ToF Right</div><div class="val" id="tof2">-- mm</div></div>
      <div class="item"><div class="label">Heading</div><div class="val" id="hd">--°</div></div>
      <div class="item"><div class="label">Mode</div><div class="val" id="md">--</div></div>
    </div>
  </div>
  <script>
    async function update() {
      try {
        const [rS, rT] = await Promise.all([fetch('/sensors').then(r=>r.json()), fetch('/telem').then(r=>r.json())]);
        document.getElementById('us').textContent = rS.us_mm + ' mm';
        document.getElementById('tof1').textContent = rS.tof1_mm + ' mm';
        document.getElementById('tof2').textContent = rS.tof2_mm + ' mm';
        document.getElementById('hd').textContent = rT.hd ? rT.hd.toFixed(1) + '°' : '--';
        document.getElementById('md').textContent = rT.mode !== undefined ? rT.mode : '--';
      } catch (e) {}
    }
    setInterval(update, 500);
    update();
  </script>
</body>
</html>
)rawliteral";
