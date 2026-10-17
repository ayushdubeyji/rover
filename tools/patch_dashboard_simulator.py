import re

with open(r'C:\Users\dell\Desktop\rover\tools\index_host.html', 'r', encoding='utf-8') as f:
    content = f.read()

# 1. Update Card 3D HTML Header & Viewport
old_card_html = """        <!-- Card 3D: Live 3D Rover Digital Twin -->
        <div class="card" style="grid-column: 1 / -1;">
            <div class="card-title" style="display: flex; justify-content: space-between; align-items: center; flex-wrap: wrap; gap: 8px;">
                <span>Rover 3D Digital Twin (Live Wheel Kinematics)</span>
                <span id="model-status" style="font-size: 0.75rem; color: #10b981; font-weight: normal;">Initializing 3D Engine...</span>
            </div>
            <div id="rover-3d-container" style="width: 100%; height: 380px; border-radius: 8px; overflow: hidden; background: #070a13; position: relative;">
                <div style="position: absolute; bottom: 8px; left: 12px; font-size: 0.72rem; color: #64748b; font-family: monospace; pointer-events: none; z-index: 5;">
                    Orbit: Left-Click Drag · Pan: Right-Click Drag · Zoom: Scroll
                </div>
                <div id="telemetry-hud-3d" style="position: absolute; top: 10px; right: 12px; font-size: 0.75rem; color: #38bdf8; font-family: monospace; z-index: 5; background: rgba(15,23,42,0.8); padding: 4px 10px; border-radius: 4px; border: 1px solid #1e293b;">
                    <span id="hud-yaw">0.0°</span> | <span id="hud-pos" style="color: #4ade80;">X: 0.00m Y: 0.00m</span>
                </div>
            </div>
        </div>"""

new_card_html = """        <!-- Card 3D: Live 3D Rover Digital Twin & Arena Simulator -->
        <div class="card" style="grid-column: 1 / -1;">
            <div class="card-title" style="display: flex; justify-content: space-between; align-items: center; flex-wrap: wrap; gap: 8px;">
                <div style="display: flex; align-items: center; gap: 10px;">
                    <span style="display: flex; align-items: center; gap: 6px;">
                        <span style="display: inline-block; width: 8px; height: 8px; border-radius: 50%; background: #38bdf8; box-shadow: 0 0 8px #38bdf8;"></span>
                        Rover 3D Digital Twin & Arena Simulator
                    </span>
                    <span id="model-status" style="font-size: 0.75rem; color: #10b981; font-weight: normal;">Initializing 3D Engine...</span>
                </div>
                <div style="display: flex; align-items: center; gap: 6px;">
                    <button id="btn-cam-mode" style="background: #1e293b; color: #38bdf8; border: 1px solid #334155; padding: 4px 10px; border-radius: 4px; font-size: 0.72rem; cursor: pointer; transition: all 0.2s;">🎥 Chase Cam: OFF</button>
                    <button id="btn-center-rover" style="background: #1e293b; color: #94a3b8; border: 1px solid #334155; padding: 4px 10px; border-radius: 4px; font-size: 0.72rem; cursor: pointer; transition: all 0.2s;">🎯 Center</button>
                    <button id="btn-clear-trail" style="background: #1e293b; color: #94a3b8; border: 1px solid #334155; padding: 4px 10px; border-radius: 4px; font-size: 0.72rem; cursor: pointer; transition: all 0.2s;">🧹 Clear Trail</button>
                </div>
            </div>
            <div id="rover-3d-container" style="width: 100%; height: 480px; border-radius: 8px; overflow: hidden; background: #070a13; position: relative;">
                <div style="position: absolute; bottom: 8px; left: 12px; font-size: 0.72rem; color: #64748b; font-family: monospace; pointer-events: none; z-index: 5;">
                    Orbit: Left Drag · Pan: Right Drag · Zoom: Scroll · Chase: Toggle Above
                </div>
                <div id="telemetry-hud-3d" style="position: absolute; top: 10px; right: 12px; font-size: 0.75rem; color: #38bdf8; font-family: monospace; z-index: 5; background: rgba(15,23,42,0.85); padding: 5px 12px; border-radius: 6px; border: 1px solid #1e293b; backdrop-filter: blur(4px);">
                    <span id="hud-yaw">0.0°</span> | <span id="hud-pos" style="color: #4ade80;">X: 0.00m Y: 0.00m</span>
                </div>
            </div>
        </div>"""

if old_card_html in content:
    content = content.replace(old_card_html, new_card_html)
    print("Card 3D HTML replaced successfully!")
else:
    print("Warning: old_card_html exact match not found!")

with open(r'C:\Users\dell\Desktop\rover\tools\index_host.html', 'w', encoding='utf-8') as f:
    f.write(content)
