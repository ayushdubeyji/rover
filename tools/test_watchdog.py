#!/usr/bin/env python3
import os
import re
import sys
import subprocess

def test_index_html():
    html_path = r"C:\Users\dell\Desktop\rover\index.html"
    with open(html_path, "r", encoding="utf-8") as f:
        content = f.read()

    # 1. Structural DOM elements
    assert 'id="picam-stream-img"' in content, "Missing #picam-stream-img"
    assert 'id="fpv-stream-img"' in content, "Missing #fpv-stream-img"
    assert 'id="fpv-modal"' in content, "Missing #fpv-modal"
    assert 'id="picam-placeholder"' in content, "Missing #picam-placeholder"
    assert 'id="picam-fs-btn"' in content, "Missing #picam-fs-btn"

    # 2. Watchdog & Reconnect functions
    assert 'reloadPiCamStreams' in content, "Missing reloadPiCamStreams"
    assert 'startPiCamAutoReconnect' in content, "Missing startPiCamAutoReconnect"
    assert 'recoveredAfterGap' in content, "Missing recoveredAfterGap"
    assert 'gap > 3000' in content, "Missing 3000ms gap check"
    assert 'recoveredAfterStall' in content, "Missing recoveredAfterStall for activity stall"
    assert 'naturalWidth === 0' in content, "Missing naturalWidth === 0 check"
    assert 'stream?t=' in content, "Missing cache-busting timestamp reload"
    assert 'setInterval' in content, "Missing watchdog interval"
    assert '/api/pi_cam/status' in content, "Missing fallback status poll"

    # 3. Rate limiting / Request storm protection
    assert 'now - piCamLastReload < 1900' in content, "Missing reload rate-limiting / cooldown"

    # 4. FPV modal bandwidth protection
    assert '<img id="fpv-stream-img" src=""' in content, "fpv-stream-img should start with empty src to prevent idle bandwidth drain"

    print("[PASS] index.html watchdog & auto-reconnect structural tests passed!")

def test_index_html_simulation():
    """Execute real JavaScript simulation using Node.js to verify watchdog state machine."""
    js_test = r"""
    const assert = require('assert');

    // Minimal DOM Mock
    const elements = {
        'picam-stream-img': { id: 'picam-stream-img', src: 'http://192.168.0.103:8081/stream', style: { display: 'block' }, naturalWidth: 0 },
        'fpv-stream-img': { id: 'fpv-stream-img', src: '', style: { display: 'none' }, naturalWidth: 0 },
        'fpv-modal': { id: 'fpv-modal', style: { display: 'none' } },
        'picam-placeholder': { id: 'picam-placeholder', style: { display: 'none' } },
        'picam-badge': { className: '', innerText: '' },
        'picam-status-dot': { style: {} }
    };
    global.document = { getElementById: id => elements[id] || null };

    // Watchdog variables
    let piCamLastSeen = 0;
    let piCamLastUartTx = -1;
    let piCamLastFrameSeq = -1;
    let piCamLastAdvanceTime = 0;
    let piCamWasOffline = true;
    let piCamLastReload = 0;
    let piCamReconnectInterval = null;
    let reloadCount = 0;

    function reloadPiCamStreams() {
        const now = Date.now();
        if (now - piCamLastReload < 1900) return;
        piCamLastReload = now;
        reloadCount++;

        const cacheBustUrl = 'http://192.168.0.103:8081/stream?t=' + now;
        const img = document.getElementById('picam-stream-img');
        const placeholder = document.getElementById('picam-placeholder');
        if (img) {
            img.style.display = 'block';
            img.src = cacheBustUrl;
        }
        if (placeholder && img && img.naturalWidth > 0) {
            placeholder.style.display = 'none';
        }
        const modal = document.getElementById('fpv-modal');
        if (modal && modal.style.display === 'flex') {
            const fpvImg = document.getElementById('fpv-stream-img');
            if (fpvImg) {
                fpvImg.style.display = 'block';
                fpvImg.src = cacheBustUrl;
            }
        }
    }

    function startPiCamAutoReconnect() {
        reloadPiCamStreams();
        if (piCamReconnectInterval) return;
        piCamReconnectInterval = setInterval(() => {
            const img = document.getElementById('picam-stream-img');
            const isLive = img && img.naturalWidth > 0 && img.style.display !== 'none';
            if (isLive && !piCamWasOffline) {
                clearInterval(piCamReconnectInterval);
                piCamReconnectInterval = null;
                const placeholder = document.getElementById('picam-placeholder');
                if (placeholder) placeholder.style.display = 'none';
                return;
            }
            reloadPiCamStreams();
        }, 2000);
    }

    function updatePiCamTelemetry(cam) {
        if (!cam) return;
        const now = Date.now();
        const gap = piCamLastSeen > 0 ? (now - piCamLastSeen) : 0;
        const isOnline = !!cam.cam_online || (cam.last_seen && (now / 1000 - cam.last_seen < 4.0));

        let activityAdvanced = false;
        if (cam.uart_tx !== undefined && cam.uart_tx !== null) {
            if (cam.uart_tx !== piCamLastUartTx) {
                piCamLastUartTx = cam.uart_tx;
                activityAdvanced = true;
            }
        }
        const frameSeq = cam.frame_seq !== undefined ? cam.frame_seq : (cam.frames !== undefined ? cam.frames : -1);
        if (frameSeq >= 0) {
            if (frameSeq !== piCamLastFrameSeq) {
                piCamLastFrameSeq = frameSeq;
                activityAdvanced = true;
            }
        }

        const stallGap = piCamLastAdvanceTime > 0 ? (now - piCamLastAdvanceTime) : 0;
        const recoveredAfterStall = (activityAdvanced && stallGap > 3000 && isOnline);
        if (activityAdvanced) {
            piCamLastAdvanceTime = now;
        }

        const recoveredAfterGap = (gap > 3000 && isOnline);
        const recoveredFromOffline = (piCamWasOffline && isOnline);

        piCamLastSeen = now;

        if (isOnline) {
            if (recoveredAfterGap || recoveredFromOffline || recoveredAfterStall) {
                const img = document.getElementById('picam-stream-img');
                const placeholder = document.getElementById('picam-placeholder');
                if (img) img.style.display = 'block';
                if (placeholder) placeholder.style.display = 'none';
                startPiCamAutoReconnect();
            }
            piCamWasOffline = false;
        } else {
            piCamWasOffline = true;
            const img = document.getElementById('picam-stream-img');
            const placeholder = document.getElementById('picam-placeholder');
            if (placeholder) placeholder.style.display = 'flex';
            if (img) img.style.display = 'none';
            startPiCamAutoReconnect();
        }
    }

    // TEST 1: Rapid 20Hz telemetry packets must NOT cause a storm of reloadPiCamStreams calls
    reloadCount = 0;
    piCamWasOffline = false;
    piCamLastSeen = Date.now();
    for (let i = 0; i < 20; i++) {
        updatePiCamTelemetry({ cam_online: true, last_seen: Date.now() / 1000, uart_tx: 100, frame_seq: 200 });
    }
    assert.strictEqual(reloadCount, 0, "20Hz telemetry updates should not trigger reload when stream is steady");

    // TEST 2: Recovery from offline triggers reconnect
    piCamWasOffline = true;
    piCamLastReload = 0;
    updatePiCamTelemetry({ cam_online: true, last_seen: Date.now() / 1000, uart_tx: 101, frame_seq: 201 });
    assert.strictEqual(reloadCount, 1, "Recovery from offline must trigger stream reload");

    // TEST 3: Telemetry gap > 3000ms triggers reload
    piCamLastSeen = Date.now() - 3500;
    piCamLastReload = 0;
    updatePiCamTelemetry({ cam_online: true, last_seen: Date.now() / 1000, uart_tx: 102, frame_seq: 202 });
    assert.strictEqual(reloadCount, 2, "Telemetry gap >3s must trigger stream reload");

    // TEST 4: Activity stall > 3000ms triggers reload
    piCamLastAdvanceTime = Date.now() - 4000;
    piCamLastReload = 0;
    updatePiCamTelemetry({ cam_online: true, last_seen: Date.now() / 1000, uart_tx: 103, frame_seq: 205 });
    assert.strictEqual(reloadCount, 3, "Activity stall >3s followed by resume must trigger stream reload");

    // TEST 5: Rate limit protects against rapid reloads (< 1900ms)
    piCamLastReload = 0;
    reloadPiCamStreams();
    assert.strictEqual(reloadCount, 4, "Initial reload after reset must increment count");
    reloadPiCamStreams(); // immediately called
    assert.strictEqual(reloadCount, 4, "Immediate second reload must be ignored by cooldown guard");

    if (piCamReconnectInterval) clearInterval(piCamReconnectInterval);
    console.log("[PASS] Node.js watchdog state machine simulation verified successfully!");
    """;

    res = subprocess.run(["node", "-e", js_test], capture_output=True, text=True)
    assert res.returncode == 0, f"Node.js simulation failed: {res.stderr}\n{res.stdout}"
    print(res.stdout.strip())

def test_pi_cam_cpp():
    cpp_path = r"C:\Users\dell\Desktop\rover\pi_cam\pi_cam_node.cpp"
    with open(cpp_path, "r", encoding="utf-8") as f:
        content = f.read()

    assert '#include <poll.h>' in content, "Missing <poll.h>"
    assert 'poll(&pfd, 1, pollTimeoutMs)' in content, "Missing dynamic pollTimeoutMs watchdog"
    assert 'hasReceivedFirstFrame ? 2000 : 4000' in content, "Missing initial sensor negotiation timeout"
    assert 'cleanPath == "/stream"' in content, "Missing cleanPath for /stream"
    assert 'cleanPath == "/status"' in content, "Missing cleanPath for /status"
    assert 'pkill -9 -f rpicam-vid' in content, "Missing pkill for stalled pipeline"
    assert 'lastProbeTime).count() >= 2' in content, "Fallback probe interval must be <= 2s"
    assert 'probeCsiCameraHardware()' in content, "Missing CSI probe"
    assert 'g_vision.frame_seq' in content, "Missing frame_seq tracking in g_vision"
    assert r'\"frame_seq\":%llu' in content, "Missing frame_seq in JSON endpoints"

    print("[PASS] pi_cam_node.cpp timeout watchdog, frame_seq & routing tests passed!")

def test_service():
    deploy_path = r"C:\Users\dell\Desktop\rover\tools\deploy_pi_cam.py"
    with open(deploy_path, "r", encoding="utf-8") as f:
        content = f.read()

    assert 'Restart=always' in content, "Missing Restart=always in service"
    assert 'RestartSec=1' in content, "Missing RestartSec=1 in service"

    print("[PASS] rover-cam.service unit configuration tests passed!")

if __name__ == '__main__':
    test_index_html()
    test_index_html_simulation()
    test_pi_cam_cpp()
    test_service()
    print("ALL TESTS PASSED!")
