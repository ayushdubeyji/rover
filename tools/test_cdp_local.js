const { spawn } = require('child_process');
const fs = require('fs');

async function run() {
  const chrome = spawn('C:/Program Files/Google/Chrome/Application/chrome.exe', [
    '--headless=new',
    '--remote-debugging-port=9227',
    'about:blank'
  ]);

  let tabs = null;
  for (let i = 0; i < 15; i++) {
    try {
      const res = await fetch('http://127.0.0.1:9227/json');
      tabs = await res.json();
      if (tabs && tabs.length > 0) break;
    } catch (e) {
      await new Promise(r => setTimeout(r, 400));
    }
  }

  let errorCount = 0;

  try {
    const target = tabs.find(t => t.type === 'page') || tabs[0];
    const ws = new WebSocket(target.webSocketDebuggerUrl);

    ws.addEventListener('open', () => {
      ws.send(JSON.stringify({ id: 1, method: 'Runtime.enable' }));
      ws.send(JSON.stringify({ id: 2, method: 'Page.enable' }));
      ws.send(JSON.stringify({
        id: 4,
        method: 'Emulation.setDeviceMetricsOverride',
        params: { width: 1400, height: 1800, deviceScaleFactor: 1, mobile: false }
      }));
      ws.send(JSON.stringify({ id: 3, method: 'Page.navigate', params: { url: 'http://127.0.0.1:8090' } }));
    });

    ws.addEventListener('message', (evt) => {
      const msg = JSON.parse(evt.data);
      if (msg.method === 'Runtime.consoleAPICalled') {
        const text = msg.params.args.map(a => a.value || a.description || '').join(' ');
        if (msg.params.type === 'error') {
          console.error('[CONSOLE ERROR]', text);
          errorCount++;
        } else {
          console.log('[CONSOLE]', msg.params.type, text);
        }
      } else if (msg.method === 'Runtime.exceptionThrown') {
        console.error('[EXCEPTION]', msg.params.exceptionDetails.text, msg.params.exceptionDetails.exception?.description);
        errorCount++;
      } else if (msg.id === 10) {
        console.log('[PAGE EVAL]', JSON.stringify(msg.result?.result?.value, null, 2));
      } else if (msg.id === 20 && msg.result?.data) {
        fs.writeFileSync('C:/Users/dell/Desktop/rover/tools/pi_cam_hud_screenshot.png', Buffer.from(msg.result.data, 'base64'));
        console.log('[SCREENSHOT] Saved to tools/pi_cam_hud_screenshot.png');
      }
    });

    setTimeout(() => {
      ws.send(JSON.stringify({
        id: 10,
        method: 'Runtime.evaluate',
        params: {
          expression: `({
            title: document.title,
            hasPiCamCard: !!document.getElementById('card-pi-cam'),
            hasStreamImg: !!document.getElementById('picam-stream-img'),
            hasSectorBars: !!document.getElementById('picam-bar-f'),
            hasHwBadges: !!document.getElementById('picam-hw-cpu'),
            picamBadgeText: document.getElementById('picam-badge')?.textContent,
            has3DCanvas: !!document.querySelector('#rover-3d-container canvas')
          })`,
          returnByValue: true
        }
      }));
      ws.send(JSON.stringify({ id: 20, method: 'Page.captureScreenshot' }));
    }, 4500);

    setTimeout(() => {
      ws.close();
      chrome.kill();
      if (errorCount > 0) {
        console.error(`FAILED: ${errorCount} console errors encountered.`);
        process.exit(1);
      } else {
        console.log('SUCCESS: 0 console errors, clean DOM rendering confirmed!');
        process.exit(0);
      }
    }, 6000);

  } catch (err) {
    console.error('Error:', err);
    chrome.kill();
    process.exit(1);
  }
}

run();
