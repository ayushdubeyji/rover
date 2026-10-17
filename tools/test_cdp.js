const { spawn } = require('child_process');

async function run() {
  const chrome = spawn('C:/Program Files/Google/Chrome/Application/chrome.exe', [
    '--headless=new',
    '--remote-debugging-port=9226',
    'about:blank'
  ]);

  let tabs = null;
  for (let i = 0; i < 15; i++) {
    try {
      const res = await fetch('http://127.0.0.1:9226/json');
      tabs = await res.json();
      if (tabs && tabs.length > 0) break;
    } catch (e) {
      await new Promise(r => setTimeout(r, 400));
    }
  }
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
      ws.send(JSON.stringify({ id: 3, method: 'Page.navigate', params: { url: 'http://192.168.0.111:8090' } }));
    });

    ws.addEventListener('message', (evt) => {
      const msg = JSON.parse(evt.data);
      if (msg.method === 'Runtime.consoleAPICalled') {
        const text = msg.params.args.map(a => a.value || a.description || '').join(' ');
        console.log('[CONSOLE]', msg.params.type, text);
      } else if (msg.method === 'Runtime.exceptionThrown') {
        console.error('[EXCEPTION]', msg.params.exceptionDetails.text, msg.params.exceptionDetails.exception?.description);
      } else if (msg.id === 10) {
        console.log('[PAGE EVAL]', msg.result?.result?.value);
      }
    });

    setTimeout(() => {
      ws.send(JSON.stringify({
        id: 10,
        method: 'Runtime.evaluate',
        params: {
          expression: `({
            title: document.title,
            status: document.getElementById('model-status')?.textContent,
            statusColor: document.getElementById('model-status')?.style?.color,
            hasCanvas: !!document.querySelector('#rover-3d-container canvas'),
            canvasWidth: document.querySelector('#rover-3d-container canvas')?.width,
            canvasHeight: document.querySelector('#rover-3d-container canvas')?.height
          })`,
          returnByValue: true
        }
      }));
      ws.send(JSON.stringify({ id: 20, method: 'Page.captureScreenshot' }));
    }, 4000);

    const fs = require('fs');
    ws.addEventListener('message', (evt) => {
      const msg = JSON.parse(evt.data);
      if (msg.id === 20 && msg.result?.data) {
        fs.writeFileSync('tools/live_page_screenshot.png', Buffer.from(msg.result.data, 'base64'));
        console.log('Saved screenshot to tools/live_page_screenshot.png');
      }
    });

    setTimeout(() => {
      ws.close();
      chrome.kill();
      process.exit(0);
    }, 5500);

  } catch (err) {
    console.error('Error:', err);
    chrome.kill();
    process.exit(1);
  }
}

run();
