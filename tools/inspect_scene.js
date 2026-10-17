const { spawn } = require('child_process');

async function inspect() {
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

  const target = tabs.find(t => t.type === 'page') || tabs[0];
  const ws = new WebSocket(target.webSocketDebuggerUrl);

  ws.addEventListener('open', () => {
    ws.send(JSON.stringify({ id: 1, method: 'Runtime.enable' }));
    ws.send(JSON.stringify({ id: 2, method: 'Page.enable' }));
    ws.send(JSON.stringify({ id: 3, method: 'Page.navigate', params: { url: 'http://192.168.0.111:8090' } }));
  });

  ws.addEventListener('message', (evt) => {
    const msg = JSON.parse(evt.data);
    if (msg.method === 'Runtime.consoleAPICalled') {
      console.log('[CONSOLE]', msg.params.args.map(a => a.value || a.description || '').join(' '));
    }
  });

  await new Promise(r => setTimeout(r, 3500));

  ws.send(JSON.stringify({
    id: 100,
    method: 'Runtime.evaluate',
    params: {
      expression: `(() => {
        const status = document.getElementById('model-status')?.textContent;
        const info = { status };
        // Check meshes in scene
        if (window.scene) {
          const meshes = [];
          window.scene.traverse(obj => {
            if (obj.isMesh) {
              meshes.push({
                name: obj.name,
                geo: obj.geometry?.type,
                mat: obj.material?.type,
                color: obj.material?.color?.getHexString ? obj.material.color.getHexString() : null,
                pos: [obj.position.x, obj.position.y, obj.position.z],
                scale: [obj.scale.x, obj.scale.y, obj.scale.z],
                visible: obj.visible
              });
            }
          });
          info.meshes = meshes;
        }
        return info;
      })()`,
      returnByValue: true
    }
  }));

  ws.addEventListener('message', (evt) => {
    const msg = JSON.parse(evt.data);
    if (msg.id === 100) {
      console.log('SCENE INFO:', JSON.stringify(msg.result?.result?.value, null, 2));
      ws.close();
      chrome.kill();
      process.exit(0);
    }
  });
}

inspect();
