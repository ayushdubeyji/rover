#pragma once
#include <pgmspace.h>

const char DASHBOARD_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html lang="en"><head><meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=0">
<title>Mecanum Rover</title>
<style>
:root{--bg:#0a0a0c;--p:#16161a;--a:#00e6b8;--t:#e0e0e0;--d:#ff4444;--w:#ffaa00;--f:#cc33ff}
*{box-sizing:border-box;touch-action:manipulation}
body{background:var(--bg);color:var(--t);font-family:'Segoe UI',sans-serif;margin:0;padding:10px;user-select:none;-webkit-user-select:none}
.wrap{max-width:900px;margin:auto}
.bar{display:flex;justify-content:space-between;align-items:center;background:var(--p);padding:10px 14px;border-radius:10px;margin-bottom:10px;flex-wrap:wrap;gap:6px}
.bar h2{margin:0;font-size:1.1rem;color:var(--a)}
.pill{font:.75rem monospace;padding:3px 8px;border-radius:12px;background:#222}
.ok{color:#0f0}.bad{color:var(--d)}
.card{background:var(--p);border-radius:10px;padding:12px;margin-bottom:10px}
.card h3{margin:0 0 8px;font-size:.85rem;color:#999;text-transform:uppercase;letter-spacing:1px}
.sticks{display:flex;gap:10px;flex-wrap:wrap;justify-content:space-around}
.stk{text-align:center}
.zone{position:relative;background:#1f1f25;border:2px solid #333;touch-action:none;margin:auto}
.zone.round{width:200px;height:200px;border-radius:50%}
.zone.bar2{width:200px;height:60px;border-radius:30px}
.knob{position:absolute;width:56px;height:56px;border-radius:50%;background:radial-gradient(circle,var(--a),#00997a);pointer-events:none;left:72px;top:72px}
.bar2 .knob{top:1px;left:68px}
.row{display:flex;align-items:center;gap:8px;margin:6px 0;font-size:.85rem}
.row label{width:42%}
.row input[type=range]{flex:1;accent-color:var(--a)}
.row input[type=number]{width:90px;background:#222;color:var(--t);border:1px solid #444;border-radius:6px;padding:5px}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(110px,1fr));gap:8px}
button{background:#2a2a35;color:var(--t);border:1px solid #333;padding:12px;border-radius:8px;font-weight:bold;font-size:.8rem;cursor:pointer}
button:active{background:var(--a);color:#000}
.st{border-color:var(--w);color:var(--w)}.st:active{background:var(--w)}
.fx{border-color:var(--f);color:var(--f)}.fx:active{background:var(--f);color:#fff}
.estop{width:100%;padding:18px;font-size:1.1rem;border-color:var(--d);color:var(--d);background:rgba(255,68,68,.12)}
.estop:active{background:var(--d);color:#fff}
.wh{display:grid;grid-template-columns:1fr 1fr;gap:8px;max-width:360px;margin:auto}
.wb{background:#111;border-radius:8px;padding:6px;text-align:center;font:.75rem monospace}
.meter{height:10px;background:#222;border-radius:5px;position:relative;margin-top:4px}
.meter i{position:absolute;top:0;bottom:0;background:var(--a);border-radius:5px}
.chk{display:inline-flex;align-items:center;gap:4px;margin-right:10px}
.tel{font:.8rem monospace;color:#aaa;text-align:center;margin-top:6px}
</style></head><body><div class="wrap">
<div class="bar"><h2>MECANUM ROVER</h2>
<span><span class="pill" id="pLink">RP2040: ?</span> <span class="pill" id="pNet">-</span> <span class="pill" id="pMac">idle</span> <span class="pill" id="pRem">Remote: ?</span> <span class="pill" id="pMode">-</span></span></div>

<div class="card"><h3>Drive</h3>
<div class="sticks">
<div class="stk"><div class="zone round" id="zT"><div class="knob" id="kT"></div></div><div>Translate</div></div>
<div class="stk"><div class="zone bar2" id="zR"><div class="knob" id="kR"></div></div><div>Rotate</div>
<div class="tel" id="tel">X0 Y0 Z0</div></div>
</div>
<div class="row"><label>Power <b id="sv">80</b>%</label><input type="range" id="spd" min="10" max="100" value="80"></div>
<div class="tel">Keys: WASD move &middot; Q/E rotate &middot; Space stop</div></div>

<div class="card"><h3>Drive Mode</h3>
<div class="grid" id="modes"></div>
<div class="tel" id="modeInfo"></div>
<h3 style="margin-top:10px">Speed Presets</h3><div class="grid">
<button onclick="preset(15)">Crawl 15%</button><button onclick="preset(35)">Slow 35%</button>
<button onclick="preset(60)">Normal 60%</button><button onclick="preset(100)">Max 100%</button></div>
<div class="row"><label>Cruise speed <b id="crv"></b>%</label><input type="range" id="cr" min="5" max="100"></div>
<div class="row"><label>Crawl cap <b id="cpv"></b>%</label><input type="range" id="cp" min="5" max="60"></div>
<div class="row"><label>Accel limit <b id="arv"></b></label><input type="range" id="ar" min="20" max="2000" step="10"></div>
<div class="row"><label>Jerk limit <b id="jrv"></b></label><input type="range" id="jr" min="50" max="10000" step="50"></div>
<div class="row"><label>Yaw trim fwd <b id="tfv"></b></label><input type="range" id="tf" min="-30" max="30"></div>
<div class="row"><label>Yaw trim strafe <b id="tsv"></b></label><input type="range" id="ts" min="-30" max="30"></div>
<div class="grid"><button onclick="rot(-360)">&#8634; 360</button><button onclick="rot(360)">&#8635; 360</button>
<button onclick="rot(-90)">&#8634; 90</button><button onclick="rot(90)">&#8635; 90</button>
<button onclick="fetch('/mode?hr=1')">Reset heading</button><button onclick="fetch('/mode?save=1').then(()=>alert('Saved'))">Save tuning</button></div></div>

<div class="card"><h3>Maneuvers</h3><div class="grid">
<button class="st" onclick="mac(1,1)">360&deg; CW</button>
<button class="st" onclick="mac(1,-1)">360&deg; CCW</button>
<button class="st" onclick="mac(2,1)">180&deg; CW</button>
<button class="st" onclick="mac(2,-1)">180&deg; CCW</button>
<button class="fx" onclick="mac(3,1)">J-Turn R</button>
<button class="fx" onclick="mac(3,-1)">J-Turn L</button>
<button class="fx" onclick="mac(4,1)">Circle R</button>
<button class="fx" onclick="mac(4,-1)">Circle L</button>
<button data-hold="0,100,50">Arc Fwd R</button>
<button data-hold="0,100,-50">Arc Fwd L</button>
<button data-hold="0,0,100">Spin CW (hold)</button>
<button data-hold="0,0,-100">Spin CCW (hold)</button>
</div></div>

<div class="card"><h3>Wheels (live)</h3><div class="wh" id="wh"></div></div>

<div class="card"><h3>Calibration</h3>
<div class="tel">Hold a button: that wheel must spin FORWARD. If not, tick Invert.</div>
<div class="grid" style="margin:8px 0">
<button data-wheel="0">Test FL</button><button data-wheel="1">Test FR</button>
<button data-wheel="2">Test RL</button><button data-wheel="3">Test RR</button></div>
<div><span class="chk"><input type="checkbox" id="i0">Inv FL</span><span class="chk"><input type="checkbox" id="i1">Inv FR</span>
<span class="chk"><input type="checkbox" id="i2">Inv RL</span><span class="chk"><input type="checkbox" id="i3">Inv RR</span></div>
<div style="margin-top:6px"><span class="chk"><input type="checkbox" id="si">Flip strafe</span><span class="chk"><input type="checkbox" id="ri">Flip rotation</span></div>
<div class="row"><label>360&deg; spin time (ms)</label><input type="number" id="sp" min="200" max="10000" step="50"></div>
<div class="row"><label>Min duty (0-60000)</label><input type="number" id="md" min="0" max="60000" step="500"></div>
<div class="row"><label>Accel step (1-100)</label><input type="number" id="ac" min="1" max="100"></div>
<button onclick="saveCfg()" style="width:100%">Save to rover</button></div>

<button class="estop" onclick="estop()">EMERGENCY STOP</button>
</div>
<script>
const $=id=>document.getElementById(id);
let tx=0,ty=0,rz=0,kx=0,ky=0,kz=0,hold=null,wheel=null,busy=false,spd=80;
$('spd').oninput=e=>{spd=+e.target.value;$('sv').textContent=spd};
$('spd').onchange=e=>{if(e.target.blur)e.target.blur()};

function stick(zone,knob,axisOnly,cb){
  const R=100,half=28;let on=false;
  function mv(e){
    if(!on)return;
    const r=zone.getBoundingClientRect();
    let dx=e.clientX-r.left-r.width/2,dy=e.clientY-r.top-r.height/2;
    if(axisOnly){const m=r.width/2-half;dx=Math.max(-m,Math.min(m,dx));dy=0;cb(Math.round(dx/m*100),0);knob.style.transform=`translate(${dx}px,0)`;}
    else{const m=R-half,d=Math.hypot(dx,dy);if(d>m){dx*=m/d;dy*=m/d}
      let X=Math.round(dx/m*100),Y=Math.round(-dy/m*100);
      if(Math.hypot(X,Y)<8){X=0;Y=0}
      cb(X,Y);knob.style.transform=`translate(${dx}px,${dy}px)`;}
  }
  zone.addEventListener('pointerdown',e=>{on=true;zone.setPointerCapture(e.pointerId);mv(e)});
  zone.addEventListener('pointermove',mv);
  const end=()=>{on=false;knob.style.transform='';cb(0,0)};
  zone.addEventListener('pointerup',end);zone.addEventListener('pointercancel',end);
}
stick($('zT'),$('kT'),false,(x,y)=>{tx=x;ty=y});
stick($('zR'),$('kR'),true,(x)=>{rz=Math.abs(x)<8?0:x});

const keys={};
const keyMap={'arrowup':'w','arrowdown':'s','arrowleft':'a','arrowright':'d'};
const DRIVE_KEYS=new Set(['w','a','s','d','q','e',' ']);

addEventListener('keydown',e=>{
  const k=keyMap[e.key.toLowerCase()]||e.key.toLowerCase();
  const isText=e.target.tagName==='INPUT'&&(e.target.type==='number'||e.target.type==='text');
  if(isText){
    if(e.key==='Enter'||e.key==='Escape')e.target.blur();
    return;
  }
  if(e.target.tagName==='INPUT'||e.target.tagName==='BUTTON'){
    e.target.blur();
  }
  if(DRIVE_KEYS.has(k)){
    e.preventDefault();
    keys[k]=1;
    if(k===' ')estop();
    kbd();
  }
});

addEventListener('keyup',e=>{
  const k=keyMap[e.key.toLowerCase()]||e.key.toLowerCase();
  delete keys[k];
  kbd();
});

window.addEventListener('blur',()=>{
  for(const k in keys)delete keys[k];
  kbd();
});

function kbd(){
  ky=(keys.w?100:0)-(keys.s?100:0);
  kx=(keys.d?100:0)-(keys.a?100:0);
  kz=(keys.e?100:0)-(keys.q?100:0);
}

document.addEventListener('pointerup',e=>{
  if(e.target&&e.target.tagName!=='BODY'&&(e.target.tagName==='INPUT'||e.target.tagName==='BUTTON')){
    setTimeout(()=>{try{e.target.blur()}catch(err){}},40);
  }
});

document.querySelectorAll('[data-hold]').forEach(b=>{
  const v=b.dataset.hold.split(',').map(Number);
  const on=e=>{e.preventDefault();hold=v};const off=()=>{hold=null};
  b.addEventListener('pointerdown',on);b.addEventListener('pointerup',off);b.addEventListener('pointerleave',off);b.addEventListener('pointercancel',off);
});
document.querySelectorAll('[data-wheel]').forEach(b=>{
  const on=e=>{e.preventDefault();wheel=+b.dataset.wheel};const off=()=>{wheel=null};
  b.addEventListener('pointerdown',on);b.addEventListener('pointerup',off);b.addEventListener('pointerleave',off);b.addEventListener('pointercancel',off);
});

let wasActive=false;
function tick(){
  let x=tx+kx,y=ty+ky,z=rz+kz;
  if(hold){[x,y,z]=hold}
  x=Math.max(-100,Math.min(100,x));y=Math.max(-100,Math.min(100,y));z=Math.max(-100,Math.min(100,z));
  $('tel').textContent=`X${x} Y${y} Z${z}`;
  let url=null;
  if(wheel!==null)url=`/w?i=${wheel}&v=${spd}`;
  else if(x||y||z)url=`/d?x=${x}&y=${y}&z=${z}&s=${spd}`;
  if(url){wasActive=true;if(!busy){busy=true;fetch(url).catch(()=>{}).finally(()=>busy=false)}}
  else if(wasActive){wasActive=false;sendStop()}
}
function sendStop(){for(let i=0;i<3;i++)setTimeout(()=>fetch('/s').catch(()=>{}),i*60)}
function estop(){tx=ty=rz=0;hold=null;wheel=null;$('kT').style.transform='';$('kR').style.transform='';sendStop()}
function mac(id,d){fetch(`/m?id=${id}&d=${d}`).catch(()=>{})}
setInterval(tick,70);

function saveCfg(){
  const q=[0,1,2,3].map(i=>`i${i}=${$('i'+i).checked?1:0}`).join('&');
  fetch(`/cfg?${q}&si=${$('si').checked?1:0}&ri=${$('ri').checked?1:0}&md=${$('md').value}&sp=${$('sp').value}&ac=${$('ac').value}`).then(()=>alert('Saved')).catch(()=>alert('Failed'));
}
fetch('/cfg').then(r=>r.json()).then(c=>{
  [0,1,2,3].forEach(i=>$('i'+i).checked=!!c.i[i]);
  $('si').checked=!!c.si;$('ri').checked=!!c.ri;$('md').value=c.md;$('sp').value=c.sp;$('ac').value=c.ac;
}).catch(()=>{});

let modeNames=[],curMode=-1;
const modeDesc=['Expo-shaped stick + S-curve profile','Low capped speed, extra fine expo','Constant set speed, stick = direction only','Full power, aggressive accel/jerk','Headless: stick axes fixed to start heading (dead-reckoned)','Rotation locked unless hard twist, yaw trim applied','Rotation couples lateral slide for drift turns','Strafe orbits around a point ahead','Skid-steer: left/right turns the rover'];
function drawModes(){$('modes').innerHTML=modeNames.map((n,i)=>`<button class="${i===curMode?'fx':''}" onclick="setMode(${i})">${n}</button>`).join('');$('modeInfo').textContent=modeDesc[curMode]||''}
function setMode(m){fetch('/mode?m='+m);curMode=m;drawModes();try{document.activeElement.blur()}catch(e){}}
function preset(v){spd=v;$('spd').value=v;$('sv').textContent=v;try{document.activeElement.blur()}catch(e){}}
function rot(d){fetch('/rot?deg='+d);try{document.activeElement.blur()}catch(e){}}
['cr','cp','ar','jr','tf','ts'].forEach(k=>{$(k).oninput=e=>{$(k+'v').textContent=e.target.value};$(k).onchange=e=>{fetch(`/mode?${k}=${e.target.value}`);try{e.target.blur()}catch(err){}}});
['sp','md','ac'].forEach(k=>{if($(k))$(k).onchange=e=>{try{e.target.blur()}catch(err){}}});
fetch('/mode').then(r=>r.json()).then(c=>{modeNames=c.names;curMode=c.m;drawModes();['cr','cp','ar','jr','tf','ts'].forEach(k=>{$(k).value=c[k];$(k+'v').textContent=c[k]})}).catch(()=>{});
const names=['FL','FR','RL','RR'];
$('wh').innerHTML=names.map((n,i)=>`<div class="wb">${n} <span id="wv${i}">0</span><div class="meter"><i id="wm${i}"></i></div></div>`).join('');
setInterval(()=>fetch('/status').then(r=>r.json()).then(s=>{
  $('pLink').textContent='RP2040: '+(s.link?'OK':'NO LINK');$('pLink').className='pill '+(s.link?'ok':'bad');
  $('pNet').textContent=s.ip+'  '+s.rssi+'dBm';
  $('pMac').textContent=s.macro?'maneuver':'idle';
  $('pRem').textContent='Remote: '+(s.rem?'OK':'off');$('pRem').className='pill '+(s.rem?'ok':'bad');
  $('pMode').textContent=(modeNames[s.mode]||'-')+'  '+s.hd+'\u00b0';
  if(s.mode!==curMode){curMode=s.mode;drawModes()}
  s.w.forEach((v,i)=>{$('wv'+i).textContent=v;const m=$('wm'+i);m.style.left=v<0?(50+v/2)+'%':'50%';m.style.width=Math.abs(v)/2+'%'});
}).catch(()=>{$('pLink').textContent='ESP offline';$('pLink').className='pill bad'}),600);
</script></body></html>
)rawliteral";
