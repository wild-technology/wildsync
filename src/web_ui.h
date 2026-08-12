// Single-page control panel, served from the binary so there is nothing to
// install and no external asset can fail to load.
#pragma once

static const char* kIndexHtml = R"HTML(<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>ILX-LR1 Control</title>
<style>
  :root{
    --bg:#101215; --panel:#181b20; --panel2:#1f232a; --line:#2c313a;
    --fg:#e8eaed; --dim:#9aa3ad; --accent:#4da3ff; --ok:#3ddc84; --warn:#ffb454; --err:#ff6b6b;
  }
  *{box-sizing:border-box}
  body{margin:0;background:var(--bg);color:var(--fg);
       font:14px/1.45 -apple-system,BlinkMacSystemFont,"SF Pro Text",system-ui,sans-serif}
  header{display:flex;align-items:center;gap:16px;flex-wrap:wrap;
         padding:10px 16px;background:var(--panel);border-bottom:1px solid var(--line);
         position:sticky;top:0;z-index:10}
  header h1{font-size:15px;margin:0;font-weight:600;letter-spacing:.2px}
  .pill{display:inline-flex;align-items:center;gap:6px;padding:3px 10px;border-radius:999px;
        background:var(--panel2);border:1px solid var(--line);font-size:12px;color:var(--dim)}
  .dot{width:8px;height:8px;border-radius:50%;background:var(--err)}
  .dot.on{background:var(--ok)}
  .wrap{display:grid;grid-template-columns:minmax(340px,1fr) minmax(360px,460px);
        gap:14px;padding:14px;align-items:start}
  @media (max-width:900px){.wrap{grid-template-columns:1fr}}
  .card{background:var(--panel);border:1px solid var(--line);border-radius:10px;overflow:hidden}
  .card h2{font-size:12px;text-transform:uppercase;letter-spacing:.8px;color:var(--dim);
           margin:0;padding:10px 14px;border-bottom:1px solid var(--line);background:var(--panel2)}
  .card .body{padding:14px;display:flex;flex-direction:column;gap:12px}
  .row{display:flex;align-items:center;gap:10px;flex-wrap:wrap}
  .row.sb{justify-content:space-between}
  label{color:var(--dim);font-size:12px}
  button{background:var(--panel2);color:var(--fg);border:1px solid var(--line);border-radius:8px;
         padding:9px 14px;font:inherit;font-size:13px;cursor:pointer;
         -webkit-user-select:none;user-select:none;touch-action:none}
  button:hover:not(:disabled){border-color:#3d4552;background:#252a32}
  button:active:not(:disabled){background:var(--accent);border-color:var(--accent);color:#04121f}
  button:disabled{opacity:.4;cursor:not-allowed}
  button.primary{background:var(--accent);border-color:var(--accent);color:#04121f;font-weight:600}
  button.danger{background:#3a1d1d;border-color:#5a2a2a;color:#ffb3b3}
  button.wide{flex:1}
  select,input[type=number]{background:var(--panel2);color:var(--fg);border:1px solid var(--line);
        border-radius:7px;padding:7px 9px;font:inherit;font-size:13px;min-width:0}
  input[type=range]{flex:1;accent-color:var(--accent);min-width:120px}
  .kv{display:grid;grid-template-columns:auto 1fr;gap:4px 12px;font-size:12px}
  .kv .k{color:var(--dim)}
  .kv .v{text-align:right;font-variant-numeric:tabular-nums}
  #lv{width:100%;display:block;background:#000;aspect-ratio:3/2;object-fit:contain}
  .lvmsg{padding:24px;text-align:center;color:var(--dim);font-size:13px}
  #log{margin:0;padding:10px 14px;max-height:150px;overflow:auto;font-size:11.5px;
       font-family:"SF Mono",Menlo,monospace;color:var(--dim);white-space:pre-wrap}
  .msg{padding:8px 12px;border-radius:8px;font-size:12.5px;display:none}
  .msg.err{display:block;background:#3a1d1d;color:#ffb3b3;border:1px solid #5a2a2a}
  .msg.ok{display:block;background:#173028;color:#9fe8c4;border:1px solid #245040}
  .big{font-size:20px;font-variant-numeric:tabular-nums;font-weight:600}
  .hint{font-size:11.5px;color:var(--dim)}
  .bar{height:6px;border-radius:3px;background:var(--panel2);overflow:hidden}
  .bar > i{display:block;height:100%;background:var(--accent);width:0}
</style>
</head>
<body>
<header>
  <h1>ILX-LR1 Control</h1>
  <span class="pill"><span class="dot" id="dot"></span><span id="connText">disconnected</span></span>
  <span class="pill" id="battPill">battery --</span>
  <span class="pill" id="shotsPill">shots --</span>
  <span style="flex:1"></span>
  <button id="btnConnect">Connect</button>
  <button id="btnDisconnect" style="display:none">Disconnect</button>
</header>

<div id="msg" class="msg"></div>

<div class="wrap">
  <div style="display:flex;flex-direction:column;gap:14px">
    <div class="card">
      <h2>Live view</h2>
      <img id="lv" alt="live view" style="display:none">
      <div class="lvmsg" id="lvmsg">Live view starts once the camera is connected</div>
    </div>
    <div class="card">
      <h2>Activity</h2>
      <pre id="log"></pre>
    </div>
  </div>

  <div style="display:flex;flex-direction:column;gap:14px">

    <div class="card">
      <h2>Shutter</h2>
      <div class="body">
        <div class="row">
          <button class="primary wide" id="btnShoot">Take photo</button>
          <label><input type="checkbox" id="useAf"> use AF</label>
        </div>
        <div class="row">
          <label style="width:70px">Save to</label>
          <select id="storeDest" style="flex:1"></select>
        </div>
        <div class="kv">
          <span class="k">Card slot 1</span><span class="v" id="kSlot">--</span>
        </div>
      </div>
    </div>

    <div class="card">
      <h2>Intervalometer</h2>
      <div class="body">
        <div class="row">
          <label>Timed by</label>
          <select id="ivMode" style="flex:1">
            <option value="camera">Camera (Interval REC) - exact</option>
            <option value="host">This app - per-frame feedback</option>
          </select>
        </div>

        <div class="row">
          <label>Interval</label>
          <input type="number" id="ivSec" value="1" min="0.5" step="0.5" style="width:80px"> s
          <label>Frames</label>
          <input type="number" id="ivCount" value="0" min="0" step="1" style="width:80px">
          <span class="hint" id="ivCountHint">0 = until stopped</span>
        </div>
        <div class="row" id="ivDelayRow">
          <label>Start delay</label>
          <input type="number" id="ivDelay" value="1" min="0" step="1" style="width:80px"> s
        </div>

        <div class="row">
          <button class="primary wide" id="btnIvStart">Start</button>
          <button class="danger wide" id="btnIvStop" disabled>Stop</button>
        </div>
        <div class="row sb">
          <span class="big" id="ivCounter">0</span>
          <span class="hint" id="ivState">idle</span>
        </div>
        <div class="bar"><i id="ivBar"></i></div>
        <div class="hint" id="ivNote"></div>
      </div>
    </div>

    <div class="card">
      <h2>Manual focus</h2>
      <div class="body">
        <div class="row">
          <label>Mode</label>
          <select id="focusMode" style="flex:1"></select>
        </div>
        <div class="row">
          <button class="wide" id="btnNear">&#9664; Near</button>
          <button class="wide" id="btnFar">Far &#9654;</button>
        </div>
        <div class="row">
          <label>Step</label>
          <input type="range" id="focusStep" min="1" max="7" value="3">
          <span id="focusStepVal" style="width:1.5em;text-align:right">3</span>
        </div>
        <div class="row">
          <label>Position</label>
          <input type="range" id="focusPos" min="0" max="65535" value="0">
          <span id="focusPosVal" style="width:4em;text-align:right">--</span>
        </div>
        <div class="kv">
          <span class="k">Current position</span><span class="v" id="kFocusCur">--</span>
          <span class="k">Focus indication</span><span class="v" id="kFocusInd">--</span>
        </div>
        <div class="hint">Hold Near/Far to drive the lens. Position slider is absolute
          (release to send).</div>
      </div>
    </div>

    <div class="card">
      <h2>Optical zoom</h2>
      <div class="body">
        <div class="row">
          <label>Zoom mode</label>
          <select id="zoomSetting" style="flex:1"></select>
        </div>
        <div class="row">
          <button class="wide" id="btnWide">&#9664; Wide</button>
          <button class="wide" id="btnTele">Tele &#9654;</button>
        </div>
        <div class="row">
          <label>Speed</label>
          <input type="range" id="zoomSpeed" min="1" max="8" value="3">
          <span id="zoomSpeedVal" style="width:1.5em;text-align:right">3</span>
        </div>
        <div class="row">
          <label>Position</label>
          <input type="range" id="zoomPos" min="0" max="100" value="0">
          <span id="zoomPosVal" style="width:4em;text-align:right">--</span>
        </div>
        <div class="kv">
          <span class="k">Current position</span><span class="v" id="kZoomCur">--</span>
          <span class="k">Zoom scale</span><span class="v" id="kZoomScale">--</span>
          <span class="k">Zoom bar</span><span class="v" id="kZoomBar">--</span>
          <span class="k">Operation</span><span class="v" id="kZoomOp">--</span>
        </div>
      </div>
    </div>

    <div class="card">
      <h2>Exposure</h2>
      <div class="body">
        <div class="row"><label style="width:70px">Mode</label>
          <select id="expProgram" style="flex:1"></select></div>
        <div class="row"><label style="width:70px">Shutter</label>
          <select id="expShutter" style="flex:1"></select></div>
        <div class="row"><label style="width:70px">Aperture</label>
          <select id="expAperture" style="flex:1"></select></div>
        <div class="row"><label style="width:70px">ISO</label>
          <select id="expIso" style="flex:1"></select></div>
        <div class="row"><label style="width:70px">Drive</label>
          <select id="expDrive" style="flex:1"></select></div>
      </div>
    </div>

  </div>
</div>

<script>
const $ = s => document.querySelector(s);
let state = {};
let busySelects = new Set();

function flash(text, isErr){
  const m = $('#msg');
  m.textContent = text;
  m.className = 'msg ' + (isErr ? 'err' : 'ok');
  clearTimeout(flash._t);
  flash._t = setTimeout(() => { m.className = 'msg'; }, isErr ? 6000 : 2500);
}

async function api(path, body){
  try{
    const r = await fetch(path, {
      method: body === undefined ? 'GET' : 'POST',
      headers: {'Content-Type':'application/json'},
      body: body === undefined ? undefined : JSON.stringify(body)
    });
    const j = await r.json();
    if(!r.ok || j.ok === false){ flash(j.error || ('request failed: ' + path), true); return null; }
    return j;
  }catch(e){ flash('' + e, true); return null; }
}

// --- live view ---------------------------------------------------------
const lv = $('#lv');
let lvFails = 0;
function pumpLiveView(){
  if(!state.connected){ setTimeout(pumpLiveView, 700); return; }
  const img = new Image();
  img.onload = () => {
    lv.src = img.src; lv.style.display = 'block'; $('#lvmsg').style.display = 'none';
    lvFails = 0; setTimeout(pumpLiveView, 90);
  };
  img.onerror = () => {
    if(++lvFails > 3){
      lv.style.display = 'none';
      $('#lvmsg').style.display = 'block';
      $('#lvmsg').textContent = 'Live view unavailable';
    }
    setTimeout(pumpLiveView, 900);
  };
  img.src = '/liveview.jpg?t=' + Date.now();
}

// --- press and hold ----------------------------------------------------
// Zoom runs until told to stop, so one command on press is enough. NearFar
// behaves as a step on some bodies, so repeat it while the button is held.
function holdButton(el, onPress, onRelease, repeatMs){
  let timer = null, active = false;
  const start = ev => {
    ev.preventDefault();
    if(active) return;
    active = true;
    onPress();
    if(repeatMs) timer = setInterval(onPress, repeatMs);
  };
  const end = () => {
    if(!active) return;
    active = false;
    if(timer){ clearInterval(timer); timer = null; }
    onRelease();
  };
  el.addEventListener('pointerdown', start);
  el.addEventListener('pointerup', end);
  el.addEventListener('pointerleave', end);
  el.addEventListener('pointercancel', end);
  window.addEventListener('blur', end);
}

// --- select helpers ----------------------------------------------------
function fillSelect(el, choices, current){
  if(busySelects.has(el.id)) return;
  const sig = JSON.stringify(choices) + '|' + current;
  if(el.dataset.sig === sig) return;
  el.dataset.sig = sig;
  el.innerHTML = '';
  if(!choices || !choices.length){
    const o = document.createElement('option');
    o.textContent = '--'; el.appendChild(o); el.disabled = true; return;
  }
  el.disabled = false;
  for(const c of choices){
    const o = document.createElement('option');
    o.value = c.v; o.textContent = c.l;
    el.appendChild(o);
  }
  el.value = current;
}

function bindSelect(el, fn){
  el.addEventListener('focus', () => busySelects.add(el.id));
  el.addEventListener('blur', () => busySelects.delete(el.id));
  el.addEventListener('change', async () => {
    busySelects.delete(el.id);
    el.dataset.sig = '';
    await fn(Number(el.value));
  });
}

// --- range helpers -----------------------------------------------------
function syncRange(el, valEl, range, current, dragging){
  if(range){
    if(Number(el.min) !== range.min) el.min = range.min;
    if(Number(el.max) !== range.max) el.max = range.max;
    if(range.step > 0 && Number(el.step) !== range.step) el.step = range.step;
    el.disabled = false;
  }else{
    el.disabled = true;
  }
  if(!dragging){ el.value = current; }
  valEl.textContent = el.disabled ? '--' : el.value;
}

let focusDragging = false, zoomDragging = false;

// --- status polling ----------------------------------------------------
function render(s){
  state = s;
  $('#dot').className = 'dot' + (s.connected ? ' on' : '');
  $('#connText').textContent = s.connected ? (s.model || 'connected') : 'disconnected';
  $('#btnConnect').style.display = s.connected ? 'none' : '';
  $('#btnDisconnect').style.display = s.connected ? '' : 'none';
  $('#battPill').textContent = 'battery ' +
    (s.connected && s.battery !== null && s.battery !== undefined ? s.battery + '%' : 'ext. power');
  $('#shotsPill').textContent = 'shots ' +
    (s.connected && s.remainingShots !== null ? s.remainingShots : '--');

  for(const id of ['btnShoot','btnIvStart','btnNear','btnFar','btnWide','btnTele'])
    $('#'+id).disabled = !s.connected;

  const iv = s.interval || {};          // host-driven loop
  const cv = s.camIv || {};             // the body's own Interval REC
  const camMode = $('#ivMode').value === 'camera';
  $('#ivDelayRow').style.display = camMode ? '' : 'none';
  $('#ivCountHint').textContent = camMode ? 'frames to shoot' : '0 = until stopped';

  const running = camMode ? !!cv.running : !!iv.running;
  $('#btnIvStart').disabled = !s.connected || running;
  $('#btnIvStop').disabled  = !s.connected || !running;

  if(camMode){
    $('#ivCounter').textContent = cv.shots ?? 0;
    $('#ivState').textContent = !cv.armed ? 'not armed'
      : (cv.running ? 'shooting every ' + cv.intervalSec + 's' : 'armed, waiting');
    $('#ivBar').style.width = (cv.running ? 100 : 0) + '%';
    $('#ivNote').textContent = cv.armed
      ? 'The body is timing this. While armed, the shutter button and most settings are locked.'
      : 'Start will configure and arm the camera, then begin the sequence.';
  }else{
    $('#ivCounter').textContent = iv.taken || 0;
    $('#ivState').textContent = iv.running
      ? ('running - every ' + iv.intervalSec + 's' + (iv.target ? ' of ' + iv.target : ''))
      : (iv.taken ? 'stopped after ' + iv.taken : 'idle');
    $('#ivBar').style.width =
      (iv.target ? Math.min(100, 100 * (iv.taken||0) / iv.target) : 0) + '%';
    $('#ivNote').textContent = 'This app fires each frame, so it needs the USB link up.';
    if(iv.lastError) $('#ivState').textContent += ' - ' + iv.lastError;
  }

  if(!s.connected){ $('#log').textContent = (s.log||[]).join('\n'); return; }

  fillSelect($('#focusMode'), s.focusModes, s.focusMode);
  fillSelect($('#zoomSetting'), s.zoomSettings, s.zoomSetting);
  fillSelect($('#expProgram'), s.programChoices, s.programValue);
  fillSelect($('#expShutter'), s.shutterChoices, s.shutterValue);
  fillSelect($('#expAperture'), s.apertureChoices, s.apertureValue);
  fillSelect($('#expIso'), s.isoChoices, s.isoValue);
  fillSelect($('#expDrive'), s.driveChoices, s.driveValue);
  fillSelect($('#storeDest'), s.storeChoices, s.storeDest);
  $('#kSlot').textContent = s.slotStatus || '--';

  // Focus step range comes from NearFar's reported max.
  const nf = s.nearFarRange;
  if(nf){ $('#focusStep').max = Math.max(1, nf.max); }
  syncRange($('#focusPos'), $('#focusPosVal'), s.focusPosRange, s.focusPos, focusDragging);
  $('#kFocusCur').textContent = s.focusPosCur ?? '--';
  $('#kFocusInd').textContent = focusIndicationLabel(s.focusIndication);

  const zs = s.zoomSpeedRange;
  if(zs){ $('#zoomSpeed').max = Math.max(1, zs.max); }
  syncRange($('#zoomPos'), $('#zoomPosVal'), s.zoomPosRange, s.zoomPos, zoomDragging);
  $('#kZoomCur').textContent = s.zoomPosCur ?? '--';
  $('#kZoomScale').textContent = s.zoomScale ? ('x' + (s.zoomScale/1000).toFixed(2)) : '--';
  const zb = s.zoomBar || {};
  $('#kZoomBar').textContent = zb.total ? (zb.current + '/' + zb.total + '  +' + zb.pos) : '--';
  $('#kZoomOp').textContent = s.zoomOpEnabled ? 'enabled' : 'disabled';

  $('#log').textContent = (s.log||[]).join('\n');
  const lg = $('#log'); lg.scrollTop = lg.scrollHeight;
}

function focusIndicationLabel(v){
  const m = {1:'AF locked',2:'AF not locked',3:'focusing',
             4:'MF in focus',5:'MF near focus',6:'MF out of focus'};
  return m[v] || (v ? '0x'+v.toString(16) : '--');
}

async function poll(){
  const s = await api('/api/status');
  if(s) render(s);
  setTimeout(poll, state.connected ? 700 : 1500);
}

// --- wiring ------------------------------------------------------------
$('#btnConnect').onclick = async () => {
  $('#btnConnect').disabled = true;
  const r = await api('/api/connect', {});
  $('#btnConnect').disabled = false;
  if(r) flash('Connected to ' + r.model, false);
};
$('#btnDisconnect').onclick = () => api('/api/disconnect', {});
$('#btnShoot').onclick = () => api('/api/shutter', {af: $('#useAf').checked});

$('#ivMode').onchange = () => { state = state || {}; render(state); };

// An empty <input type=number> yields '' and Number('') is 0, which must never
// be read as "shoot as fast as possible, forever".
function numField(sel, min, fallback){
  const raw = $(sel).value.trim();
  const v = raw === '' ? NaN : Number(raw);
  if(!Number.isFinite(v) || v < min){
    $(sel).value = fallback;
    return fallback;
  }
  return v;
}

$('#btnIvStart').onclick = async () => {
  const camMode = $('#ivMode').value === 'camera';
  const sec = numField('#ivSec', camMode ? 1 : 0.5, 1);
  const count = numField('#ivCount', 0, camMode ? 10 : 0);
  $('#btnIvStart').disabled = true;
  if(camMode){
    // Configure, arm, then trigger. Settings are locked once armed, so order matters.
    if(!await api('/api/camera-interval/arm', {armed: false})) return;
    if(!await api('/api/camera-interval/config', {
        intervalSec: sec, shots: count,
        startDelaySec: numField('#ivDelay', 0, 1)})) return;
    if(!await api('/api/camera-interval/arm', {armed: true})) return;
    await api('/api/camera-interval/run', {start: true});
  }else{
    await api('/api/interval/start', {intervalSec: sec, count: count, af: $('#useAf').checked});
  }
};

$('#btnIvStop').onclick = async () => {
  if($('#ivMode').value === 'camera'){
    await api('/api/camera-interval/run', {start: false});
    await api('/api/camera-interval/arm', {armed: false});
  }else{
    await api('/api/interval/stop', {});
  }
};

$('#focusStep').oninput = e => $('#focusStepVal').textContent = e.target.value;
$('#zoomSpeed').oninput = e => $('#zoomSpeedVal').textContent = e.target.value;

holdButton($('#btnNear'),
  () => api('/api/focus/drive', {step: -Number($('#focusStep').value)}),
  () => api('/api/focus/drive', {step: 0}), 220);
holdButton($('#btnFar'),
  () => api('/api/focus/drive', {step:  Number($('#focusStep').value)}),
  () => api('/api/focus/drive', {step: 0}), 220);

holdButton($('#btnWide'),
  () => api('/api/zoom/drive', {speed: -Number($('#zoomSpeed').value)}),
  () => api('/api/zoom/drive', {speed: 0}), 0);
holdButton($('#btnTele'),
  () => api('/api/zoom/drive', {speed:  Number($('#zoomSpeed').value)}),
  () => api('/api/zoom/drive', {speed: 0}), 0);

const fp = $('#focusPos');
fp.addEventListener('pointerdown', () => focusDragging = true);
fp.addEventListener('input', () => $('#focusPosVal').textContent = fp.value);
fp.addEventListener('change', async () => {
  await api('/api/focus/position', {value: Number(fp.value)});
  focusDragging = false;
});

const zp = $('#zoomPos');
zp.addEventListener('pointerdown', () => zoomDragging = true);
zp.addEventListener('input', () => $('#zoomPosVal').textContent = zp.value);
zp.addEventListener('change', async () => {
  await api('/api/zoom/position', {value: Number(zp.value)});
  zoomDragging = false;
});

bindSelect($('#focusMode'),   v => api('/api/focus/mode', {mode: v}));
bindSelect($('#zoomSetting'), v => api('/api/zoom/setting', {value: v}));
bindSelect($('#expProgram'),  v => api('/api/exposure', {which:'program',  value: v}));
bindSelect($('#expShutter'),  v => api('/api/exposure', {which:'shutter',  value: v}));
bindSelect($('#expAperture'), v => api('/api/exposure', {which:'aperture', value: v}));
bindSelect($('#expIso'),      v => api('/api/exposure', {which:'iso',      value: v}));
bindSelect($('#expDrive'),    v => api('/api/exposure', {which:'drive',    value: v}));
bindSelect($('#storeDest'),   v => api('/api/store', {value: v}));

// Never leave the lens driving if the page goes away mid-hold.
window.addEventListener('pagehide', () => {
  navigator.sendBeacon('/api/zoom/drive', JSON.stringify({speed:0}));
  navigator.sendBeacon('/api/focus/drive', JSON.stringify({step:0}));
});

poll();
pumpLiveView();
</script>
</body>
</html>
)HTML";
