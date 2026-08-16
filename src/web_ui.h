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
  .shotwrap{position:relative;background:#000;aspect-ratio:3/2;outline:none}
  .shotwrap:focus-visible{outline:2px solid var(--accent);outline-offset:-2px}
  #shotImg{width:100%;height:100%;display:block;object-fit:contain}
  .burn{position:absolute;top:10px;left:10px;background:rgba(4,8,10,.74);
        border:1px solid rgba(255,255,255,.14);border-radius:7px;padding:8px 11px;
        pointer-events:none;font-family:"SF Mono",Menlo,monospace}
  .burn .be{font-size:19px;font-weight:600;font-variant-numeric:tabular-nums;
            text-shadow:0 2px 8px rgba(0,0,0,.9)}
  .burn .bs{font-size:11px;color:var(--dim);margin-top:2px}
  .burn .bn{font-size:13px;font-weight:600;color:var(--accent);margin-top:3px;
            max-width:34ch;overflow-wrap:anywhere}
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
      <h2>Captures</h2>
      <div class="shotwrap" id="shotWrap" tabindex="0">
        <img id="shotImg" alt="" style="display:none">
        <div class="lvmsg" id="shotMsg">No JPEGs yet - shoot a frame</div>
        <div class="burn" id="burn" style="display:none">
          <div class="be" id="burnExp">--</div>
          <div class="bs" id="burnSub"></div>
          <div class="bn" id="burnNote"></div>
        </div>
      </div>
      <div class="body">
        <div class="row">
          <button id="shotPrev" title="previous frame">&larr;</button>
          <button id="shotNext" title="next frame">&rarr;</button>
          <button id="shotLast" title="jump to newest">Newest</button>
          <span style="flex:1"></span>
          <span class="big" id="shotIdx">0 / 0</span>
        </div>
        <div class="row">
          <input id="shotNote" placeholder="tag this frame - e.g. flash 1/32" style="flex:1">
          <button id="shotTag">Tag</button>
          <label class="hint" style="display:flex;align-items:center;gap:5px">
            <input type="checkbox" id="shotFollow" checked> follow
          </label>
        </div>
        <div class="hint">Click the frame, then <b>&larr; &rarr;</b> to step.
          Exposure is read from each file's EXIF; tags are kept in this browser.</div>
      </div>
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

// --- captures ----------------------------------------------------------
// Frames the camera sent to the PC, reviewable without leaving the page. Each
// file is fetched once; its EXIF is parsed from the same bytes the <img> shows,
// so the exposure shown is what the camera recorded, not what we asked for.
let shots = [], shotAt = 0, shotFollow = true;
const shotCache = new Map();   // name -> {url, exp}
const SHOT_KEEP = 12;

function shotTags(){ try { return JSON.parse(localStorage.getItem('ilxTags') || '{}'); }
                     catch(e){ return {}; } }
function setShotTag(name, text){
  const t = shotTags();
  if(text) t[name] = text; else delete t[name];
  try { localStorage.setItem('ilxTags', JSON.stringify(t)); } catch(e){}
}

// Minimal Exif reader: JPEG APP1 -> TIFF IFD0 -> Exif sub-IFD. We only want the
// exposure triple, so this walks tags rather than building a full parser.
function readExif(buf){
  const dv = new DataView(buf);
  const out = {};
  if(dv.getUint16(0) !== 0xFFD8) return out;
  let p = 2;
  while(p < dv.byteLength - 4){
    if(dv.getUint8(p) !== 0xFF) break;
    const marker = dv.getUint8(p + 1), len = dv.getUint16(p + 2);
    if(marker === 0xE1){
      const base = p + 10;                                  // past "Exif\0\0"
      if(base + 8 > dv.byteLength) return out;
      const le = dv.getUint16(base) === 0x4949;
      // Bounds-safe reads: a truncated or malformed APP1 can point an offset
      // (especially a big-type value-offset `vo`) past end-of-buffer, and an
      // unchecked getUint32 there throws RangeError, which would reject loadShot
      // and silently freeze the capture pane. Out-of-range reads return 0.
      const u16 = o => (o >= 0 && o + 2 <= dv.byteLength) ? dv.getUint16(o, le) : 0;
      const u32 = o => (o >= 0 && o + 4 <= dv.byteLength) ? dv.getUint32(o, le) : 0;
      const rat = o => u32(o + 4) ? u32(o) / u32(o + 4) : 0;
      const walk = (off, want) => {
        if(off + 2 > dv.byteLength) return;
        const n = u16(off);
        for(let i = 0; i < n; i++){
          const e = off + 2 + i * 12;
          if(e + 12 > dv.byteLength) return;
          const tag = u16(e), typ = u16(e + 2);
          const big = (typ === 5 || typ === 10);
          const vo = big ? base + u32(e + 8) : e + 8;
          if(want[tag]) want[tag](big ? rat(vo) : u16(vo), vo);
        }
      };
      walk(base + u32(base + 4), {
        0x8769: (_v, vo) => walk(base + u32(vo), {
          0x829A: v => out.exposure = v,
          0x829D: v => out.fnumber  = v,
          0x8827: v => out.iso      = v,
        })
      });
      return out;
    }
    if(marker === 0xD9 || marker === 0xDA) break;
    p += 2 + len;
  }
  return out;
}

function shutterText(sec){
  if(!sec) return '--';
  return sec < 1 ? '1/' + Math.round(1 / sec) : (+sec.toFixed(1)) + '"';
}

async function loadShot(name){
  if(shotCache.has(name)) return shotCache.get(name);
  const res = await fetch('/shot/' + encodeURIComponent(name));
  if(!res.ok) throw new Error('shot ' + name + ': HTTP ' + res.status);
  const buf = await res.arrayBuffer();
  // Never cache a non-JPEG body (e.g. a transient 404 JSON) — it would show a
  // broken image under this name for the life of the page. Let the caller retry.
  if(buf.byteLength < 2 || new DataView(buf).getUint16(0) !== 0xFFD8)
    throw new Error('shot ' + name + ': not a JPEG yet');
  const ex = readExif(buf);
  const entry = {
    url: URL.createObjectURL(new Blob([buf], {type:'image/jpeg'})),
    exp: [shutterText(ex.exposure),
          ex.fnumber ? 'f/' + (+ex.fnumber.toFixed(1)) : '--',
          ex.iso ? 'ISO ' + ex.iso : '--'].join('   ')
  };
  shotCache.set(name, entry);
  while(shotCache.size > SHOT_KEEP){                     // blobs are not free
    const oldest = shotCache.keys().next().value;
    URL.revokeObjectURL(shotCache.get(oldest).url);
    shotCache.delete(oldest);
  }
  return entry;
}

async function showShot(){
  const s = shots[shotAt];
  $('#shotIdx').textContent = shots.length ? (shotAt + 1) + ' / ' + shots.length : '0 / 0';
  if(!s){
    $('#shotImg').style.display = 'none'; $('#burn').style.display = 'none';
    $('#shotMsg').style.display = 'block'; return;
  }
  $('#shotMsg').style.display = 'none';
  let e;
  try {
    e = await loadShot(s.name);
  } catch (err) {
    // A frame still being written 404s or arrives non-JPEG; leave the pane as
    // is and let the next poll retry rather than crashing the update loop.
    return;
  }
  if(shots[shotAt] !== s) return;                        // moved on while loading
  $('#shotImg').src = e.url; $('#shotImg').style.display = 'block';
  $('#burn').style.display = 'block';
  $('#burnExp').textContent = e.exp;
  $('#burnSub').textContent = s.name + '   ' + Math.round(s.size / 1024) + ' KB';
  $('#burnNote').textContent = shotTags()[s.name] || '';
  $('#shotNote').value = shotTags()[s.name] || '';
  for(const n of [shotAt + 1, shotAt - 1])
    if(shots[n]) loadShot(shots[n].name).catch(() => {});   // prefetch, ignore
}

function stepShot(d){
  if(!shots.length) return;
  shotFollow = false; $('#shotFollow').checked = false;
  shotAt = Math.max(0, Math.min(shots.length - 1, shotAt + d));
  showShot();
}

async function pollShots(){
  try {
    const next = await (await fetch('/api/shots')).json();
    const grew = next.length !== shots.length;
    const cur = shots[shotAt] && shots[shotAt].name;
    shots = next;
    if(shotFollow && grew) shotAt = shots.length - 1;
    else {
      const at = shots.findIndex(s => s.name === cur);
      shotAt = at >= 0 ? at : Math.min(shotAt, shots.length - 1);
    }
    if(shotAt < 0) shotAt = 0;
    if(grew || !$('#shotImg').src) showShot();
  } catch(e){ /* server busy; next tick */ }
  setTimeout(pollShots, 1200);
}

$('#shotPrev').onclick = () => stepShot(-1);
$('#shotNext').onclick = () => stepShot(1);
$('#shotLast').onclick = () => { shotAt = shots.length - 1; showShot(); };
$('#shotFollow').onchange = e => { shotFollow = e.target.checked;
                                   if(shotFollow){ shotAt = shots.length - 1; showShot(); } };
$('#shotTag').onclick = () => {
  const s = shots[shotAt]; if(!s) return;
  setShotTag(s.name, $('#shotNote').value.trim()); showShot();
};
$('#shotNote').addEventListener('keydown', e => {
  if(e.key === 'Enter'){ $('#shotTag').click(); e.preventDefault(); }
  e.stopPropagation();                                   // digits are not frame steps
});
addEventListener('keydown', e => {
  if(/^(INPUT|SELECT|TEXTAREA)$/.test(document.activeElement.tagName)) return;
  if(e.key === 'ArrowLeft'){ stepShot(-1); e.preventDefault(); }
  else if(e.key === 'ArrowRight'){ stepShot(1); e.preventDefault(); }
});

poll();
pumpLiveView();
pollShots();
</script>
</body>
</html>
)HTML";
