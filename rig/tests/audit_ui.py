#!/usr/bin/env python3
"""Audit regression suite - ui lane (rig/rig_ui.html), findings U1-U15.

rig_ui.html is browser JavaScript, so this suite does what soaktest does for
the Python side: it runs the REAL file. The page's <script> is loaded into
JavaScriptCore (via `osascript -l JavaScript`, which ships with macOS) on top
of the smallest DOM the script needs, and the functions under test are then
called with hand-built rig state. Nothing is re-implemented here; a failing
check means rig_ui.html behaves that way.

What each group pins, and the defect it reproduces on the pre-fix file:

U1  pickRunWithFrames - Review chose /api/runs' newest entry unconditionally,
    so a run started and stopped before a frame landed pinned the pane to an
    empty list ("looking for frames..." with no way out).
U2  navFixKind/renderNav - the operator's armed STATIC fix wore the same green
    "nav fix" pill as a live GPS fix, and SOG read s.sog while the API emits
    sog_mps (so it was always "-").
U3  syncApplyBtn - "Apply cam2's exposure" stayed enabled after cam2 went
    offline and then silently applied cam1's exposure instead.
U4  fmtPushOk - a dropped fetch or a rigd 500 took the SUCCESS branch: green
    "pushed", dirty cleared, cameras still on the old format.
U5  frameImg - the pair <img> was shared with live view, so live OFF left a
    live frame captioned as the recorded survey file.
U6  jitChip - a one-camera shot printed a green measured "0.00ms" beside the
    MISSING panel.
U7  strobeFastCams/startInputsSig - the strobe check read the fleet desired
    vector, so a per-camera 1/250 split sailed through; and "Start anyway" had
    no expiry.
U8  the keydown handler stepped the pair cursor for arrows typed into the
    Label/interval inputs, silently dropping follow mode mid-run.
U10 healthStrip/renderFmtReadback - the node-vs-host clock offset was rendered
    nowhere, and there was no per-camera format readback at all.
U11 renderAnoms - an anomaly kind with no msg printed "undefined" at the
    operator.
U13 healthStrip - the clock chip fired on clock_offset_ms, the LAST RAW
    /health sample, not on the RTT-filtered median every other consumer reads:
    one poll's RTT spike raised an amber "clk +85 ms" on a chrony-locked node,
    and a stale raw sample could not expire.
U14 renderFmtReadback - a format field the body did not report was dropped
    from the row instead of marked, so a PARTIAL answer rendered as a
    confident complete one on the panel the format push tells the operator to
    confirm against.
U15 the fleet exposure apply - only a COMPLETELY empty body was refused, so a
    partial one was posted and reported with the same green line as a full
    apply (and the verdict was then erased by the hydrate that follows it).

U12 the card drain - the flow that gets the irreplaceable RAWs off the cards,
    blocks the next transect while it runs and can wedge a body - had NO UI at
    all: /api/drain, /api/drain/cancel and the whole wedge state were reachable
    only with curl. There was no progress, no cancel, no per-node result, and
    nothing anywhere said why Start refused during a drain.

Hermetic: no network, no rigd, no node, no camera. The only subprocess is
osascript on a temp file. The executable half needs macOS; everywhere else it
degrades to the structural checks below and says so.

Run standalone:  python3 rig/tests/audit_ui.py
"""

import json
import os
import platform
import re
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
RIG = os.path.dirname(HERE)
sys.path.insert(0, RIG)
sys.path.insert(0, HERE)

from soaktest import check, note, sect          # noqa: E402

UI_PATH = os.path.join(RIG, "rig_ui.html")
JS_TIMEOUT_S = 90

SHIM = r"""// ---------------------------------------------------------------------------
// The smallest DOM rig_ui.html can load against. Not a browser: just enough
// object surface for the script's top level to run and for the pure/render
// functions under test to be called. Anything the tests do not exercise is a
// no-op, on purpose — a richer fake would be a second implementation to be
// wrong in its own way.
// ---------------------------------------------------------------------------
var __IDTAG = __IDTAG__;          // id -> tagName, read out of the real markup
var __PREFIX = [[/^(ap|sh|iso|zset)-/, 'SELECT'],
                [/^(fstep|zspd|fpos|zpos)-/, 'INPUT'],
                [/^(lv)-/, 'IMG']];
function __tagFor(id){
  if (__IDTAG[id]) return __IDTAG[id];
  for (var i=0;i<__PREFIX.length;i++) if (__PREFIX[i][0].test(id)) return __PREFIX[i][1];
  return 'DIV';
}
function __ctx2d(){
  var noop=function(){ return {width:0}; };
  var grad=function(){ return {addColorStop:function(){}}; };
  return {clearRect:noop, beginPath:noop, moveTo:noop, lineTo:noop, stroke:noop,
          arc:noop, fill:noop, save:noop, restore:noop, translate:noop,
          rotate:noop, fillText:noop, closePath:noop, setLineDash:noop,
          measureText:noop, rect:noop, strokeRect:noop, fillRect:noop, clip:noop,
          createLinearGradient:grad, createRadialGradient:grad,
          strokeStyle:'', fillStyle:'', lineWidth:1, font:'', textAlign:''};
}
function El(tag, id){
  this.tagName = String(tag||'DIV').toUpperCase();
  this.id = id || '';
  this._cls = {}; this._html = ''; this._kids = [];
  this.dataset = {}; this.style = {}; this.options = [];
  this.value = ''; this.checked = false; this.disabled = false;
  this.title = ''; this.textContent = ''; this.src = undefined;
  this.onclick = null; this.onchange = null; this.oninput = null;
  this.onload = null; this.onerror = null;
  this.width = 180; this.height = 180; this.min = 0; this.max = 65535;
  this._parent = null;
  var self = this;
  this.classList = {
    add:      function(c){ self._cls[c] = 1; },
    remove:   function(c){ delete self._cls[c]; },
    toggle:   function(c,on){ if(on===undefined) on = !self._cls[c];
                              if(on) self._cls[c]=1; else delete self._cls[c]; },
    contains: function(c){ return !!self._cls[c]; }
  };
}
Object.defineProperty(El.prototype,'className',{
  get: function(){ return Object.keys(this._cls).join(' '); },
  set: function(v){ this._cls={}; var self=this;
    String(v||'').split(/\s+/).forEach(function(c){ if(c) self._cls[c]=1; }); }
});
Object.defineProperty(El.prototype,'children',{ get:function(){ return this._kids; } });
function __parseMini(html, parent){
  // Recognises only the tags the pair viewer builds by string: <img> and the
  // LIVE burn <div>. Enough for frameImg's contract, nothing more.
  var out=[], re=/<(img|div|span)\b([^>]*)>/gi, m;
  while((m = re.exec(html))){
    var e = new El(m[1]);
    var cm = /class="([^"]*)"/.exec(m[2]||'');
    if(cm) e.className = cm[1];
    e._parent = parent;
    out.push(e);
  }
  return out;
}
Object.defineProperty(El.prototype,'innerHTML',{
  get: function(){ return this._html; },
  set: function(v){ this._html = String(v); this._kids = __parseMini(this._html, this);
    // A real DOM drops the <option> children with the rest of the markup;
    // without this, a select rebuilt from a new node list kept the old options.
    if(this.tagName==='SELECT') this.options = []; }
});
El.prototype.insertAdjacentHTML = function(pos, html){
  this._html += String(html);
  this._kids = this._kids.concat(__parseMini(String(html), this));
};
El.prototype.querySelector = function(sel){
  for(var i=0;i<this._kids.length;i++){
    var k = this._kids[i];
    if(sel === 'img'   && k.tagName === 'IMG') return k;
    if(sel === '.burn' && k.classList.contains('burn')) return k;
    if(sel === '.fill' && k.classList.contains('fill')) return k;
  }
  if(sel === '.fill'){ var f = new El('div'); f.className='fill'; f._parent=this;
                       this._kids.push(f); return f; }
  return null;
};
El.prototype.querySelectorAll = function(){ return []; };
El.prototype.appendChild = function(c){ c._parent=this; this._kids.push(c);
  if(this.tagName==='SELECT') this.options.push(c); return c; };
El.prototype.removeChild = function(c){ var i=this._kids.indexOf(c);
  if(i>=0) this._kids.splice(i,1);
  var j=this.options.indexOf(c); if(j>=0) this.options.splice(j,1); };
El.prototype.remove = function(){ if(this._parent) this._parent.removeChild(this); };
El.prototype.removeAttribute = function(a){ if(a==='src') this.src = undefined; };
El.prototype.setAttribute = function(){};
El.prototype.addEventListener = function(){};
El.prototype.getContext = function(){ return __ctx2d(); };
El.prototype.scrollIntoView = function(){};
El.prototype.focus = function(){ document.activeElement = this; };

var __REG = {}, __H = {};
var document = {
  hidden: false,
  activeElement: null,
  body: new El('body'),
  createElement: function(t){ return new El(t); },
  addEventListener: function(t, fn){ (__H[t] = __H[t] || []).push(fn); },
  querySelectorAll: function(){ return []; },
  querySelector: function(sel){
    // The one compound selector the script really asks for.
    if(sel === '#ctlCols select.edited'){
      for(var k in __REG){
        var e = __REG[k];
        if(e.tagName==='SELECT' && /^#(ap|sh|iso)-/.test(k)
           && e.classList.contains('edited')) return e;
      }
      return null;
    }
    if(__REG[sel]) return __REG[sel];
    var id = sel.charAt(0)==='#' ? sel.slice(1) : sel;
    var el = new El(__tagFor(id), id);
    __REG[sel] = el;
    return el;
  }
};
function Image(){ this.src=''; this.onload=null; this.onerror=null; }
function AbortController(){ this.signal={}; this.abort=function(){}; }
function fetch(){ return new Promise(function(){}); }   // never settles
function setTimeout(){ return 0; }      function clearTimeout(){}
function setInterval(){ return 0; }     function clearInterval(){}
function requestAnimationFrame(){ return 0; }
function cancelAnimationFrame(){}
"""

DRIVER = r"""// ---------------------------------------------------------------------------
// Drives the loaded rig_ui.html script. Every entry pushed onto __R becomes one
// soaktest check. Names are prefixed with the lane finding they pin.
// ---------------------------------------------------------------------------
var __R = [];
function T(name, ok, detail){ __R.push({name:name, ok:!!ok, detail:String(detail==null?'':detail)}); }
function TEQ(name, got, want){ T(name, got===want, 'got ' + JSON.stringify(got) + ', want ' + JSON.stringify(want)); }
function THAS(name, hay, needle){
  hay = String(hay); var ok = hay.indexOf(needle) >= 0;
  T(name, ok, (ok ? 'found ' : 'MISSING ') + JSON.stringify(needle) +
    ' in ' + JSON.stringify(hay.slice(0,240)));
}
function TNOT(name, hay, needle){
  hay = String(hay); var ok = hay.indexOf(needle) < 0;
  T(name, ok, (ok ? 'absent as required: ' : 'PRESENT: ') + JSON.stringify(needle) +
    ' in ' + JSON.stringify(hay.slice(0,240)));
}
var SH = function(num, den){ return ((num & 0xFFFF) << 16) | (den & 0xFFFF); };
function node(o){
  var n = {name:'cam1', cam_num:1, connected:true, state:'CAM_CONNECTED',
           battery:80, disk_free_mb:9000, slot_writing_label:'idle',
           gpio:{available:true, edges_seen:4}, aperture_raw:1100,
           shutter_raw:SH(1,60), iso_raw:1600};
  for(var k in o) n[k] = o[k];
  return n;
}

// ---- U6  jitter chip must never assert 0 ms for a one-camera shot ---------
try{
  var one = {files:{cam1:'Cam1_a.jpg'}, missing:['cam2'], spread_ms:0,
             spread_src:'index', srcs:{cam1:'gpio_edge'}};
  var h = jitChip(one);
  TNOT('U6 one-camera shot shows no spread number', h, '0.00ms');
  THAS('U6 one-camera shot names the missing camera', h, 'missing cam2');
  var pair = {files:{cam1:'a.jpg', cam2:'b.jpg'}, missing:[], spread_ms:3.2,
              spread_src:'index', srcs:{cam1:'gpio_edge', cam2:'gpio_edge'}};
  var hp = jitChip(pair);
  THAS('U6 real pair still shows its measured spread', hp, '3.20ms');
  THAS('U6 real pair still grades the spread', hp, 'jit ok');
}catch(e){ T('U6 jitChip', false, 'threw ' + e); }

// ---- U1  Review picks the newest run that HAS frames ---------------------
try{
  TEQ('U1 skips a 0-frame newest run for an older one with frames',
      pickRunWithFrames([{run_id:'260823_1500_b', frames:0},
                         {run_id:'260823_1400_a', frames:12}]), '260823_1400_a');
  TEQ('U1 no run has frames -> empty pick (caller falls to the spool)',
      pickRunWithFrames([{run_id:'260823_1500_b', frames:0}]), '');
  TEQ('U1 unknown frame count with cam dirs is still offered',
      pickRunWithFrames([{run_id:'260823_1500_b', frames:0},
                         {run_id:'260823_1400_a', frames:null, cams:['cam1']}]),
      '260823_1400_a');
  TEQ('U1 empty listing -> empty pick', pickRunWithFrames([]), '');
}catch(e){ T('U1 pickRunWithFrames', false, 'threw ' + e); }

// ---- U2  a STATIC fix is not a GPS fix -----------------------------------
try{
  TEQ('U2 armed static fix is classed static',
      navFixKind({present:true, valid:false, lat:-17.5, lon:177.3, static_fix:'quay'}), 'static');
  TEQ('U2 live fix is classed live',
      navFixKind({present:true, valid:true, lat:-17.5, lon:177.3}), 'live');
  TEQ('U2 no position at all is classed none',
      navFixKind({present:true, valid:false}), 'none');
  TEQ("U2 rigd's own fix_kind wins over the derivation",
      navFixKind({present:true, valid:false, lat:1, lon:2, fix_kind:'live'}), 'live');

  track.length = 0;
  renderNav({present:true, valid:false, lat:-17.5, lon:177.3, static_fix:'quay wall',
             sog_mps:1.25, sats:0, depth_m:3.2});
  TEQ('U2 header pill reads STATIC, not fix', document.querySelector('#hNav').textContent, 'nav STATIC');
  THAS('U2 header pill wears the warn colour', document.querySelector('#hNav').className, 'warn');
  THAS('U2 Nav tab labels the source as STATIC', document.querySelector('#navKv').innerHTML, 'STATIC fix');
  THAS('U2 Nav tab names the static fix', document.querySelector('#navKv').innerHTML, 'quay wall');
  TEQ('U2 a static fix is never plotted as a track', track.length, 0);
  THAS('U3/U2 SOG is read from sog_mps, not sog',
       document.querySelector('#navKv').innerHTML, '1.25 m/s');

  track.length = 0;
  renderNav({present:true, valid:true, lat:-17.5, lon:177.3, sog_mps:0.4, sats:11});
  TEQ('U2 a live fix still reads "nav fix"', document.querySelector('#hNav').textContent, 'nav fix');
  THAS('U2 a live fix still wears the ok colour', document.querySelector('#hNav').className, 'on');
  TEQ('U2 a live fix is plotted', track.length, 1);
}catch(e){ T('U2 nav', false, 'threw ' + e); }

// ---- U4  "pushed" only when something was actually applied ---------------
try{
  TEQ('U4 a dropped fetch is not a successful push',
      fmtPushOk({ok:false, error:'TypeError: Failed to fetch', _err:true}), false);
  TEQ('U4 a rigd 500 is not a successful push',
      fmtPushOk({ok:false, error:'boom'}), false);
  TEQ('U4 an empty applied set is not a successful push',
      fmtPushOk({ok:true, applied:{}}), false);
  TEQ('U4 a real apply is a successful push',
      fmtPushOk({ok:true, applied:{filetype:3}, rejected:{}}), true);
  TEQ('U4 no response at all is not a successful push', fmtPushOk(null), false);
}catch(e){ T('U4 fmtPushOk', false, 'threw ' + e); }

// ---- U7  strobe check reads each BODY's shutter, not the fleet desired ----
try{
  // The finding's scenario exactly: fleet desired is 1/30 (slow enough for the
  // strobe), the operator has put cam2 alone on 1/250.
  var split = strobeFastCams(
    [node({name:'cam1', shutter_raw:SH(1,30)}), node({name:'cam2', shutter_raw:SH(1,250)})],
    {shutter:SH(1,30)});
  TEQ('U7 a per-camera split is caught even when desired is slow', split.length, 1);
  THAS('U7 the offending camera is named', split.join(','), 'cam2');
  THAS('U7 the offending shutter is shown', split.join(','), '1/250');
  TEQ('U7 both bodies slow -> no warning',
      strobeFastCams([node({name:'cam1', shutter_raw:SH(1,30)}),
                      node({name:'cam2', shutter_raw:SH(1,15)})], {shutter:SH(1,250)}).length, 0);
  var nb = strobeFastCams([node({name:'cam1', shutter_raw:null})], {shutter:SH(1,250)});
  TEQ('U7 with no readback anywhere it falls back to desired', nb.length, 1);
  THAS('U7 the fallback says it is the desired vector', nb.join(','), 'fleet desired');
}catch(e){ T('U7 strobeFastCams', false, 'threw ' + e); }

// ---- U8  arrow keys in a Review input must not step the pair cursor -------
try{
  var handlers = __H['keydown'] || [];
  T('U8 a keydown handler is registered', handlers.length > 0, handlers.length + ' handler(s)');
  if(handlers.length){
    TAB = 'review';
    PAIRS = {src:'run', rid:'r', total:10, start:0,
             list:[1,2,3,4,5,6,7,8,9,10], cursor:9, follow:true};
    handlers[0]({key:'ArrowLeft', target:{tagName:'INPUT'}});
    TEQ('U8 ArrowLeft in a text input leaves the cursor alone', PAIRS.cursor, 9);
    TEQ('U8 ArrowLeft in a text input does not drop follow mode', PAIRS.follow, true);
    handlers[0]({key:'ArrowLeft', target:{tagName:'SELECT'}});
    TEQ('U8 ArrowLeft in a select leaves the cursor alone', PAIRS.cursor, 9);
    handlers[0]({key:'ArrowLeft', target:{tagName:'BODY'}});
    TEQ('U8 ArrowLeft outside an input still steps the pair', PAIRS.cursor, 8);
    TEQ('U8 stepping off the newest pair drops follow', PAIRS.follow, false);
  }
}catch(e){ T('U8 arrow keys', false, 'threw ' + e); }

// ---- U5  live view and recorded frames must not share a stale <img> -------
try{
  var fr = new El('div');
  fr.innerHTML = '<img alt="">';
  var img = fr.querySelector('img');
  img.dataset.name = 'Cam1_20260823_101500.00.jpg';
  img.src = '/api/run/frame?id=r&cam=cam1&name=Cam1_20260823_101500.00.jpg';

  var li = frameImg(fr, true);                    // operator turns live view ON
  T('U5 live ON adds the LIVE badge', !!fr.querySelector('.burn'), 'burn present');
  TEQ('U5 live ON drops the recorded filename latch', li.dataset.name, '');
  TEQ('U5 live ON clears the recorded frame src', li.src, undefined);

  li.src = '/api/liveview?node=cam1&_=1';         // a live frame arrives
  var re = frameImg(fr, false);                   // operator turns live view OFF
  T('U5 live OFF removes the LIVE badge', !fr.querySelector('.burn'), 'burn gone');
  TEQ('U5 live OFF forces the recorded frame to reload', re.dataset.name, '');
  TEQ('U5 live OFF clears the live frame src', re.src, undefined);

  frameImg(fr, true); var b1 = fr.querySelector('.burn');
  frameImg(fr, true); var b2 = fr.querySelector('.burn');
  T('U5 staying live does not restart the stream every render', b1 === b2, 'badge kept');
}catch(e){ T('U5 frameImg', false, 'threw ' + e); }

// ---- U3  Apply names its source and refuses an offline one ---------------
try{
  var A = document.querySelector('#btnApply');

  FLEET = {nodes:[node({name:'cam1'}), node({name:'cam2', connected:false})], run:{}};
  LAST_EDITED = 'cam2'; APPLY_BUSY = false;
  syncApplyBtn();
  TEQ('U3 Apply is disabled when the edited camera is offline', A.disabled, true);
  THAS('U3 Apply still names the camera it promised', A.textContent, 'cam2');
  THAS('U3 Apply says the camera is offline', A.textContent, 'OFFLINE');

  FLEET = {nodes:[node({name:'cam1', iso_raw:1600}),
                  node({name:'cam2', iso_raw:3200})], run:{}};
  LAST_EDITED = null; syncApplyBtn();
  TEQ('U3 Apply is enabled with both bodies up', A.disabled, false);
  THAS('U3 with the bodies split, Apply names the camera it would read', A.textContent, 'cam1');

  FLEET = {nodes:[node({name:'cam1'}), node({name:'cam2'})], run:{}};
  LAST_EDITED = null; syncApplyBtn();
  TEQ('U3 once the bodies match the label goes back to generic',
      A.textContent, 'Apply exposure to fleet');

  LAST_EDITED = 'cam2'; syncApplyBtn();
  TEQ('U3 a finished per-camera promise is released', LAST_EDITED, null);

  FLEET = {nodes:[node({name:'cam1', connected:false}),
                  node({name:'cam2', connected:false})], run:{}};
  LAST_EDITED = null; syncApplyBtn();
  TEQ('U3 Apply is disabled with no camera connected', A.disabled, true);
}catch(e){ T('U3 syncApplyBtn', false, 'threw ' + e); }

// ---- U10 the node/host clock offset is visible on the health strip -------
// The fleet vector carries BOTH numbers per node (rigd publishes them side by
// side on purpose): clock_offset_ms is the last RAW /health sample, and
// clock_offset_s / clock_offset_info is the RTT-gated median every decision is
// made on. WHICH one drives the chip is U13 below; that an offset is shown at
// all is U10.
function clocked(o){
  var c = {clock_offset_ms:null, clock_offset_s:null,
           clock_offset_info:{offset_s:null, n:0, rtt_ms_best:null, age_s:null}};
  for(var k in (o||{})) c[k] = o[k];
  return c;
}
try{
  var far = healthStrip(node(clocked({clock_offset_ms:190.2, clock_offset_s:0.1874,
    clock_offset_info:{offset_s:0.1874, n:8, rtt_ms_best:2.9, age_s:0.4}})));
  THAS('U10 a 187 ms host offset is shown on the strip', far, 'clk +187 ms');
  THAS('U10 the offset chip wears the warn tint', far, 'hwarn');
  var neg = healthStrip(node(clocked({clock_offset_ms:-91.3, clock_offset_s:-0.0903,
    clock_offset_info:{offset_s:-0.0903, n:8, rtt_ms_best:3.0, age_s:0.4}})));
  THAS('U10 a negative offset keeps its sign', neg, 'clk -90 ms');
  TNOT('U10 a chrony-tight node gets no chip',
       healthStrip(node(clocked({clock_offset_ms:12.0, clock_offset_s:0.012,
         clock_offset_info:{offset_s:0.012, n:8, rtt_ms_best:2.9, age_s:0.4}}))), 'clk ');
  TNOT('U10 an unmeasured offset gets no chip', healthStrip(node(clocked({}))), 'clk ');
}catch(e){ T('U10 healthStrip', false, 'threw ' + e); }

// ---- U13 the clock chip reads the FILTERED offset, never the raw sample ---
// The chip fired on n.clock_offset_ms, the LAST RAW /health sample. rigd's own
// anomaly code refuses to decide anything on that number and says why: one
// poll's RTT lands whole in the midpoint estimate, and the live journal shows
// single-scan "82.9 ms" alarms while chronyc had the two Pis 20 us apart. The
// chip's bar is 50 ms, so a healthy chrony-locked fleet raised it mid-transect.
try{
  var spike = healthStrip(node(clocked({clock_offset_ms:85.0, clock_offset_s:0.005,
    clock_offset_info:{offset_s:0.005, n:8, rtt_ms_best:2.9, age_s:0.4}})));
  TNOT('U13 one poll of network noise raises no chip', spike, 'clk ');
  var real = healthStrip(node(clocked({clock_offset_ms:4.0, clock_offset_s:0.1871,
    clock_offset_info:{offset_s:0.1871, n:8, rtt_ms_best:2.9, age_s:0.4}})));
  THAS('U13 a real offset the raw sample understates still raises the chip',
       real, 'clk +187 ms');
  THAS('U13 the raw sample is kept, in the title rigd publishes it for',
       real, 'last raw sample 4 ms');
  THAS('U13 the chip says what window it is the median of', real, 'median of 8 samples');
  TNOT('U13 a one-sample window is not dressed up as a filtered figure',
       healthStrip(node(clocked({clock_offset_ms:85.0, clock_offset_s:0.085,
         clock_offset_info:{offset_s:0.085, n:1, rtt_ms_best:2.9, age_s:0.2}}))), 'clk ');
  // Over a link this slow the midpoint estimate is network, not clock — the
  // same call rigd's node_clock_unmeasurable detector makes.
  var slow = healthStrip(node(clocked({clock_offset_ms:60.0, clock_offset_s:0.06,
    clock_offset_info:{offset_s:0.06, n:6, rtt_ms_best:45.0, age_s:0.5}})));
  THAS('U13 a link too slow to measure a clock over says so', slow, 'unmeasurable');
  THAS('U13 the unmeasurable chip names the link', slow, 'link 45 ms');
  TNOT('U13 the unmeasurable chip does not print the network as a clock error',
       slow, 'vs host');
  // piagent gone while ilxctl stays up: the raw field freezes where it was,
  // the filter expires at 60 s. A frozen number is not an offset.
  TNOT('U13 a stale raw sample with no filtered figure raises no chip',
       healthStrip(node(clocked({clock_offset_ms:187.4}))), 'clk ');
  TNOT('U13 a clock field that is not a number cannot reach the markup',
       healthStrip(node(clocked({clock_offset_s:'<img src=x>',
         clock_offset_info:{n:'<b>8</b>', rtt_ms_best:'<i>2</i>'}}))), '<img src=x');
}catch(e){ T('U13 clock chip', false, 'threw ' + e); }

// ---- U11 new anomaly kinds render generically and are escaped ------------
try{
  ANOMS = [{kind:'host_clock_offset', sev:'warn',
            msg:'host clock is 187 ms behind the nodes',
            suggested_action:'discipline the host clock'}];
  renderAnoms();
  var ah = document.querySelector('#anoms').innerHTML;
  THAS('U11 host_clock_offset renders', ah, 'host_clock_offset');
  THAS('U11 its message renders', ah, '187 ms behind');
  THAS('U11 its suggested action renders', ah, 'discipline the host clock');

  ANOMS = [{kind:'node_clock_unmeasurable', node:'cam2'},
           {kind:'nav_no_fix'}];
  renderAnoms();
  var ah2 = document.querySelector('#anoms').innerHTML;
  THAS('U11 node_clock_unmeasurable renders', ah2, 'node_clock_unmeasurable');
  THAS('U11 nav_no_fix renders', ah2, 'nav_no_fix');
  TNOT('U11 a kind with no message prints no "undefined"', ah2, 'undefined');

  ANOMS = [{kind:'nav_no_fix', msg:'<script>x</script>'}];
  renderAnoms();
  var ah3 = document.querySelector('#anoms').innerHTML;
  TNOT('U11 anomaly text is HTML-escaped', ah3, '<script>');
  THAS('U11 anomaly text is escaped, not dropped', ah3, '&lt;script&gt;');
}catch(e){ T('U11 anomalies', false, 'threw ' + e); }

// ---- U10 per-camera format readback panel --------------------------------
try{
  FLEET = {nodes:[node({name:'cam1'}), node({name:'cam2'})], run:{}};
  FMT_RB['cam1'] = {at:Date.now(), s:{filetypeLabel:'RAW+JPEG', imagesizeLabel:'L',
                                      transsizeLabel:'Original', rawtypeLabel:'LossLessL',
                                      qualityLabel:'Fine', pcsaveLabel:'RAW+JPEG'}};
  FMT_RB['cam2'] = {at:Date.now(), s:{connected:true}};      // ilxctl too old
  renderFmtReadback();
  var fh = document.querySelector('#fmtReadback').innerHTML;
  THAS('U10 a body that reports its format shows it', fh, 'LossLessL');
  THAS('U10 the file type label is shown', fh, 'RAW+JPEG');
  THAS('U10 an older ilxctl gets an explicit no-readback badge', fh, 'no readback');
  FMT_RB['cam2'] = {at:Date.now(), err:'timed out'};
  renderFmtReadback();
  var fh2 = document.querySelector('#fmtReadback').innerHTML;
  THAS('U10 an unreachable body is not called "older ilxctl"', fh2, 'status unreadable');
  THAS('U10 the unreachable reason is shown', fh2, 'timed out');
}catch(e){ T('U10 renderFmtReadback', false, 'threw ' + e); }

// ---- U14 a format field the body withheld reads as unconfirmed -----------
// camera.cpp emits <name>Value/<name>Label per property and omits the pair
// when the read came back < 0, so fields go missing ONE at a time. The row
// dropped those silently and only badged the all-six case, so a body that
// withheld filetype rendered as a full, confident line — on the very panel the
// format push's success message tells the operator to confirm against.
try{
  FLEET = {nodes:[node({name:'cam1'}), node({name:'cam2'})], run:{}};
  var six = {filetypeLabel:'RAW+JPEG', imagesizeLabel:'S', transsizeLabel:'Small',
             rawtypeLabel:'LossLessL', qualityLabel:'Fine', pcsaveLabel:'RAW+JPEG'};
  var five = {imagesizeLabel:'S', transsizeLabel:'Small', rawtypeLabel:'LossLessL',
              qualityLabel:'Fine', pcsaveLabel:'RAW+JPEG'};   // filetype withheld
  FMT_RB['cam1'] = {at:Date.now(), s:six};
  FMT_RB['cam2'] = {at:Date.now(), s:five};
  renderFmtReadback();
  var ph = document.querySelector('#fmtReadback').innerHTML;
  THAS('U14 a withheld field is marked, not dropped', ph, 'not reported');
  THAS('U14 the withheld field is still named', ph,
       'file <span class="neq">not reported</span>');
  THAS('U14 the fields the body did answer still read normally', ph, 'size <b>S</b>');
  TNOT('U14 a partial answer is not mistaken for an ilxctl too old to report',
       ph, 'no readback');
  FMT_RB['cam2'] = {at:Date.now(), s:six};
  renderFmtReadback();
  var ph2 = document.querySelector('#fmtReadback').innerHTML;
  TNOT('U14 a body that reported all six carries no unconfirmed field',
       ph2, 'not reported');
  THAS('U14 ... and still shows every one of them', ph2, 'pcsave <b>RAW+JPEG</b>');
}catch(e){ T('U14 partial format readback', false, 'threw ' + e); }

// ---- U15 Apply to fleet is honest about what it could not read -----------
// Only a COMPLETELY empty body was refused: a body built from one surviving
// field was posted and reported with the same green "fleet now on cam2's
// exposure" line as a full apply, while rigd wrote only that field and left
// the pair split on the rest.
try{
  FLEET = {nodes:[node({name:'cam1'}), node({name:'cam2'})], run:{}};
  var src = node({name:'cam2'});
  delete EV_LEDGER['cam2'];
  var isoSel = document.querySelector('#iso-cam2');
  isoSel.classList.remove('edited');

  var full = exposureSource(src, {ok:true, apertureValue:1100,
    shutterValue:SH(1,250), isoValue:800, expcompValue:0});
  TEQ('U15 a body that answers fully leaves nothing unsourced', full.missing.length, 0);
  TEQ('U15 ... and the whole exposure is what gets sent',
      JSON.stringify(Object.keys(full.body).sort()),
      '["aperture","expcomp","iso","shutter"]');

  // ilxctl held on its SDK mutex answers a degraded body: no exposure keys and
  // no 'ok' key, so `live.ok!==false` waved it through as a good readback.
  var busy = exposureSource(src, {connected:true, busy:true, model:'', id:'', log:[]});
  TEQ('U15 the degraded busy body is not a readback', busy.live_ok, false);
  TEQ('U15 a busy body leaves the whole exposure unsourced', busy.missing.length, 3);

  // The live read dropped; only the edit the operator can see survives.
  isoSel.value = '3200'; isoSel.classList.add('edited');
  var part = exposureSource(src, {ok:false, error:'aborted', _err:true});
  TEQ('U15 the fields that could not be read are named',
      JSON.stringify(part.missing), '["aperture","shutter"]');
  TEQ('U15 the edit on screen is still sent', part.body.iso, 3200);
  TEQ('U15 an unread field is never guessed', part.body.aperture, undefined);

  var v = applyVerdict('cam2', ['iso'], part.missing);
  THAS('U15 a partial apply is reported amber', v, 'var(--warn)');
  TNOT('U15 ... and never wears the success colour', v, 'var(--ok)');
  THAS('U15 the partial verdict names the unread fields', v, 'aperture, shutter');
  THAS('U15 the partial verdict says they were NOT applied', v, 'NOT applied');
  THAS('U15 the partial verdict says the pair may still be split',
       v, 'may still be split');
  var vfull = applyVerdict('cam2', ['aperture','shutter','iso','expcomp'], []);
  THAS('U15 a complete apply still reads as success', vfull, 'var(--ok)');
  THAS('U15 ... and still names what landed', vfull, 'aperture, shutter, iso, expcomp');
  TNOT('U15 a hostile node name cannot inject markup',
       applyVerdict('<img src=x>', ['iso'], ['aperture']), '<img src=x');
  isoSel.classList.remove('edited');

  // The refusal used to test the whole body, and the success path writes
  // EV_LEDGER for every connected node — so from the first successful apply
  // onwards the body was never empty and a camera that answered NOTHING still
  // posted a forced fleet reconcile built from a stale ledger value.
  EV_LEDGER['cam2'] = 300;
  var none = exposureSource(src, {ok:false, error:'aborted', _err:true});
  TEQ('U15 a camera that answered nothing leaves all three unsourced',
      none.missing.length, 3);
  TEQ('U15 ... even though the ledger still puts an expcomp in the body',
      none.body.expcomp, 300);
  delete EV_LEDGER['cam2'];
}catch(e){ T('U15 exposureSource/applyVerdict', false, 'threw ' + e); }

// ---- U7 pre-flight snapshot identity -------------------------------------
try{
  document.querySelector('#cLabel').value = 'transect-01';
  document.querySelector('#cInt').value = '2';
  document.querySelector('#cFrames').value = '0';
  document.querySelector('#cAuto').checked = true;
  var sig1 = startInputsSig();
  document.querySelector('#cInt').value = '0.3';
  T('U7 changing the interval changes the pre-flight signature',
    startInputsSig() !== sig1, sig1 + ' -> ' + startInputsSig());
  document.querySelector('#cInt').value = '2';
  TEQ('U7 restoring the inputs restores the signature', startInputsSig(), sig1);
  T('U7 the pre-flight has a bounded lifetime', typeof PF_TTL_MS === 'number' && PF_TTL_MS > 0,
    'PF_TTL_MS=' + PF_TTL_MS);
}catch(e){ T('U7 startInputsSig', false, 'threw ' + e); }

// ---- U12 card drain: reachable, honest, cancellable ----------------------
try{
  FLEET = {nodes:[node({name:'cam1'}), node({name:'cam2'})], run:{}};
  DRAIN_LAST = {}; DRAIN_ERR = ''; DRAIN_BUSY = false;
  var idle = function(o){
    var d = {active:false, node:null, queue:[], skipped:{}, wedged:{},
             cancel_requested:false, last:null};
    for(var k in (o||{})) d[k] = o[k];
    return d;
  };

  // --- idle ---------------------------------------------------------------
  TEQ('U12 an idle status is accepted', applyDrain(idle()), true);
  var s0 = document.querySelector('#drainState').innerHTML;
  THAS('U12 an idle drain says so', s0, 'idle');
  THAS('U12 idle explains where drains come from', s0, 'automatically after Stop');
  THAS('U12 idle repeats the safety rule', s0, 'verified its bytes');
  TEQ('U12 no cancel control is offered while idle',
      document.querySelector('#btnDrainCancel').style.display, 'none');
  TEQ('U12 a drain can be started by hand while idle',
      document.querySelector('#btnDrain').disabled, false);
  THAS('U12 with no history the panel says so rather than inventing one',
       document.querySelector('#drainLast').innerHTML, 'no drain has finished');

  var sel = document.querySelector('#drNode');
  TEQ('U12 the drain scope offers all-connected plus every camera', sel.options.length, 3);
  TEQ('U12 the default scope sends no node list', sel.options[0].value, '');
  TEQ('U12 each camera can be picked alone', sel.options[1].value, 'cam1');

  // --- active -------------------------------------------------------------
  applyDrain(idle({active:true, node:'cam1', queue:['cam1','cam2']}));
  var s1 = document.querySelector('#drainState').innerHTML;
  THAS('U12 the node being drained is named', s1, 'draining <b>cam1</b>');
  THAS('U12 the queue behind it is shown', s1, 'cam2');
  THAS('U12 the panel says the next transect is blocked', s1, 'next transect cannot start');
  THAS('U12 it is honest that rigd reports no per-shot progress', s1, 'no per-shot progress');
  THAS('U12 it points at the file-by-file log', s1, 'Events');
  TEQ('U12 Cancel appears while a drain runs',
      document.querySelector('#btnDrainCancel').style.display, '');
  TEQ('U12 Cancel is pressable while a drain runs',
      document.querySelector('#btnDrainCancel').disabled, false);
  TEQ('U12 a second drain cannot be stacked on a running one',
      document.querySelector('#btnDrain').disabled, true);

  // --- a dropped poll is not "idle" ---------------------------------------
  TEQ('U12 a dropped poll is not accepted as a status',
      applyDrain({ok:false, error:'aborted', _err:true}), false);
  var se = document.querySelector('#drainState').innerHTML;
  THAS('U12 a dropped poll says the panel is not live', se, 'not live');
  THAS('U12 a dropped poll names the failure', se, 'aborted');
  THAS('U12 a dropped poll keeps the running drain on screen, never "idle"',
       se, 'draining <b>cam1</b>');
  TEQ('U12 a reply with no active flag is not a status either', applyDrain({}), false);
  TEQ('U12 no reply at all is not a status either', applyDrain(null), false);

  // --- cancel requested ---------------------------------------------------
  applyDrain(idle({active:true, node:'cam1', queue:['cam1'], cancel_requested:true}));
  var s2 = document.querySelector('#drainState').innerHTML;
  THAS('U12 a requested cancel carries its honest caveat', s2, 'finishes the shot it is on');
  THAS('U12 a requested cancel says nothing verified is lost', s2, 'already verified stays pulled');
  THAS('U12 a requested cancel says the rest of the card is untouched', s2, 'untouched');
  TEQ('U12 the cancel button cannot be pressed a second time',
      document.querySelector('#btnDrainCancel').disabled, true);
  TEQ('U12 the cancel button relabels itself',
      document.querySelector('#btnDrainCancel').textContent, 'cancelling…');

  // --- per-node results ---------------------------------------------------
  applyDrain(idle({last:{node:'cam1', at:1755950000, pulled:41, bytes:1.42e9,
                         deleted:41, verified:41, errors:0, cancelled:false}}));
  var l1 = document.querySelector('#drainLast').innerHTML;
  THAS('U12 the last result names its camera', l1, 'cam1');
  THAS('U12 shots pulled are shown', l1, '41');
  THAS('U12 bytes are shown in GB', l1, '1.4 GB');
  TNOT('U12 a fully deleted card is not accused of leaving shots behind',
       l1, 'still on the card');
  // rigd keeps ONE `last`; cam2 finishing must not erase cam1's numbers.
  applyDrain(idle({last:{node:'cam2', at:1755950600, pulled:12, bytes:4.2e8,
                         deleted:9, verified:12, errors:2, cancelled:true}}));
  var l2 = document.querySelector('#drainLast').innerHTML;
  THAS("U12 cam1's result survives cam2 finishing", l2, 'cam1');
  THAS('U12 cam2 is listed too', l2, 'cam2');
  THAS('U12 bytes under a GB are shown in MB', l2, '420 MB');
  THAS('U12 a cancelled drain is labelled cancelled', l2, 'CANCELLED');
  THAS('U12 errors are counted and pointed at the journal', l2, '2 — see Events');
  THAS('U12 shots left on the card are stated, not implied', l2, '3 still on the card');
  applyDrain(idle({last:{node:'cam1', at:'not-a-time', pulled:1, bytes:null,
                         deleted:1, errors:0}}));
  THAS('U12 a malformed finish time renders as a dash, not a crash',
       document.querySelector('#drainLast').innerHTML, '—');
  applyDrain(idle({last:{node:'cam1', at:1755950000, pulled:'<b>9</b>',
                         bytes:'lots', deleted:null, errors:'many'}}));
  var l4 = document.querySelector('#drainLast').innerHTML;
  TNOT('U12 a count that is not a number cannot inject markup', l4, '<b>9</b>');
  THAS('U12 a count that is not a number reads as unknown, not as zero', l4, '—');

  // --- skipped nodes ------------------------------------------------------
  applyDrain(idle({skipped:{cam2:'not connected - its card was NOT drained'}}));
  var l3 = document.querySelector('#drainLast').innerHTML;
  THAS('U12 a skipped node is named', l3, 'cam2');
  THAS('U12 the skip reason is shown verbatim', l3, 'card was NOT drained');
  THAS('U12 a skip is attributed to the last drain START, not to now',
       l3, 'last drain start skipped');

  // --- the wedge ----------------------------------------------------------
  applyDrain(idle({wedged:{cam2:{at:1755950000, at_iso:'2026-08-23T11:02:03Z',
                                 rebooted_at:null}}}));
  var w = document.querySelector('#drainWedge').innerHTML;
  THAS('U12 the wedged node is named', w, 'cam2');
  THAS('U12 the wedge is called what it is', w, 'wedged');
  THAS('U12 the wedge carries its real remedy', w, 'power-cycle that camera body');
  THAS('U12 the wedge says a drain can then be run from here', w,
       'drain it again from here');
  THAS('U12 the wedge promises the card keeps every frame', w, 'keeps every frame');
  THAS('U12 the wedge says the automatic drain will skip that node', w, 'SKIP');
  THAS('U12 the wedge is stamped with when it happened', w, '2026-08-23T11:02:03Z');
  applyDrain(idle());
  TEQ('U12 a cleared wedge leaves nothing behind',
      document.querySelector('#drainWedge').innerHTML, '');

  // --- a run is recording -------------------------------------------------
  FLEET = {nodes:[node({name:'cam1'})], run:{active:true, run_id:'r'}};
  applyDrain(idle());
  TEQ('U12 a drain cannot be started while a run is recording',
      document.querySelector('#btnDrain').disabled, true);
  THAS('U12 ... and the button says why',
       document.querySelector('#btnDrain').title, 'cannot shoot and drain');
  FLEET = {nodes:[node({name:'cam1'}), node({name:'cam2'})], run:{}};

  // --- everything from the server is escaped ------------------------------
  applyDrain({active:true, node:'<img src=x onerror=1>', queue:['<b>q</b>'],
              skipped:{'<i>n</i>':'<script>alert(1)</script>'},
              wedged:{'<u>w</u>':{at_iso:'<em>t</em>'}},
              cancel_requested:false, last:null});
  var all = document.querySelector('#drainState').innerHTML +
            document.querySelector('#drainLast').innerHTML +
            document.querySelector('#drainWedge').innerHTML;
  TNOT('U12 a node name cannot inject markup', all, '<img src=x');
  TNOT('U12 a queue entry cannot inject markup', all, '<b>q</b>');
  TNOT('U12 a skip reason cannot inject a script tag', all, '<script>');
  TNOT('U12 a wedge key cannot inject markup', all, '<u>w</u>');
  THAS('U12 hostile text is escaped, not dropped', all, '&lt;img src=x');
  DRAIN_LAST = {}; applyDrain(idle());
}catch(e){ T('U12 drain panel', false, 'threw ' + e); }

// ---- U12 the panel really calls rigd's drain API -------------------------
try{
  var __REQ = [], __oldFetch = fetch;
  fetch = function(p, opt){ __REQ.push({p:p, opt:opt||{}});
    var o = {}; o.then = function(){ return o; }; o.catch = function(){ return o; };
    o.finally = function(){ return o; }; return o; };

  DRAIN_BUSY = false;
  document.querySelector('#drNode').value = '';
  document.querySelector('#drKeep').checked = false;
  startDrain();
  T('U12 Drain posts to /api/drain',
    __REQ.length === 1 && __REQ[0].p === '/api/drain', JSON.stringify(__REQ));
  TEQ('U12 the drain start is a POST', (__REQ[0].opt||{}).method, 'POST');
  var b0 = JSON.parse((__REQ[0].opt||{}).body || '{}');
  TEQ('U12 "all connected" sends no node list - rigd picks the set', b0.nodes, undefined);
  TEQ('U12 the default drain empties every card it verifies', b0.keep, false);
  THAS('U12 the operator is told the start is in flight',
       document.querySelector('#drainNote').innerHTML, 'starting');
  TEQ('U12 the Drain button greys out the moment a start is sent, not a poll later',
      document.querySelector('#btnDrain').disabled, true);

  __REQ = []; DRAIN_BUSY = false;
  document.querySelector('#drNode').value = 'cam2';
  document.querySelector('#drKeep').checked = true;
  startDrain();
  var b1 = JSON.parse((__REQ[0].opt||{}).body || '{}');
  TEQ('U12 one camera can be drained on its own', JSON.stringify(b1.nodes), '["cam2"]');
  TEQ('U12 keep-cards is passed through to rigd', b1.keep, true);

  __REQ = []; DRAIN_BUSY = true;
  startDrain();
  TEQ('U12 a start while one is already in flight sends nothing', __REQ.length, 0);
  DRAIN_BUSY = false;

  __REQ = [];
  document.querySelector('#btnDrainCancel').disabled = false;
  cancelDrain();
  T('U12 Cancel posts to /api/drain/cancel',
    __REQ.length === 1 && __REQ[0].p === '/api/drain/cancel', JSON.stringify(__REQ));
  TEQ('U12 cancel disables itself the moment it is sent',
      document.querySelector('#btnDrainCancel').disabled, true);
  THAS('U12 the cancel note repeats the caveat',
       document.querySelector('#drainNote').innerHTML, 'finishes the shot it is on');
  __REQ = [];
  cancelDrain();
  TEQ('U12 a disabled Cancel cannot be re-sent', __REQ.length, 0);

  fetch = __oldFetch;
}catch(e){ T('U12 drain API calls', false, 'threw ' + e); }

return JSON.stringify(__R);
"""


# ---------------------------------------------------------------------------
# Loading the real page
# ---------------------------------------------------------------------------
def _split(html):
    """(markup-before-the-script, script-body). The page has exactly one."""
    m = re.search(r"<script>\n(.*)\n</script>", html, re.S)
    if not m:
        return html, ""
    return html[:m.start()], m.group(1)


def _idtags(markup):
    """id -> tagName, read out of the real markup so the shim hands the script
    a SELECT where the page has a SELECT. Ids the page creates at runtime
    (rvfr-cam1, ap-cam1, ...) are covered by the shim's prefix table."""
    out = {}
    for m in re.finditer(r'<([a-zA-Z][\w-]*)\b[^>]*\bid="([^"]+)"', markup):
        out.setdefault(m.group(2), m.group(1).upper())
    return out


def _js_available():
    if platform.system() != "Darwin":
        return False, "not macOS - JavaScriptCore via osascript is unavailable"
    if not shutil.which("osascript"):
        return False, "osascript not on PATH"
    return True, ""


def _run_js(html):
    """(results, error). results is the driver's list of {name, ok, detail}."""
    markup, script = _split(html)
    if not script:
        return None, "rig_ui.html has no <script> block"
    shim = SHIM.replace("__IDTAG__", json.dumps(_idtags(markup)))
    # One function scope around everything: JavaScriptCore already owns a
    # global `$` (the ObjC bridge) and the page declares `const $` at top
    # level, which is a redeclaration error at global scope but fine inside a
    # function.
    src = "var __OUT=(function(){\n%s\n%s\n%s\n})();\n__OUT;\n" % (
        shim, script, DRIVER)
    tmp = tempfile.mkdtemp(prefix="audit_ui_")
    try:
        path = os.path.join(tmp, "run.js")
        with open(path, "w", encoding="utf-8") as fh:
            fh.write(src)
        try:
            p = subprocess.run(["osascript", "-l", "JavaScript", path],
                               capture_output=True, text=True,
                               timeout=JS_TIMEOUT_S)
        except subprocess.TimeoutExpired:
            return None, "osascript timed out after %ds" % JS_TIMEOUT_S
        if p.returncode != 0:
            return None, (p.stderr or "").strip()[:400]
        try:
            return json.loads(p.stdout.strip()), ""
        except ValueError:
            return None, "driver produced no JSON: %r" % p.stdout.strip()[:400]
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


# ---------------------------------------------------------------------------
# The executable half
# ---------------------------------------------------------------------------
def _behaviour(opts):
    sect("ui: rig_ui.html behaviour (loaded and called)")
    ok, why = _js_available()
    if not ok:
        note("skipped the executable half: %s. The structural checks below "
             "still run, but they are far weaker - re-run this suite on the "
             "rigd host (macOS) before trusting a green ui lane." % why)
        return
    html = open(UI_PATH, encoding="utf-8").read()
    results, err = _run_js(html)
    if results is None:
        # A page that will not even parse is the loudest possible UI defect:
        # the operator gets a blank screen with no rig control at all.
        check("ui rig_ui.html loads and runs", False, err)
        return
    check("ui rig_ui.html loads and runs", True,
          "%d assertions driven against the real script" % len(results))
    for r in results:
        check(r["name"], r["ok"], r.get("detail", ""))


# ---------------------------------------------------------------------------
# The structural half - contracts that cannot be called, only read
# ---------------------------------------------------------------------------
def _structure(opts):
    sect("ui: rig_ui.html structure")
    html = open(UI_PATH, encoding="utf-8").read()
    markup, script = _split(html)

    # U7: RunManager.stop legitimately runs 12-20 s; api()'s default abort is
    # 8 s, so a healthy stop was reported as failed and the pulled/FAILED
    # tally never appeared.
    check("U7 stop is given a timeout longer than a real stop takes",
          re.search(r"api\('/api/run/stop',\s*\{\},\s*(\d+)\)", script)
          and int(re.search(r"api\('/api/run/stop',\s*\{\},\s*(\d+)\)",
                            script).group(1)) >= 25000,
          "want >=25000 ms")
    check("U7 the operator is told a stop is in progress",
          "stopping" in script, "a 20 s silent button reads as a dead button")
    check("U7 a lost stop reply is reconciled against the run flag, not "
          "declared a failure", "stop sent" in script)
    check("U7 the pre-flight snapshot expires",
          "PF_TTL_MS" in script and "startInputsSig" in script)

    # U9: a long transect label wrapped the header onto two lines at 1440px.
    check("U9 the run pill truncates instead of wrapping the header",
          re.search(r"#hRun\{[^}]*text-overflow:ellipsis", markup, re.S))
    check("U9 the full run id survives in the pill's title",
          "$('#hRun').title" in script)
    check("U9 the clock is kept on one line",
          re.search(r"header \.clock\{[^}]*white-space:nowrap", markup, re.S))
    check("U9 Start transect is disabled while a run is recording",
          "sb.disabled=!!run.active" in script)
    check("U9 the disabled Start button says why",
          "is recording" in script)

    # U8: the 5 s hydrate overwrote a half-typed Kelvin value under the caret.
    m = re.search(r"function syncFmtControls\(\)\{.*?\n\}", script, re.S)
    check("U8 the format hydrate never writes under the operator's caret",
          m and "document.activeElement===el" in m.group(0))
    check("U8 a text input goes dirty on the first keystroke, not on blur",
          "el.oninput=dirty" in script)

    # U10: the format fields were blind - the select showed what rigd pushes,
    # never what the body reports (contract C2).
    check("U10 the format card offers a RAW type",
          'id="sRawtype"' in markup)
    check("U10 LossLessL is the RAW default",
          re.search(r'id="sRawtype".*?<option value="5">LossLessL',
                    markup, re.S))
    for key in ("filetypeLabel", "imagesizeLabel", "transsizeLabel",
                "rawtypeLabel", "qualityLabel", "pcsaveLabel"):
        check("U10 the readback panel reads %s" % key, key in script)
    check("U10 the readback is read live per body, not from the fleet cache",
          "/api/status?node=" in script and "pollFmtReadback" in script)
    check("U10 the node/host clock offset reaches the health strip",
          "clock_offset_ms" in script)

    # U3: Apply built its body from the <=4 s-old fleet poll cache.
    m = re.search(r"\$\('#btnApply'\)\.onclick.*?\n\};", script, re.S)
    check("U3 Apply reads the source body live", m and "/api/status?node=" in m.group(0))
    check("U3 Apply no longer builds its body from the fleet poll cache",
          m and "src.shutter_raw" not in m.group(0) and
          "src.aperture_raw" not in m.group(0))

    # U13: the strip's clock chip is a decision surface for the operator, so it
    # reads what every other consumer reads - the filtered figure.
    m = re.search(r"function healthStrip\(n\)\{.*?\n\}", script, re.S)
    check("U13 the health strip reads the filtered clock offset",
          m and "n.clock_offset_s" in m.group(0)
          and "clock_offset_info" in m.group(0))
    check("U13 the raw sample survives where rigd publishes it for - the title",
          m and "last raw sample" in m.group(0))
    check("U13 the strip and rigd agree on what link is too slow to measure",
          "CLK_RTT_LIMIT_MS" in script and "unmeasurable" in script)

    # U15: the fleet exposure apply. The refusal has to be about the exposure
    # triple (aperture/shutter/iso) - the body always carries an expcomp once
    # EV_LEDGER is populated, which made the whole-body test unreachable.
    m = re.search(r"\$\('#btnApply'\)\.onclick.*?\n\};", script, re.S)
    check("U15 Apply refuses on the exposure triple, not on the whole body",
          m and "plan.missing.length===EXP_TRIPLE.length" in m.group(0)
          and "if(!Object.keys(body).length)" not in script)
    check("U15 the apply verdict is built from what could NOT be read, not "
          "only from what rigd applied",
          "function applyVerdict(" in script and "plan.missing" in script)
    check("U15 ilxctl's degraded busy body is not accepted as a readback",
          "live.busy!==true" in script)
    m = re.search(r"async function hydrateSettings\(\)\{.*?\n\}", script, re.S)
    check("U15 the apply verdict is not erased by the hydrate that follows it",
          m and "APPLY_MSG_AT" in m.group(0) and "APPLY_MSG_TTL_MS" in m.group(0))

    # U1: the old pick took /api/runs' first entry whatever its frame count.
    check("U1 Review no longer takes the newest run sight-unseen",
          "/api/runs?limit=1'" not in script)
    check("U1 the spool is reached only when no run can be shown",
          "pickRunWithFrames" in script)

    # U5: live view and the recorded pair shared one <img> and one latch.
    check("U5 the live/recorded transition is owned by one helper",
          "function frameImg(" in script)
    check("U5 the old unguarded badge insertion is gone",
          '<img alt=""><div class="burn">LIVE</div>' not in script)

    # U12: the card drain had no UI at all. Placement is part of the contract:
    # the drain is what stands between this transect and the next (rigd refuses
    # a run while it holds a camera), so it belongs on Review beside Start/Stop
    # - not on Controls, which is per-camera setup done BEFORE a line.
    i_rev = markup.find('id="tab-review"')
    i_tr = markup.find('id="tab-transects"')
    i_dr = markup.find('id="drainCard"')
    check("U12 the drain panel exists at all", i_dr > -1,
          "no #drainCard in the markup")
    check("U12 the drain panel is on Review, with Start/Stop",
          -1 < i_rev < i_dr < i_tr,
          "drain@%d review@%d transects@%d" % (i_dr, i_rev, i_tr))
    i_run = markup.find('id="runInfo"')
    check("U12 it sits under the run-control card that starts it",
          -1 < i_run < i_dr, "runInfo@%d drain@%d" % (i_run, i_dr))
    for el in ('id="drainState"', 'id="drainLast"', 'id="drainWedge"',
               'id="btnDrain"', 'id="btnDrainCancel"', 'id="drNode"',
               'id="drKeep"'):
        check("U12 the panel has %s" % el, el in markup)

    # Starting a drain is the irreversible half - each verified shot is DELETED
    # from the card - so it wears the Stop button's hold gesture. Cancel is the
    # reversible half (the server never stops between a verify and a delete)
    # and stays one click, made unrepeatable by disabling itself.
    check("U12 starting a drain is hold-to-confirm, like Stop",
          re.search(r'id="btnDrain"[^>]*class="act hold"', markup)
          or re.search(r'class="act hold"[^>]*id="btnDrain"', markup))
    check("U12 the hold has a fill to animate over",
          re.search(r'id="btnDrain"><span class="fill">', markup))
    check("U12 the hold is wired to the drain start",
          re.search(r"holdConfirm\(\$\('#btnDrain'\),\s*(\d+)", script)
          and int(re.search(r"holdConfirm\(\$\('#btnDrain'\),\s*(\d+)",
                            script).group(1)) >= 1000,
          "want a hold of at least 1000 ms")
    check("U12 cancel is a plain click, not a second hold gesture",
          "$('#btnDrainCancel').onclick" in script
          and "holdConfirm($('#btnDrainCancel')" not in script)
    check("U12 the reason cancel needs no hold is written down",
          "cannot cost a frame" in script)

    # The whole point: these endpoints were unreachable from the browser.
    check("U12 the panel reads rigd's drain status", "'/api/drain'" in script)
    check("U12 the panel can cancel a drain",
          "'/api/drain/cancel'" in script)
    check("U12 a drain can be started by hand",
          "function startDrain(" in script)

    # Polling: Review's own cadence and Review's own gating, never faster.
    m = re.search(r"if\(liveOk\('review'\)\) pollDrain\(\); \}, (\d+)\)", script)
    check("U12 the drain poll is gated on the visible tab, like the others",
          m is not None, "no liveOk('review')-gated pollDrain interval")
    check("U12 the drain poll is no faster than Review's own cadence",
          m and int(m.group(1)) >= 3000,
          "cadence %s ms" % (m.group(1) if m else "?"))
    # Anything that is not the definition, not an explicit forced one-shot
    # (tab switch, tab un-hidden, after a Stop, page load) and not behind the
    # review gate would poll rigd while the panel is not even on screen.
    ungated = []
    for m in re.finditer(r"pollDrain\(([^)]*)\)", script):
        before = script[max(0, m.start() - 40):m.start()]
        if m.group(1) in ("force", "true") or "liveOk('review')" in before:
            continue
        ungated.append(before[-30:] + m.group(0))
    check("U12 the drain is polled only behind the review-tab gate",
          not ungated, "ungated: %r" % (ungated[:2],))

    # Honesty: every string rigd hands the panel is escaped, and a failed read
    # is never drawn as "idle".
    m = re.search(r"function renderDrain\(\)\{.*?\n\}\n", script, re.S)
    body = m.group(0) if m else ""
    check("U12 renderDrain escapes the server's strings", body.count("escapeHtml(") >= 8,
          "%d escapeHtml calls in renderDrain" % body.count("escapeHtml("))
    check("U12 no server string is interpolated raw",
          body and "'+node+'" not in body and "'+(DRAIN.node" not in body)
    check("U12 counts rigd sends are type-checked before they reach the page",
          "function drainNum(" in script and "L.pulled==null" not in script)
    check("U12 a failed status read is not rendered as idle",
          "function applyDrain(" in script and "typeof d.active!=='boolean'" in script)
    check("U12 the wedge remedy is the real one",
          "power-cycle that camera body" in script)
    check("U12 Start says a drain is blocking it, instead of bouncing off rigd",
          "a card drain is running on" in script)


def suite(opts):
    _behaviour(opts)
    _structure(opts)


if __name__ == "__main__":
    import argparse
    import soaktest
    ap = argparse.ArgumentParser()
    ap.add_argument("--verbose", action="store_true")
    suite(ap.parse_args())
    print("\naudit_ui: %d passed, %d failed"
          % (len(soaktest.PASS), len(soaktest.FAIL)))
    sys.exit(1 if soaktest.FAIL else 0)
