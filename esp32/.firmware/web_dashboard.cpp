/*
 * web_dashboard.cpp — HTTP + WebSocket dashboard for Tesla FSD ESP32
 *
 * HTTP  :80  → serves the embedded HTML page
 * WS    :81  → pushes JSON state every 1 s; receives control commands
 *
 * All HTML/CSS/JS is embedded as a raw-string literal — no external CDN.
 * State is read directly from the FSDState pointer; controls write back to it.
 *
 * Single-threaded: both handleClient() and ws.loop() are called from loop()
 * after the CAN drain, so there is no concurrency issue.
 */

#include "web_dashboard.h"
#include "can_dump.h"
#include "prefs.h"
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <WiFi.h>
#include <Arduino.h>

// ── Module state ──────────────────────────────────────────────────────────────
static FSDState  *g_state = nullptr;   // shared with main
static CanDriver *g_can   = nullptr;   // for setListenOnly()

static WebServer        g_http(80);
static WebSocketsServer g_ws(81);

static uint32_t g_start_ms    = 0;
static uint32_t g_last_rx     = 0;
static uint32_t g_last_fps_ms = 0;
static uint32_t g_last_can_seen_ms = 0;
static float    g_fps         = 0.0f;

#define CAN_VEHICLE_ALIVE_MS 3000u

// ── Embedded HTML/CSS/JS ──────────────────────────────────────────────────────
// Tesla dark theme; mobile-first (max 480 px); WebSocket on :81
static const char WEB_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<meta name="apple-mobile-web-app-capable" content="yes">
<meta name="apple-mobile-web-app-status-bar-style" content="black-translucent">
<meta name="theme-color" content="#0a0a1a">
<link rel="icon" href="data:,">
<title>Tesla FSD</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
:root{
  --bg:#0a0a1a;--card:#111827;--card2:#1a1f35;
  --accent:#00d4aa;--accent2:#00b894;
  --red:#ff6b6b;--yellow:#ffd93d;--blue:#4dabf7;
  --border:#1e293b;--text:#e2e8f0;--text2:#94a3b8;--text3:#475569
}
body{font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',Roboto,sans-serif;
  background:var(--bg);color:var(--text);min-height:100vh}
.wrap{max-width:480px;margin:0 auto;padding:16px 16px 40px}

/* ── Header ── */
.hdr{text-align:center;padding:20px 0 12px;position:relative}
.hdr h1{font-size:1.65em;font-weight:700;
  background:linear-gradient(135deg,var(--accent),var(--blue));
  -webkit-background-clip:text;-webkit-text-fill-color:transparent;
  letter-spacing:-.02em}
.hdr .sub{font-size:.68em;color:var(--text3);margin-top:3px;
  letter-spacing:.1em;text-transform:uppercase}
.cdot{position:absolute;right:0;top:26px;width:10px;height:10px;
  border-radius:50%;background:var(--accent);
  box-shadow:0 0 10px var(--accent);transition:.4s}
.cdot.off{background:var(--red);box-shadow:0 0 10px var(--red)}

/* ── OTA Warning ── */
.ota{display:none;background:rgba(255,107,107,.1);border:1px solid rgba(255,107,107,.4);
  border-radius:12px;padding:12px 16px;margin-bottom:12px;text-align:center;
  color:var(--red);font-weight:700;font-size:.9em;letter-spacing:.04em;
  animation:pulse 1s ease-in-out infinite}
@keyframes pulse{0%,100%{opacity:1}50%{opacity:.55}}

/* ── Error banner ── */
.err{display:none;color:var(--red);text-align:center;font-size:.78em;padding:8px;
  background:rgba(255,107,107,.07);border-radius:10px;margin-bottom:10px;
  border:1px solid rgba(255,107,107,.18)}

/* ── Cards ── */
.card{background:var(--card);border-radius:16px;padding:16px;
  margin-bottom:12px;border:1px solid var(--border)}
.card-head{display:flex;align-items:center;gap:8px;margin-bottom:12px}
.icon{width:28px;height:28px;border-radius:8px;display:flex;
  align-items:center;justify-content:center;font-size:.85em;font-weight:700}
.ic-s{background:rgba(0,212,170,.14);color:var(--accent)}
.ic-b{background:rgba(77,171,247,.14);color:var(--blue)}
.ic-c{background:rgba(255,217,61,.14);color:var(--yellow)}
.ic-d{background:rgba(148,163,184,.14);color:var(--text2)}
.card-head h2{font-size:.78em;font-weight:600;color:var(--text2);
  text-transform:uppercase;letter-spacing:.07em}

/* ── Rows ── */
.row{display:flex;justify-content:space-between;align-items:center;padding:9px 0}
.row+.row{border-top:1px solid rgba(255,255,255,.04)}
.lbl{color:var(--text2);font-size:.85em}

/* ── Pills ── */
.pill{display:inline-flex;align-items:center;gap:5px;
  padding:3px 10px;border-radius:20px;font-size:.8em;font-weight:600}
.pill.on{background:rgba(0,212,170,.14);color:var(--accent)}
.pill.off{background:rgba(71,85,105,.22);color:var(--text3)}
.pill.warn{background:rgba(255,107,107,.14);color:var(--red)}
.pd{width:6px;height:6px;border-radius:50%;flex-shrink:0;
  background:currentColor;box-shadow:0 0 5px currentColor}

/* ── Battery Hero ── */
.hero{text-align:center;padding-bottom:4px}
.soc-ring{width:120px;height:120px;margin:0 auto 14px;position:relative}
.soc-ring svg{transform:rotate(-90deg)}
.trk{fill:none;stroke:#1e293b;stroke-width:8}
.bar{fill:none;stroke:var(--accent);stroke-width:8;stroke-linecap:round;
  transition:stroke-dashoffset .8s ease,stroke .5s}
.soc-val{position:absolute;inset:0;display:flex;flex-direction:column;
  align-items:center;justify-content:center}
.soc-num{font-size:2em;font-weight:700;line-height:1;
  font-variant-numeric:tabular-nums}
.soc-lbl{font-size:.6em;color:var(--text3);margin-top:3px;text-transform:uppercase}
.hg{display:grid;grid-template-columns:repeat(3,1fr);gap:8px;text-align:center}
.hg .hv{font-size:1.1em;font-weight:600;font-variant-numeric:tabular-nums}
.hg .hl{font-size:.65em;color:var(--text3);margin-top:2px}

/* ── CAN stat grid ── */
.sg{display:grid;grid-template-columns:repeat(2,1fr);gap:8px}
.sb{background:var(--card2);border-radius:10px;padding:10px 12px}
.sb .sv{font-size:1.15em;font-weight:700;font-variant-numeric:tabular-nums}
.sb .sl{font-size:.64em;color:var(--text3);margin-top:2px}

/* ── Controls ── */
.btn-main{width:100%;padding:14px;border:none;border-radius:12px;
  font-size:.95em;font-weight:700;cursor:pointer;letter-spacing:.04em;
  transition:opacity .2s;margin-bottom:10px}
.btn-main:active{opacity:.75}
.btn-act{background:linear-gradient(135deg,var(--accent),var(--accent2));color:#000}
.btn-stop{background:rgba(255,107,107,.14);color:var(--red);
  border:1px solid rgba(255,107,107,.3)}
.sw{position:relative;width:44px;height:24px;flex-shrink:0}
.sw input{opacity:0;width:0;height:0}
.sl2{position:absolute;cursor:pointer;inset:0;background:#2a2a3e;
  border-radius:24px;transition:.3s}
.sl2:before{content:"";position:absolute;height:18px;width:18px;
  left:3px;bottom:3px;background:#555;border-radius:50%;transition:.3s}
input:checked+.sl2{background:var(--accent)}
input:checked+.sl2:before{transform:translateX(20px);background:#fff}

/* ── Footer ── */
.foot{text-align:center;padding:16px 0 0;font-size:.64em;color:var(--text3)}
</style>
</head>
<body>
<div class="wrap">

<!-- Header -->
<div class="hdr">
  <h1>Tesla FSD</h1>
  <div class="sub">ESP32 CAN Controller &middot; 192.168.4.1</div>
  <div class="cdot" id="dot"></div>
</div>
<div id="connErr" class="err">Connection lost &mdash; retrying&hellip;</div>

<!-- OTA Warning -->
<div id="otaBanner" class="ota">&#9888;&#xFE0F; OTA UPDATE IN PROGRESS &mdash; CAN TX SUSPENDED</div>

<!-- FSD Status -->
<div class="card">
  <div class="card-head"><div class="icon ic-s">S</div><h2>FSD Status</h2></div>
  <div class="row">
    <span class="lbl">FSD Active</span>
    <span class="pill off" id="fsdSt"><span class="pd"></span>--</span>
  </div>
  <div class="row">
    <span class="lbl">Mode</span>
    <span class="pill off" id="opMode"><span class="pd"></span>--</span>
  </div>
  <div class="row">
    <span class="lbl">Hardware</span>
    <span class="pill off" id="hwVer"><span class="pd"></span>--</span>
  </div>
  <div class="row">
    <span class="lbl">NAG Killer</span>
    <span class="pill off" id="nagSt"><span class="pd"></span>--</span>
  </div>
  <div class="row">
    <span class="lbl">CAN Vehicle</span>
    <span class="pill off" id="canVeh"><span class="pd"></span>--</span>
  </div>
</div>

<!-- Battery -->
<div class="card">
  <div class="card-head"><div class="icon ic-b">B</div><h2>Battery</h2></div>
  <div class="row">
    <span class="lbl">BMS Status</span>
    <span class="pill off" id="bmsSt"><span class="pd"></span>Waiting Frames</span>
  </div>
  <div class="row">
    <span class="lbl">BMS Frames</span>
    <span id="bmsFrames" style="font-size:.8em;color:var(--text2)">HV:0 SOC:0 TH:0</span>
  </div>
  <div class="hero">
    <div class="soc-ring">
      <svg viewBox="0 0 120 120" width="120" height="120">
        <circle class="trk" cx="60" cy="60" r="52"/>
        <circle class="bar" id="socBar" cx="60" cy="60" r="52"
          stroke-dasharray="326.73" stroke-dashoffset="326.73"/>
      </svg>
      <div class="soc-val">
        <span class="soc-num" id="bSoc">--</span>
        <span class="soc-lbl">SOC</span>
      </div>
    </div>
    <div class="hg">
      <div><div class="hv" id="bVolt">--</div><div class="hl">Voltage</div></div>
      <div><div class="hv" id="bCurr">--</div><div class="hl">Current</div></div>
      <div><div class="hv" id="bTemp">--</div><div class="hl">Temp</div></div>
    </div>
    <div class="row" style="margin-top:8px;border-top:1px solid var(--border);padding-top:6px;font-size:.85em">
      <span class="lbl">12V bus</span>
      <span id="lvBus">--</span>
    </div>
  </div>
</div>

<!-- CAN Stats -->
<div class="card">
  <div class="card-head"><div class="icon ic-d">C</div><h2>CAN Bus</h2></div>
  <div class="sg">
    <div class="sb"><div class="sv" id="rxCnt">0</div><div class="sl">RX Frames</div></div>
    <div class="sb"><div class="sv" id="txCnt">0</div><div class="sl">TX Modified</div></div>
    <div class="sb"><div class="sv" id="crcErr">0</div><div class="sl">CRC Errors</div></div>
    <div class="sb"><div class="sv" id="fps">0.0</div><div class="sl">Frames/s</div></div>
  </div>
  <div class="row" style="margin-top:8px;font-size:.85em;color:var(--text2)">
    <span class="lbl">Frames seen</span>
    <span id="seenIds">0x398:0 0x3FD:0 0x3F8:0 0x318:0</span>
  </div>
  <div class="row" style="font-size:.85em;color:var(--text2)">
    <span class="lbl">Stalk → speed profile</span>
    <span id="stalkVal">--</span>
  </div>
</div>

<!-- Vehicle live diagnostics -->
<div class="card">
  <div class="card-head"><div class="icon ic-d">V</div><h2>Vehicle (live)</h2></div>
  <div class="row"><span class="lbl">Speed</span><span id="vSpeed">--</span></div>
  <div class="row"><span class="lbl">Steering</span><span id="vSteer">--</span></div>
  <div class="row"><span class="lbl">Motor torque</span><span id="vTorque">--</span></div>
  <div class="row"><span class="lbl">Brake pedal</span><span id="vBrake">--</span></div>
  <div class="row" style="margin-top:6px;border-top:1px solid var(--border);padding-top:6px">
    <span class="lbl">DAS_autopilot</span><span id="vDasAp">--</span>
  </div>
  <div class="row"><span class="lbl">DAS_autopilotBase</span><span id="vDasApBase">--</span></div>
  <div class="row"><span class="lbl">Hands-on level</span><span id="vHands">--</span></div>
  <div class="row"><span class="lbl">Lane change</span><span id="vLane">--</span></div>
  <div class="row"><span class="lbl">Blind spot</span><span id="vBlind">--</span></div>
  <div class="row"><span class="lbl">Forward coll. warn</span><span id="vFcw">--</span></div>
  <div class="row"><span class="lbl">Vision speed limit</span><span id="vVisLim">--</span></div>
</div>

<!-- Raw frame inspector — shows last bytes for IDs whose bit positions can vary by firmware -->
<div class="card">
  <div class="card-head"><div class="icon ic-d">R</div><h2>Raw frames</h2></div>
  <div class="row" style="font-family:monospace;font-size:.85em">
    <span class="lbl">0x145 ESP</span><span id="raw145">--</span>
  </div>
  <div class="row" style="font-family:monospace;font-size:.85em">
    <span class="lbl">0x39B DAS</span><span id="raw39b">--</span>
  </div>
  <div class="row" style="font-family:monospace;font-size:.85em">
    <span class="lbl">0x129 STR</span><span id="raw129">--</span>
  </div>
  <div class="row" style="font-size:.75em;color:var(--text2);margin-top:6px">
    <span style="white-space:normal">Watch which byte changes when you press brake (0x145) or turn the wheel (0x129) — that tells us the right bit position for this firmware.</span>
  </div>
</div>

<!-- Controls -->
<div class="card">
  <div class="card-head"><div class="icon ic-c">C</div><h2>Controls</h2></div>
  <button id="btnMode" class="btn-main btn-act" onclick="toggleMode()">ACTIVATE FSD</button>
  <div class="row">
    <span class="lbl">NAG Killer</span>
    <label class="sw"><input type="checkbox" id="swNag" onchange="cmd('nag',this.checked)"><span class="sl2"></span></label>
  </div>
  <div class="row">
    <span class="lbl">BMS Display</span>
    <label class="sw"><input type="checkbox" id="swBms" onchange="cmd('bms',this.checked)"><span class="sl2"></span></label>
  </div>
  <div class="row">
    <span class="lbl">Force FSD</span>
    <label class="sw"><input type="checkbox" id="swFsd" onchange="cmd('force_fsd',this.checked)"><span class="sl2"></span></label>
  </div>
  <div class="row">
    <span class="lbl">TLSSC Restore</span>
    <label class="sw"><input type="checkbox" id="swTlssc" onchange="cmd('tlssc_restore',this.checked)"><span class="sl2"></span></label>
  </div>
  <div class="row">
    <span class="lbl">CAN Dump</span>
    <label class="sw"><input type="checkbox" id="swDump" onchange="cmd('dump',this.checked)"><span class="sl2"></span></label>
  </div>
  <div class="row">
    <span class="lbl">Deep Sleep (sec)</span>
    <input type="number" id="numSleep" min="10" max="3600" style="width:60px;background:var(--card2);border:1px solid var(--border);color:var(--text);padding:4px;border-radius:4px;text-align:right" onchange="cmd('sleep',parseInt(this.value)*1000)">
  </div>
  <div class="row">
    <span class="lbl">HW Override</span>
    <select id="hwOv" style="background:var(--card2);border:1px solid var(--border);color:var(--text);padding:4px;border-radius:4px" onchange="cmd('hw_override',parseInt(this.value))">
      <option value="0">Auto-detect</option>
      <option value="1">Legacy</option>
      <option value="2">HW3</option>
      <option value="3">HW4</option>
    </select>
  </div>
  <div class="row">
    <span class="lbl">Ignore OTA detect</span>
    <label class="sw"><input type="checkbox" id="swOtaIg" onchange="cmd('ota_ignore',this.checked)"><span class="sl2"></span></label>
  </div>
  <div class="row">
    <span class="lbl">CAN trace (serial)</span>
    <label class="sw"><input type="checkbox" id="swTrace" onchange="cmd('can_trace',this.checked)"><span class="sl2"></span></label>
  </div>
  <div class="row" style="font-size:.8em;color:var(--text2)">
    <span class="lbl">OTA raw value</span>
    <span id="otaRaw">--</span>
  </div>
  <div class="row" style="margin-top:8px">
    <button onclick="if(confirm('Reboot the device? WiFi will drop and reconnect in ~3 s.'))cmd('reboot',true)"
            style="background:#3a1010;border:1px solid #5c1a1a;color:#ff8a8a;padding:6px 12px;border-radius:4px;cursor:pointer">Reboot device</button>
  </div>
</div>

<!-- WiFi Config -->
<div class="card">
  <div class="card-head"><div class="icon ic-c">W</div><h2>WiFi Configuration</h2></div>
  <div class="row">
    <span class="lbl">SSID</span>
    <input type="text" id="wifiSsid" maxlength="32" style="width:140px;background:var(--card2);border:1px solid var(--border);color:var(--text);padding:4px;border-radius:4px;text-align:right">
  </div>
  <div class="row">
    <span class="lbl">Password</span>
    <input type="password" id="wifiPass" maxlength="64" style="width:140px;background:var(--card2);border:1px solid var(--border);color:var(--text);padding:4px;border-radius:4px;text-align:right">
  </div>
  <div class="row">
    <span class="lbl">Stealth Mode (Hidden)</span>
    <label class="sw"><input type="checkbox" id="swWifiHid"><span class="sl2"></span></label>
  </div>
  <button class="btn-main btn-stop" onclick="saveWifi()" style="margin-top:12px">SAVE & RESTART WIFI</button>
</div>

<!-- SD Card -->
<div class="card">
  <div class="card-head"><div class="icon ic-d">S</div><h2>SD Card</h2></div>
  <div class="row">
    <span class="lbl">Dump Status</span>
    <span class="pill off" id="dumpSt"><span class="pd"></span>Idle</span>
  </div>
  <button id="btnFmt" class="btn-main btn-stop" onclick="sdFormat()" style="margin-top:8px">FORMAT SD CARD</button>
  <div id="fmtOut" style="font-size:.75em;color:var(--text2);margin-top:8px;display:none"></div>
</div>

<!-- Device Info -->
<div class="card">
  <div class="card-head"><div class="icon ic-d">D</div><h2>Device</h2></div>
  <div class="row">
    <span class="lbl">Firmware</span>
    <span id="fwBuild" style="font-size:.8em;color:var(--text2)">--</span>
  </div>
  <div class="row">
    <span class="lbl">Uptime</span>
    <span id="uptime" style="font-variant-numeric:tabular-nums">--</span>
  </div>
  <div class="row">
    <span class="lbl">WiFi Clients</span>
    <span id="wifiCl">--</span>
  </div>
</div>

<div class="foot">Tesla FSD ESP32 &middot; M5Stack ATOM Lite + ATOMIC CAN Base</div>
</div><!-- /wrap -->

<script>
var ws,rt,busy=0,wifiOnce=false;
var HW=['Unknown','Legacy','HW3','HW4'];
var CIRC=326.73;

function initWifi(d){
  if(wifiOnce)return;
  wifiOnce=true;
  document.getElementById('wifiSsid').value=d.wifi_ssid||'';
  document.getElementById('wifiPass').value=d.wifi_pass||'';
  document.getElementById('swWifiHid').checked=!!d.wifi_hidden;
}

function fmt(s){
  var h=Math.floor(s/3600),m=Math.floor((s%3600)/60),sc=s%60;
  return h+':'+(m<10?'0':'')+m+':'+(sc<10?'0':'')+sc;
}
function socCol(p){return p>60?'var(--accent)':p>30?'var(--yellow)':'var(--red)';}
function pill(id,on,txt,warnClass){
  var e=document.getElementById(id);
  e.className='pill '+(warnClass||''+(on?'on':'off'));
  e.innerHTML='<span class="pd"></span>'+txt;
}
function ring(p){
  var b=document.getElementById('socBar');
  b.style.strokeDashoffset=CIRC-(CIRC*Math.min(p,100)/100);
  b.style.stroke=socCol(p);
}

function upd(d){
  if(!d || Date.now() < busy) return;
  // Status
  pill('fsdSt', d.fsd_enabled, d.fsd_enabled?'Active':'Waiting');
  pill('opMode', d.op_mode===1, d.op_mode===1?'Active':'Listen-Only');

  var hwEl=document.getElementById('hwVer');
  if(hwEl){
    hwEl.className='pill '+(d.hw_version>0?'on':'off');
    hwEl.innerHTML='<span class="pd"></span>'+(HW[d.hw_version]||'?');
  }

  pill('nagSt', d.nag_killer, d.nag_killer?'ON':'OFF');
  pill('canVeh', d.can_vehicle_detected, d.can_vehicle_detected?'Detected':'No CAN Traffic');
  pill('bmsSt', d.bms && d.bms.seen, (d.bms && d.bms.seen)?'Live':'Waiting Frames');
  var bF=document.getElementById('bmsFrames');
  if(bF) bF.textContent='HV:'+(d.bms_hv_seen||0)+' SOC:'+(d.bms_soc_seen||0)+' TH:'+(d.bms_thermal_seen||0);

  // OTA banner
  var otaB=document.getElementById('otaBanner');
  if(otaB) otaB.style.display=d.ota?'block':'none';

  // Mode button
  var act=d.op_mode===1;
  var btn=document.getElementById('btnMode');
  if(btn){
    btn.textContent=act?'STOP FSD  \u2192  Listen-Only':'ACTIVATE FSD  \u2192  Active';
    btn.className='btn-main '+(act?'btn-stop':'btn-act');
  }

  // Switches sync
  if(document.getElementById('swNag')) document.getElementById('swNag').checked=d.nag_killer;
  if(document.getElementById('swBms')) document.getElementById('swBms').checked=d.bms_output;
  if(document.getElementById('swFsd')) document.getElementById('swFsd').checked=d.force_fsd;
  if(document.getElementById('swTlssc')) document.getElementById('swTlssc').checked=d.tlssc_restore;
  if(document.getElementById('swDump')) document.getElementById('swDump').checked=!!d.can_dump;
  
  if(document.activeElement.id!=='numSleep' && document.getElementById('numSleep'))
    document.getElementById('numSleep').value=Math.floor((d.sleep_ms||0)/1000);
  if(document.activeElement.id!=='hwOv' && document.getElementById('hwOv'))
    document.getElementById('hwOv').value=String(d.hw_override||0);
  if(document.getElementById('swOtaIg')) document.getElementById('swOtaIg').checked=!!d.ota_ignore;
  if(document.getElementById('swTrace')) document.getElementById('swTrace').checked=!!d.can_trace;
  if(document.getElementById('otaRaw')){
    var rv=(d.ota_raw===undefined?'?':String(d.ota_raw));
    var note=d.ota_ignore?' (detected, but ignored)':(d.ota?' (in progress)':' (idle)');
    document.getElementById('otaRaw').textContent=rv+note;
  }
  
  pill('dumpSt',d.can_dump,d.can_dump?'Recording':'Idle');

  // CAN stats
  if(document.getElementById('rxCnt')) document.getElementById('rxCnt').textContent=(d.rx_count||0).toLocaleString();
  if(document.getElementById('txCnt')) document.getElementById('txCnt').textContent=(d.tx_count||0).toLocaleString();
  if(document.getElementById('crcErr')) document.getElementById('crcErr').textContent=d.crc_errors||0;
  if(document.getElementById('fps')) document.getElementById('fps').textContent=(d.fps||0.0).toFixed(1);
  if(document.getElementById('seenIds'))
    document.getElementById('seenIds').textContent=
      '0x398:'+(d.seen_398||0)+' 0x3FD:'+(d.seen_3fd||0)+
      ' 0x3F8:'+(d.seen_3f8||0)+' 0x318:'+(d.seen_318||0);
  if(document.getElementById('stalkVal'))
    document.getElementById('stalkVal').textContent=
      'profile '+((d.speed_profile===undefined||d.speed_profile===null)?'?':d.speed_profile);

  // Vehicle live readouts. Tier enum: 0=NONE 1=HIGHWAY 2=ENHANCED 3=SELF_DRIVING 4=BASIC.
  var TIER=['NONE','HIGHWAY','ENHANCED','SELF_DRIVING','BASIC'];
  var setT=function(id,t){var el=document.getElementById(id);if(el)el.textContent=t;};
  setT('vSpeed', d.speed_seen?(d.vehicle_speed_kph.toFixed(1)+' kph (UI:'+d.ui_speed+')'):'--');
  setT('vSteer', d.steering_seen?(d.steering_deg.toFixed(1)+'°'):'--');
  setT('vTorque',d.torque_seen?(d.motor_torque_nm.toFixed(0)+' Nm'):'--');
  setT('vBrake', d.brake_seen?(d.brake?'pressed':'released'):'--');
  setT('vDasAp', d.das_ap_seen?((TIER[d.das_ap]||'?')+' ('+d.das_ap+')'):'--');
  setT('vDasApBase', d.das_ap_seen?((TIER[d.das_ap_base]||'?')+' ('+d.das_ap_base+')'):'--');
  setT('vHands', d.das_seen?String(d.das_hands_on):'--');
  setT('vLane',  d.das_seen?String(d.das_lane):'--');
  setT('vBlind', d.das_seen?String(d.das_blind):'--');
  setT('vFcw',   d.das_seen?String(d.das_fcw):'--');
  setT('vVisLim',d.das_seen?(d.das_vis_lim?(d.das_vis_lim*5+' kph'):'unset'):'--');
  setT('raw145', d.raw_145||'--');
  setT('raw39b', d.raw_39b||'--');
  setT('raw129', d.raw_129||'--');

  // Battery
  if(d.bms && d.bms.seen){
    var sn=document.getElementById('bSoc');
    if(sn){
      sn.textContent=d.bms.soc.toFixed(0)+'%';
      sn.style.color=socCol(d.bms.soc);
    }
    ring(d.bms.soc);
    if(document.getElementById('bVolt')) document.getElementById('bVolt').textContent=d.bms.voltage.toFixed(0)+'V';
    var ce=document.getElementById('bCurr');
    if(ce){
      ce.textContent=(d.bms.current>=0?'+':'')+d.bms.current.toFixed(1)+'A';
      ce.style.color=d.bms.current>=0?'var(--accent)':'var(--red)';
    }
    if(document.getElementById('bTemp')) document.getElementById('bTemp').textContent=d.bms.temp_min+'~'+d.bms.temp_max+'\u00b0C';
  }
  if(document.getElementById('lvBus')){
    if(d.lv_bus_seen){
      document.getElementById('lvBus').textContent=
        d.lv_bus_v.toFixed(2)+' V  /  '+d.lv_bus_a.toFixed(1)+' A';
    } else {
      document.getElementById('lvBus').textContent='--';
    }
  }

  // Device
  if(document.getElementById('fwBuild')) document.getElementById('fwBuild').textContent=d.fw_build;
  if(document.getElementById('uptime')) document.getElementById('uptime').textContent=fmt(d.uptime_s||0);
  if(document.getElementById('wifiCl')) document.getElementById('wifiCl').textContent=d.wifi_clients||0;
}

function sdFormat(){
  if(!confirm('Format SD card? All data will be lost.'))return;
  var btn=document.getElementById('btnFmt');
  var out=document.getElementById('fmtOut');
  btn.disabled=true;btn.textContent='FORMATTING\u2026';
  fetch('/sdformat').then(function(r){return r.json();}).then(function(d){
    out.style.display='block';
    out.style.color=d.ok?'var(--accent)':'var(--red)';
    out.textContent=d.msg+(d.ok?' \u2014 '+d.free_mb+' MB free':'');
  }).catch(function(){
    out.style.display='block';out.style.color='var(--red)';out.textContent='Request failed';
  }).then(function(){
    btn.disabled=false;btn.textContent='FORMAT SD CARD';
  });
}
function saveWifi(){
  var s=document.getElementById('wifiSsid').value;
  var p=document.getElementById('wifiPass').value;
  var h=document.getElementById('swWifiHid').checked;
  if(s.length<1){alert('SSID required');return;}
  if(p.length>0 && p.length<8){alert('Password must be 8+ chars');return;}
  if(confirm('WiFi settings will be updated and the device will restart.')){
    var b=document.activeElement; if(b&&b.tagName==='BUTTON'){b.disabled=true;b.textContent='SAVING...';}
    cmd('wifi_cfg',{ssid:s,pass:p,hidden:h});
  }
}
function cmd(c,v){
  if(ws&&ws.readyState===1) {
    ws.send(JSON.stringify({cmd:c,value:v}));
    busy = Date.now() + 3000;
  }
}
function toggleMode(){ cmd('mode',null); }

function conn(){
  ws=new WebSocket('ws://'+location.hostname+':81/');
  ws.onopen=function(){
    document.getElementById('dot').className='cdot';
    document.getElementById('connErr').style.display='none';
    clearTimeout(rt);
  };
  ws.onmessage=function(e){ try{var d=JSON.parse(e.data);initWifi(d);upd(d);}catch(x){} };
  ws.onclose=function(){
    document.getElementById('dot').className='cdot off';
    document.getElementById('connErr').style.display='block';
    rt=setTimeout(conn,2000);
  };
  ws.onerror=function(){ ws.close(); };
}
conn();
</script>
</body>
</html>
)rawliteral";

// ── JSON helpers ──────────────────────────────────────────────────────────────
static String json_escape(const char *s) {
    String out;
    for (; *s; ++s) {
        if (*s == '"')       out += "\\\"";
        else if (*s == '\\') out += "\\\\";
        else                 out += *s;
    }
    return out;
}

// ── JSON builder ──────────────────────────────────────────────────────────────
static String build_json() {
    uint32_t uptime_s = (millis() - g_start_ms) / 1000;
    bool can_vehicle_detected = false;
    if (g_state != nullptr && g_state->rx_count > 0) {
        can_vehicle_detected = (millis() - g_last_can_seen_ms) <= CAN_VEHICLE_ALIVE_MS;
    }

    // BMS sub-object
    char bms[128];
    if (g_state->bms_seen) {
        snprintf(bms, sizeof(bms),
            "{\"seen\":true,\"voltage\":%.1f,\"current\":%.1f,"
            "\"soc\":%.1f,\"temp_min\":%d,\"temp_max\":%d}",
            g_state->pack_voltage_v,
            g_state->pack_current_a,
            g_state->soc_percent,
            (int)g_state->batt_temp_min_c,
            (int)g_state->batt_temp_max_c);
    } else {
        strcpy(bms, "{\"seen\":false}");
    }

    // fps as fixed-point string
    char fps_s[12];
    snprintf(fps_s, sizeof(fps_s), "%.1f", g_fps);

    String j;
    j.reserve(512);
    j  = "{";
    j += "\"fsd_enabled\":";   j += g_state->fsd_enabled             ? "true" : "false"; j += ',';
    j += "\"op_mode\":";       j += (int)g_state->op_mode;            j += ',';
    j += "\"hw_version\":";    j += (int)g_state->hw_version;         j += ',';
    j += "\"hw_override\":";   j += (int)g_state->hw_override;        j += ',';
    j += "\"speed_profile\":"; j += (int)g_state->speed_profile;      j += ',';
    j += "\"ota_raw\":";       j += (int)g_state->ota_raw_state;      j += ',';
    j += "\"ota_ignore\":";    j += g_state->ota_ignore               ? "true" : "false"; j += ',';
    j += "\"can_trace\":";     j += g_state->can_trace                ? "true" : "false"; j += ',';
    // Vehicle dynamics + DAS readbacks (read-only, parsed from Party CAN).
    // *_seen flags let the UI render "--" when the bus doesn't carry that ID.
    j += "\"speed_seen\":";        j += g_state->speed_seen            ? "true" : "false"; j += ',';
    j += "\"vehicle_speed_kph\":"; j += g_state->vehicle_speed_kph;     j += ',';
    j += "\"ui_speed\":";          j += (int)g_state->ui_speed;         j += ',';
    j += "\"steering_seen\":";     j += g_state->steering_seen         ? "true" : "false"; j += ',';
    j += "\"steering_deg\":";      j += g_state->steering_angle_deg;    j += ',';
    j += "\"torque_seen\":";       j += g_state->torque_seen           ? "true" : "false"; j += ',';
    j += "\"motor_torque_nm\":";   j += g_state->motor_torque_nm;       j += ',';
    j += "\"brake_seen\":";        j += g_state->brake_seen            ? "true" : "false"; j += ',';
    j += "\"brake\":";             j += g_state->driver_brake_applied  ? "true" : "false"; j += ',';
    j += "\"das_seen\":";          j += g_state->das_seen              ? "true" : "false"; j += ',';
    j += "\"das_hands_on\":";      j += (int)g_state->das_hands_on_state; j += ',';
    j += "\"das_lane\":";          j += (int)g_state->das_lane_change;  j += ',';
    j += "\"das_blind\":";         j += (int)g_state->das_side_coll_warn; j += ',';
    j += "\"das_avoid\":";         j += (int)g_state->das_side_coll_avoid; j += ',';
    j += "\"das_fcw\":";           j += (int)g_state->das_fcw;          j += ',';
    j += "\"das_vis_lim\":";       j += (int)g_state->das_vision_speed_lim; j += ',';
    j += "\"das_ap_seen\":";       j += g_state->das_ap_seen           ? "true" : "false"; j += ',';
    j += "\"das_ap\":";            j += (int)g_state->das_autopilot;    j += ',';
    j += "\"das_ap_base\":";       j += (int)g_state->das_autopilot_base; j += ',';
    j += "\"lv_bus_seen\":";       j += g_state->lv_bus_seen           ? "true" : "false"; j += ',';
    j += "\"lv_bus_v\":";          j += g_state->lv_bus_voltage_v;       j += ',';
    j += "\"lv_bus_a\":";          j += g_state->lv_bus_current_a;       j += ',';
    // Raw byte snapshots for debugging firmware-version-specific bit positions.
    auto raw_arr = [&](const char *key, uint8_t dlc, const uint8_t *bytes) {
        j += '"'; j += key; j += "\":\"";
        if (dlc == 0) { j += "--"; }
        else {
            char hex[4];
            for (uint8_t i = 0; i < dlc && i < 8; i++) {
                snprintf(hex, sizeof(hex), "%02X ", bytes[i]);
                j += hex;
            }
        }
        j += "\",";
    };
    raw_arr("raw_145", g_state->raw_145_dlc, g_state->raw_145_bytes);
    raw_arr("raw_39b", g_state->raw_39b_dlc, g_state->raw_39b_bytes);
    raw_arr("raw_129", g_state->raw_129_dlc, g_state->raw_129_bytes);
    j += "\"seen_398\":";      j += g_state->seen_gtw_car_config;     j += ',';
    j += "\"seen_3fd\":";      j += g_state->seen_ap_control;         j += ',';
    j += "\"seen_3f8\":";      j += g_state->seen_follow_dist;        j += ',';
    j += "\"seen_318\":";      j += g_state->seen_gtw_car_state;      j += ',';
    // "ota" is the *effective* state (banner / TX-gate). When ota_ignore is
    // on we bypass the detection, so callers see false even if the raw bits
    // still match OTA_IN_PROGRESS_RAW_VALUE. The raw value is exposed
    // separately in ota_raw for diagnostics.
    j += "\"ota\":";           j += (g_state->tesla_ota_in_progress && !g_state->ota_ignore) ? "true" : "false"; j += ',';
    j += "\"nag_killer\":";    j += g_state->nag_killer               ? "true" : "false"; j += ',';
    j += "\"bms_output\":";    j += g_state->bms_output               ? "true" : "false"; j += ',';
    j += "\"force_fsd\":";     j += g_state->force_fsd                ? "true" : "false"; j += ',';
    j += "\"tlssc_restore\":"; j += g_state->tlssc_restore            ? "true" : "false"; j += ',';
    j += "\"can_vehicle_detected\":"; j += can_vehicle_detected       ? "true" : "false"; j += ',';
    j += "\"bms_hv_seen\":";   j += g_state->seen_bms_hv;              j += ',';
    j += "\"bms_soc_seen\":";  j += g_state->seen_bms_soc;             j += ',';
    j += "\"bms_thermal_seen\":"; j += g_state->seen_bms_thermal;       j += ',';
    j += "\"rx_count\":";      j += g_state->rx_count;                 j += ',';
    j += "\"tx_count\":";      j += g_state->frames_modified;          j += ',';
    j += "\"crc_errors\":";    j += g_state->crc_err_count;            j += ',';
    j += "\"fps\":";           j += fps_s;                             j += ',';
    j += "\"bms\":";           j += bms;                               j += ',';
    j += "\"uptime_s\":";      j += uptime_s;                          j += ',';
    j += "\"fw_build\":\"";    j += __DATE__;  j += ' '; j += __TIME__; j += "\",";
    j += "\"can_dump\":";      j += can_dump_active()                 ? "true" : "false"; j += ',';
    j += "\"sleep_ms\":";     j += g_state->sleep_idle_ms;            j += ',';
    j += "\"wifi_ssid\":\"";  j += json_escape(g_state->wifi_ssid);   j += "\",";
    j += "\"wifi_pass\":\"***\",";
    j += "\"wifi_hidden\":";  j += g_state->wifi_hidden               ? "true" : "false"; j += ',';
    j += "\"wifi_clients\":";  j += (int)WiFi.softAPgetStationNum();
    j += '}';
    return j;
}

// ── WebSocket event handler ───────────────────────────────────────────────────
static void ws_event(uint8_t num, WStype_t type,
                     uint8_t *payload, size_t length)
{
    if (type == WStype_CONNECTED) {
        // Push current state immediately on connect
        String json = build_json();
        g_ws.sendTXT(num, json.c_str(), json.length());
        return;
    }

    if (type != WStype_TEXT || g_state == nullptr || length == 0) return;

    // Use a slightly more robust way to find the value after the second colon
    char buf[256] = {};
    size_t n = (length < sizeof(buf) - 1) ? length : sizeof(buf) - 1;
    memcpy(buf, payload, n);

    // Find the "value" part of {"cmd":"xxx","value":yyy}
    const char *vptr = strstr(buf, "\"value\":");
    if (vptr) vptr = strstr(vptr, ":") + 1;

    if (strstr(buf, "\"mode\"")) {
        if (g_state->op_mode == OpMode_ListenOnly) {
            g_state->op_mode = OpMode_Active;
            if (g_can) g_can->setListenOnly(false);
            Serial.println("[Web] → Active mode");
        } else {
            g_state->op_mode = OpMode_ListenOnly;
            if (g_can) g_can->setListenOnly(true);
            Serial.println("[Web] → Listen-Only mode");
        }
        prefs_save(g_state);
    } else if (strstr(buf, "\"nag\"")) {
        if (vptr) {
            while (*vptr == ' ' || *vptr == ':') vptr++;
            g_state->nag_killer = (strncmp(vptr, "true", 4) == 0);
            Serial.printf("[Web] NAG Killer: %s\n", g_state->nag_killer ? "ON" : "OFF");
            prefs_save(g_state);
        }
    } else if (strstr(buf, "\"bms\"")) {
        if (vptr) {
            while (*vptr == ' ' || *vptr == ':') vptr++;
            g_state->bms_output = (strncmp(vptr, "true", 4) == 0);
            Serial.printf("[Web] BMS output: %s\n", g_state->bms_output ? "ON" : "OFF");
            prefs_save(g_state);
        }
    } else if (strstr(buf, "\"tlssc_restore\"")) {
        if (vptr) {
            while (*vptr == ' ' || *vptr == ':') vptr++;
            g_state->tlssc_restore = (strncmp(vptr, "true", 4) == 0);
            Serial.printf("[Web] TLSSC Restore: %s\n", g_state->tlssc_restore ? "ON" : "OFF");
            prefs_save(g_state);
        }
    } else if (strstr(buf, "\"force_fsd\"")) {
        if (vptr) {
            while (*vptr == ' ' || *vptr == ':') vptr++;
            g_state->force_fsd = (strncmp(vptr, "true", 4) == 0);
            Serial.printf("[Web] Force FSD: %s\n", g_state->force_fsd ? "ON" : "OFF");
            prefs_save(g_state);
        }
    } else if (strstr(buf, "\"dump\"")) {
        if (vptr) {
            while (*vptr == ' ' || *vptr == ':') vptr++;
            bool want = (strncmp(vptr, "true", 4) == 0);
            if (want) can_dump_start();
            else      can_dump_stop();
            Serial.printf("[Web] CAN Dump: %s\n", want ? "START" : "STOP");
        }
    } else if (strstr(buf, "\"sleep\"")) {
        if (vptr) {
            while (*vptr == ' ' || *vptr == ':') vptr++;
            uint32_t val = (uint32_t)atoi(vptr);
            if (val >= 10000) { // minimum 10s
                g_state->sleep_idle_ms = val;
                Serial.printf("[Web] Sleep timeout: %u ms\n", val);
                prefs_save(g_state);
            }
        }
    } else if (strstr(buf, "\"reboot\"")) {
        Serial.println("[Web] Reboot requested via dashboard");
        delay(200);  // let the WebSocket frame land in the browser
        ESP.restart();
    } else if (strstr(buf, "\"ota_ignore\"")) {
        if (vptr) {
            while (*vptr == ' ' || *vptr == ':') vptr++;
            g_state->ota_ignore = (strncmp(vptr, "true", 4) == 0);
            Serial.printf("[Web] OTA ignore: %s (raw value seen: %u)\n",
                          g_state->ota_ignore ? "ON" : "OFF",
                          (unsigned)g_state->ota_raw_state);
            prefs_save(g_state);
        }
    } else if (strstr(buf, "\"can_trace\"")) {
        if (vptr) {
            while (*vptr == ' ' || *vptr == ':') vptr++;
            g_state->can_trace = (strncmp(vptr, "true", 4) == 0);
            Serial.printf("[Web] CAN trace: %s\n", g_state->can_trace ? "ON" : "OFF");
            prefs_save(g_state);  // persisted so it survives the reset that
                                  // happens when the user opens a serial monitor
        }
    } else if (strstr(buf, "\"hw_override\"")) {
        if (vptr) {
            while (*vptr == ' ' || *vptr == ':') vptr++;
            int v = atoi(vptr);
            if (v >= TeslaHW_Unknown && v <= TeslaHW_HW4) {
                g_state->hw_override = (TeslaHWVersion)v;
                if (g_state->hw_override != TeslaHW_Unknown) {
                    fsd_apply_hw_version(g_state, g_state->hw_override);
                }
                const char *hw_str =
                    (v == TeslaHW_HW4)    ? "HW4"    :
                    (v == TeslaHW_HW3)    ? "HW3"    :
                    (v == TeslaHW_Legacy) ? "Legacy" : "Auto";
                Serial.printf("[Web] HW Override: %s\n", hw_str);
                prefs_save(g_state);
            }
        }
    } else if (strstr(buf, "\"wifi_cfg\"")) {
        // Find the "value":{ object start
        const char *vobj = strstr(buf, "\"value\":");
        if (vobj) {
            char *s = strstr(vobj, "\"ssid\":\"");
            char *p = strstr(vobj, "\"pass\":\"");
            char *h = strstr(vobj, "\"hidden\":");
            if (s) {
                s += 8;
                char *end = strchr(s, '\"');
                if (end) {
                    int len = end - s;
                    if (len > 32) len = 32;
                    if (memchr(s, '\\', len) == nullptr) {
                        memcpy(g_state->wifi_ssid, s, len);
                        g_state->wifi_ssid[len] = '\0';
                    }
                }
            }
            if (p) {
                p += 8;
                char *end = strchr(p, '\"');
                if (end) {
                    int len = end - p;
                    if (len > 64) len = 64;
                    if (memchr(p, '\\', len) == nullptr &&
                        !(len == 3 && memcmp(p, "***", 3) == 0)) {
                        memcpy(g_state->wifi_pass, p, len);
                        g_state->wifi_pass[len] = '\0';
                    }
                }
            }
            if (h) {
                h += 9;
                while (*h == ' ' || *h == ':') h++;
                if (strncmp(h, "true", 4) == 0) g_state->wifi_hidden = true;
                else if (strncmp(h, "false", 5) == 0) g_state->wifi_hidden = false;
            }
            Serial.printf("[Web] WiFi config: SSID=\"%s\" PASS=*** HIDDEN=%d\n",
                g_state->wifi_ssid, g_state->wifi_hidden);
            prefs_save(g_state);
            delay(500);
            ESP.restart();
        }
    }
}

// ── HTTP handlers ─────────────────────────────────────────────────────────────
static void handle_root() {
    g_http.send_P(200, "text/html", WEB_HTML);
}

static void handle_status() {
    if (g_state == nullptr) { g_http.send(503, "application/json", "{}"); return; }
    g_http.send(200, "application/json", build_json());
}

static void handle_sdformat() {
    String result = sd_format_card();
    g_http.send(200, "application/json", result);
}

// ── Public API ────────────────────────────────────────────────────────────────
void web_dashboard_init(FSDState *state, CanDriver *can) {
    g_state       = state;
    g_can         = can;
    g_start_ms    = millis();
    g_last_fps_ms = millis();
    g_last_rx     = state ? state->rx_count : 0;
    g_last_can_seen_ms = (state && state->rx_count > 0) ? millis() : 0;

    g_http.on("/",           HTTP_GET, handle_root);
    g_http.on("/api/status", HTTP_GET, handle_status);
    g_http.on("/sdformat",   HTTP_GET, handle_sdformat);
    g_http.begin();

    g_ws.begin();
    g_ws.onEvent(ws_event);

    Serial.println("[Web] HTTP :80  WS :81 — ready");
}

void web_dashboard_update() {
    if (g_state == nullptr) return;   // init was never called (WiFi failed)

    g_http.handleClient();
    g_ws.loop();

    // FPS calculation + 1 Hz WebSocket broadcast
    uint32_t now = millis();
    if ((now - g_last_fps_ms) >= 1000u) {
        uint32_t rx = g_state->rx_count;
        float    dt = (now - g_last_fps_ms) / 1000.0f;
        if (rx != g_last_rx) g_last_can_seen_ms = now;
        g_fps        = (float)(rx - g_last_rx) / dt;
        g_last_rx    = rx;
        g_last_fps_ms = now;

        String json = build_json();
        g_ws.broadcastTXT(json.c_str(), json.length());
    }
}
