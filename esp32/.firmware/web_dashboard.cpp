/*
 * web_dashboard.cpp — HTTP + WebSocket dashboard for Tesla CAN Controller ESP32
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
#include "nvs_settings.h"
#include "wifi_manager.h"
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
// Tabbed interface: Dashboard, Controls, CAN Bus, Device
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
<title>Tesla CAN Controller</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
:root{
  --bg:#0a0a1a;--card:#111827;--card2:#1a1f35;
  --accent:#00d4aa;--accent2:#00b894;
  --red:#ff6b6b;--yellow:#ffd93d;--blue:#4dabf7;
  --border:#1e293b;--text:#e2e8f0;--text2:#94a3b8;--text3:#475569
}
body{font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',Roboto,sans-serif;
  background:var(--bg);color:var(--text);min-height:100vh;
  -webkit-tap-highlight-color:transparent}
.wrap{max-width:480px;margin:0 auto;padding:12px 16px 32px}

/* ── Header ── */
.hdr{text-align:center;padding:16px 0 8px;position:relative}
.hdr h1{font-size:1.5em;font-weight:700;
  background:linear-gradient(135deg,var(--accent),var(--blue));
  -webkit-background-clip:text;-webkit-text-fill-color:transparent;
  letter-spacing:-.02em}
.hdr .sub{font-size:.65em;color:var(--text3);margin-top:2px;
  letter-spacing:.1em;text-transform:uppercase}
.cdot{position:absolute;right:0;top:22px;width:10px;height:10px;
  border-radius:50%;background:var(--accent);
  box-shadow:0 0 10px var(--accent);transition:.4s}
.cdot.off{background:var(--red);box-shadow:0 0 10px var(--red)}

/* ── Banners ── */
.ota{display:none;background:rgba(255,107,107,.1);border:1px solid rgba(255,107,107,.4);
  border-radius:12px;padding:10px 14px;margin-bottom:10px;text-align:center;
  color:var(--red);font-weight:700;font-size:.85em;letter-spacing:.04em;
  animation:pulse 1s ease-in-out infinite}
.sleep-ban{display:none;background:rgba(148,93,255,.1);border:1px solid rgba(148,93,255,.4);
  border-radius:12px;padding:10px 14px;margin-bottom:10px;text-align:center;
  color:#b87aff;font-weight:700;font-size:.85em;letter-spacing:.04em}
@keyframes pulse{0%,100%{opacity:1}50%{opacity:.55}}
.err{display:none;color:var(--red);text-align:center;font-size:.78em;padding:8px;
  background:rgba(255,107,107,.07);border-radius:10px;margin-bottom:8px;
  border:1px solid rgba(255,107,107,.18)}

/* ── Tabs ── */
.tabs{display:flex;gap:4px;margin-bottom:12px;
  background:var(--card);border-radius:12px;padding:4px;border:1px solid var(--border)}
.tab{flex:1;text-align:center;padding:10px 4px;border-radius:8px;
  font-size:.72em;font-weight:600;color:var(--text3);cursor:pointer;
  transition:all .2s;text-transform:uppercase;letter-spacing:.06em;
  user-select:none;-webkit-user-select:none}
.tab.active{background:rgba(0,212,170,.12);color:var(--accent)}
.tab:active{opacity:.7}
.pane{display:none}.pane.active{display:block}

/* ── Cards ── */
.card{background:var(--card);border-radius:16px;padding:14px;
  margin-bottom:10px;border:1px solid var(--border)}
.card-head{display:flex;align-items:center;gap:8px;margin-bottom:10px}
.icon{width:26px;height:26px;border-radius:8px;display:flex;
  align-items:center;justify-content:center;font-size:.8em;font-weight:700}
.ic-s{background:rgba(0,212,170,.14);color:var(--accent)}
.ic-b{background:rgba(77,171,247,.14);color:var(--blue)}
.ic-c{background:rgba(255,217,61,.14);color:var(--yellow)}
.ic-d{background:rgba(148,163,184,.14);color:var(--text2)}
.ic-p{background:rgba(184,122,255,.14);color:#b87aff}
.card-head h2{font-size:.75em;font-weight:600;color:var(--text2);
  text-transform:uppercase;letter-spacing:.07em}

/* ── Rows ── */
.row{display:flex;justify-content:space-between;align-items:center;padding:8px 0}
.row+.row{border-top:1px solid rgba(255,255,255,.04)}
.lbl{color:var(--text2);font-size:.82em}

/* ── Pills ── */
.pill{display:inline-flex;align-items:center;gap:5px;
  padding:3px 10px;border-radius:20px;font-size:.78em;font-weight:600}
.pill.on{background:rgba(0,212,170,.14);color:var(--accent)}
.pill.off{background:rgba(71,85,105,.22);color:var(--text3)}
.pill.warn{background:rgba(255,107,107,.14);color:var(--red)}
.pd{width:6px;height:6px;border-radius:50%;flex-shrink:0;
  background:currentColor;box-shadow:0 0 5px currentColor}

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
.sep{font-size:.68em;color:var(--text3);text-transform:uppercase;letter-spacing:.08em;
  padding:10px 0 4px;border-bottom:1px solid rgba(255,255,255,.06);margin-bottom:2px}

/* ── Footer ── */
.foot{text-align:center;padding:12px 0 0;font-size:.6em;color:var(--text3)}
/* ── Inputs ── */
.inp{width:100%;background:var(--card2);border:1px solid var(--border);border-radius:8px;
  padding:8px 10px;color:var(--text);font-size:.85em;outline:none}
.inp:focus{border-color:var(--accent)}
.inp-lbl{font-size:.75em;color:var(--text2);margin-bottom:4px}
</style>
</head>
<body>
<div class="wrap">

<!-- Header -->
<div class="hdr">
  <h1>Tesla CAN Controller</h1>
  <div class="sub">ESP32 CAN Controller</div>
  <div class="cdot" id="dot"></div>
</div>
<div id="connErr" class="err">Connection lost &mdash; retrying&hellip;</div>
<div id="otaBanner" class="ota">&#9888;&#xFE0F; OTA IN PROGRESS &mdash; TX SUSPENDED</div>
<div id="sleepBanner" class="sleep-ban">&#x1F4A4; CAR ASLEEP &mdash; TX SUSPENDED</div>

<!-- Tabs -->
<div class="tabs">
  <div class="tab active" onclick="showTab(0)">Dashboard</div>
  <div class="tab" onclick="showTab(1)">Controls</div>
  <div class="tab" onclick="showTab(2)">CAN</div>
  <div class="tab" onclick="showTab(3)">BMS</div>
  <div class="tab" onclick="showTab(4)">Device</div>
</div>

<!-- ═══ TAB 0: Dashboard ═══ -->
<div class="pane active" id="p0">

  <!-- Status -->
  <div class="card">
    <div class="card-head"><div class="icon ic-s">S</div><h2>Status</h2></div>
    <div class="row">
      <span class="lbl">Mode</span>
      <span class="pill off" id="opMode"><span class="pd"></span>--</span>
    </div>
    <div class="row">
      <span class="lbl">Hardware</span>
      <span class="pill off" id="hwVer"><span class="pd"></span>--</span>
    </div>
    <div class="row">
      <span class="lbl">Vehicle</span>
      <span class="pill off" id="canVeh"><span class="pd"></span>--</span>
    </div>
    <div class="row">
      <span class="lbl">BAN Shield</span>
      <span class="pill off" id="banSt"><span class="pd"></span>--</span>
    </div>
  </div>

</div>

<!-- ═══ TAB 1: Controls ═══ -->
<div class="pane" id="p1">

  <div class="card">
    <div class="card-head"><div class="icon ic-c">T</div><h2>Transmission Mode</h2></div>
    <div class="row">
      <span class="lbl">TX Active (off = Listen-Only)</span>
      <label class="sw"><input type="checkbox" id="swTxMode" onchange="cmd('mode',this.checked)"><span class="sl2"></span></label>
    </div>
  </div>

  <div class="card">
    <div class="card-head"><div class="icon ic-s">&#9881;</div><h2>Features</h2></div>
    <div class="row">
      <span class="lbl">FSD Inject</span>
      <label class="sw"><input type="checkbox" id="swFsdInject" onchange="cmd('fsd_inject',this.checked)"><span class="sl2"></span></label>
    </div>
    <div class="row">
      <span class="lbl">NAG Killer</span>
      <label class="sw"><input type="checkbox" id="swNag" onchange="cmd('nag',this.checked)"><span class="sl2"></span></label>
    </div>
    <div class="row">
      <span class="lbl">BAN Shield</span>
      <label class="sw"><input type="checkbox" id="swBan" onchange="cmd('ban_shield',this.checked)"><span class="sl2"></span></label>
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
      <span class="lbl">Speed Chime Suppress (HW4 only)</span>
      <label class="sw"><input type="checkbox" id="swChime" onchange="cmd('chime',this.checked)"><span class="sl2"></span></label>
    </div>
    <div class="row">
      <span class="lbl">Emergency Vehicle Detect (HW4 only)</span>
      <label class="sw"><input type="checkbox" id="swEmerg" onchange="cmd('emerg_veh',this.checked)"><span class="sl2"></span></label>
    </div>

    <div class="sep">Settings</div>
    <div class="row">
      <span class="lbl">Auto-Activate on Wake</span>
      <label class="sw"><input type="checkbox" id="swAutoWake" onchange="cmd('auto_wake',this.checked)"><span class="sl2"></span></label>
    </div>
  </div>

</div>

<!-- ═══ TAB 2: CAN Bus ═══ -->
<div class="pane" id="p2">

  <div class="card">
    <div class="card-head"><div class="icon ic-d">C</div><h2>CAN Statistics</h2></div>
    <div class="sg">
      <div class="sb"><div class="sv" id="rxCnt">0</div><div class="sl">RX Frames</div></div>
      <div class="sb"><div class="sv" id="txCnt">0</div><div class="sl">TX Modified</div></div>
      <div class="sb"><div class="sv" id="crcErr">0</div><div class="sl">CRC Errors</div></div>
      <div class="sb"><div class="sv" id="fps">0.0</div><div class="sl">Frames/s</div></div>
    </div>
  </div>

  <div class="card">
    <div class="card-head"><div class="icon ic-b">F</div><h2>Frame Counters</h2></div>
    <div class="row">
      <span class="lbl">NAG Echoes</span>
      <span id="nagEcho" style="font-variant-numeric:tabular-nums">0</span>
    </div>
    <div class="row">
      <span class="lbl">TLSSC Restores</span>
      <span id="tlsscCnt" style="font-variant-numeric:tabular-nums">0</span>
    </div>
  </div>
  <div class="row">
    <span class="lbl">CAN Dump</span>
    <label class="sw"><input type="checkbox" id="swDump" onchange="cmd('dump',this.checked)"><span class="sl2"></span></label>
  </div>
</div>

<!-- ═══ TAB 3: BMS ═══ -->
<div class="pane" id="p3">

  <div class="card">
    <div class="card-head"><div class="icon ic-b">⚡</div><h2>Battery Management</h2></div>
    <div class="row">
      <span class="lbl">BMS Enabled</span>
      <label class="sw"><input type="checkbox" id="swBmsEn" onchange="cmd('bms_enable',this.checked)"><span class="sl2"></span></label>
    </div>
    <div class="row">
      <span class="lbl">Status</span>
      <span class="pill off" id="bmsSt"><span class="pd"></span>--</span>
    </div>
    <div class="row">
      <span class="lbl">Frames Seen</span>
      <span id="bmsFrames" style="font-variant-numeric:tabular-nums">0</span>
    </div>
  </div>

  <div class="card">
    <div class="card-head"><div class="icon ic-b">V</div><h2>Live Data</h2></div>
    <div class="row">
      <span class="lbl">Pack Voltage</span>
      <span id="bmsVolt">--V</span>
    </div>
    <div class="row">
      <span class="lbl">Pack Current</span>
      <span id="bmsCurr">--A</span>
    </div>
    <div class="row">
      <span class="lbl">State of Charge</span>
      <span id="bmsSoc" class="ring">--</span>
    </div>
    <div class="row">
      <span class="lbl">Temperature</span>
      <span id="bmsTemp">--°C</span>
    </div>
  </div>

</div>

<!-- ═══ TAB 4: Device ═══ -->
<div class="pane" id="p4">

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

  <div class="card">
    <div class="card-head"><div class="icon ic-b">W</div><h2>WiFi AP</h2></div>
    <div class="row">
      <span class="lbl">Current SSID</span>
      <span id="wifiSSID" style="font-size:.82em;color:var(--text2)">--</span>
    </div>
    <div class="row">
      <span class="lbl">Current Password</span>
      <span id="wifiPass" style="font-size:.82em;color:var(--text2)">--</span>
    </div>
    <div class="sep">Change credentials</div>
    <div style="margin-bottom:8px">
      <div class="inp-lbl">New SSID (1&ndash;32 chars)</div>
      <input class="inp" id="inSSID" maxlength="32" placeholder="Tesla-FSD" autocomplete="off">
    </div>
    <div style="margin-bottom:10px">
      <div class="inp-lbl">Password (min 8 chars, leave empty = unchanged)</div>
      <input class="inp" type="password" id="inPass" maxlength="63" placeholder="&bull;&bull;&bull;&bull;&bull;&bull;&bull;&bull;" autocomplete="new-password">
    </div>
    <button class="btn-main btn-act" onclick="saveWifi()" style="margin-bottom:4px">SAVE</button>
    <div id="wifiOut" style="font-size:.75em;margin-top:4px;display:none"></div>
  </div>

  <div class="card">
    <div class="card-head"><div class="icon ic-d">D</div><h2>Device Info</h2></div>
    <div class="row">
      <span class="lbl">Firmware</span>
      <span id="fwBuild" style="font-size:.78em;color:var(--text2)">--</span>
    </div>
    <div class="row">
      <span class="lbl">Uptime</span>
      <span id="uptime" style="font-variant-numeric:tabular-nums">--</span>
    </div>
    <div class="row">
      <span class="lbl">WiFi Clients</span>
      <span id="wifiCl">--</span>
    </div>
    <div class="row">
      <span class="lbl">Speed Profile</span>
      <span id="speedProf" style="font-variant-numeric:tabular-nums">--</span>
    </div>
  </div>

  <div class="card">
    <button class="btn-main btn-stop" onclick="reboot()">REBOOT</button>
  </div>

</div>

<div class="foot">Tesla CAN Controller ESP32 &middot; M5Stack ATOM Lite</div>
</div><!-- /wrap -->

<script>
var ws,rt;
var HW=['Unknown','Legacy','HW3','HW4'];

function showTab(i){
  var tabs=document.querySelectorAll('.tab');
  var panes=document.querySelectorAll('.pane');
  for(var t=0;t<tabs.length;t++){
    tabs[t].className='tab'+(t===i?' active':'');
    panes[t].className='pane'+(t===i?' active':'');
  }
}
function fmt(s){
  var h=Math.floor(s/3600),m=Math.floor((s%3600)/60),sc=s%60;
  return h+':'+(m<10?'0':'')+m+':'+(sc<10?'0':'')+sc;
}
function pill(id,on,txt){
  var e=document.getElementById(id);
  e.className='pill '+(on?'on':'off');
  e.innerHTML='<span class="pd"></span>'+txt;
}

function upd(d){
  // Dashboard — Status pills
  pill('opMode', d.op_mode===1, d.op_mode===1?'TX Mode':'Listen-Only');
  var hwEl=document.getElementById('hwVer');
  hwEl.className='pill '+(d.hw_version>0?'on':'off');
  hwEl.innerHTML='<span class="pd"></span>'+(HW[d.hw_version]||'?');
  pill('canVeh', d.can_vehicle_detected, d.can_vehicle_detected?'Detected':'No CAN');
  var banTxt=d.ban_shield_armed?'Armed ('+d.ban_shield_blocks+')':(d.ban_shield?'Learning':'Off');
  pill('banSt', d.ban_shield_armed, banTxt);

  // Banners
  document.getElementById('otaBanner').style.display=d.ota?'block':'none';
  document.getElementById('sleepBanner').style.display=d.car_asleep?'block':'none';

  // Switches sync
  document.getElementById('swTxMode').checked=d.op_mode===1;
  document.getElementById('swFsdInject').checked=d.fsd_inject_enabled;
  document.getElementById('swNag').checked=d.nag_killer;
  document.getElementById('swFsd').checked=d.force_fsd;
  document.getElementById('swTlssc').checked=d.tlssc_restore;
  document.getElementById('swDump').checked=!!d.can_dump;
  pill('dumpSt',d.can_dump,d.can_dump?'Recording':'Idle');
  document.getElementById('swAutoWake').checked=d.auto_wake;
  document.getElementById('swBan').checked=d.ban_shield;
  document.getElementById('swChime').checked=d.chime;
  document.getElementById('swEmerg').checked=d.emerg_veh;

  // CAN stats
  document.getElementById('rxCnt').textContent=d.rx_count.toLocaleString();
  document.getElementById('txCnt').textContent=d.tx_count.toLocaleString();
  document.getElementById('crcErr').textContent=d.crc_errors;
  document.getElementById('fps').textContent=d.fps.toFixed(1);
  document.getElementById('nagEcho').textContent=d.nag_echo_count;
  document.getElementById('tlsscCnt').textContent=d.tlssc_restore_count;

  // BMS Tab
  document.getElementById('swBmsEn').checked=d.bms_enabled;
  var bmsOk=d.bms_enabled && d.bms_seen;
  pill('bmsSt', bmsOk, bmsOk?'Reading':'--');
  var bmsF=d.bms_hv_seen+d.bms_soc_seen+d.bms_thermal_seen;
  document.getElementById('bmsFrames').textContent=bmsF;
  document.getElementById('bmsVolt').textContent=d.bms_voltage.toFixed(1)+'V';
  document.getElementById('bmsCurr').textContent=(d.bms_current>=0?'+':'')+d.bms_current.toFixed(1)+'A';
  document.getElementById('bmsSoc').textContent=d.bms_soc.toFixed(1)+'%';
  document.getElementById('bmsTemp').textContent=d.bms_temp_min+'–'+d.bms_temp_max+'°C';

  document.getElementById('wifiSSID').textContent=d.wifi_ssid||'--';
  document.getElementById('wifiPass').textContent=d.wifi_pass||'--';

  // Device
  document.getElementById('fwBuild').textContent=d.fw_build;
  document.getElementById('uptime').textContent=fmt(d.uptime_s);
  document.getElementById('wifiCl').textContent=d.wifi_clients;
  document.getElementById('speedProf').textContent=d.speed_profile;
}

function cmd(c,v){
  if(ws&&ws.readyState===1) ws.send(JSON.stringify({cmd:c,value:v}));
}
function reboot(){
  if(!confirm('Reboot the device?'))return;
  fetch('/reboot').catch(function(){});
}
function saveWifi(){
  var ssid=document.getElementById('inSSID').value.trim();
  var pass=document.getElementById('inPass').value;
  if(ssid.length<1||ssid.length>32){alert('SSID: 1-32 characters');return;}
  if(pass.length>0&&pass.length<8){alert('Password: 8 characters minimum');return;}
  var out=document.getElementById('wifiOut');
  out.style.display='none';
  fetch('/wificreds',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({ssid:ssid,pass:pass})})
    .then(function(r){return r.json();})
    .then(function(d){
      out.style.display='block';
      out.style.color=d.ok?'var(--accent)':'var(--red)';
      out.textContent=d.msg;
    }).catch(function(){
      out.style.display='block';out.style.color='var(--red)';out.textContent='Network error';
    });
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
function conn(){
  ws=new WebSocket('ws://'+location.hostname+':81/');
  ws.onopen=function(){
    document.getElementById('dot').className='cdot';
    document.getElementById('connErr').style.display='none';
    clearTimeout(rt);
  };
  ws.onmessage=function(e){ try{upd(JSON.parse(e.data));}catch(x){} };
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

// ── JSON builder ──────────────────────────────────────────────────────────────
static String build_json() {
    uint32_t uptime_s = (millis() - g_start_ms) / 1000;
    bool can_vehicle_detected = false;
    if (g_state != nullptr && g_state->rx_count > 0) {
        can_vehicle_detected = (millis() - g_last_can_seen_ms) <= CAN_VEHICLE_ALIVE_MS;
    }

    // fps as fixed-point string
    char fps_s[12];
    snprintf(fps_s, sizeof(fps_s), "%.1f", g_fps);

    String j;
    j.reserve(640);
    j  = "{";
    j += "\"fsd_enabled\":";   j += g_state->fsd_enabled             ? "true" : "false"; j += ',';
    j += "\"op_mode\":";       j += (int)g_state->op_mode;            j += ',';
    j += "\"hw_version\":";    j += (int)g_state->hw_version;         j += ',';
    j += "\"ota\":";           j += g_state->tesla_ota_in_progress    ? "true" : "false"; j += ',';
    j += "\"car_asleep\":";    j += g_state->car_asleep               ? "true" : "false"; j += ',';
    j += "\"nag_killer\":";    j += g_state->nag_killer               ? "true" : "false"; j += ',';
    j += "\"force_fsd\":";     j += g_state->force_fsd                ? "true" : "false"; j += ',';
    j += "\"fsd_inject_enabled\":"; j += g_state->fsd_inject_enabled  ? "true" : "false"; j += ',';
    j += "\"tlssc_restore\":"; j += g_state->tlssc_restore            ? "true" : "false"; j += ',';
    j += "\"auto_wake\":";     j += g_state->auto_activate_on_wake    ? "true" : "false"; j += ',';
    j += "\"chime\":";         j += g_state->suppress_speed_chime     ? "true" : "false"; j += ',';
    j += "\"emerg_veh\":";     j += g_state->emergency_vehicle_detect ? "true" : "false"; j += ',';
    j += "\"can_vehicle_detected\":"; j += can_vehicle_detected       ? "true" : "false"; j += ',';
    j += "\"nag_echo_count\":"; j += g_state->nag_echo_count;           j += ',';
    j += "\"tlssc_restore_count\":"; j += g_state->tlssc_restore_count; j += ',';
    j += "\"speed_profile\":"; j += g_state->speed_profile;             j += ',';
    j += "\"rx_count\":";      j += g_state->rx_count;                 j += ',';
    j += "\"tx_count\":";      j += g_state->frames_modified;          j += ',';
    j += "\"crc_errors\":";    j += g_state->crc_err_count;            j += ',';
    j += "\"fps\":";           j += fps_s;                             j += ',';
    j += "\"uptime_s\":";      j += uptime_s;                          j += ',';
    j += "\"fw_build\":\"";    j += __DATE__;  j += ' '; j += __TIME__; j += "\",";
    j += "\"can_dump\":";      j += can_dump_active()                 ? "true" : "false"; j += ',';
    j += "\"bms_enabled\":";   j += g_state->bms_enabled               ? "true" : "false"; j += ',';
    j += "\"bms_seen\":";      j += g_state->bms_seen                  ? "true" : "false"; j += ',';
    j += "\"bms_voltage\":";   snprintf(fps_s, sizeof(fps_s), "%.2f", g_state->pack_voltage_v); j += fps_s; j += ',';
    j += "\"bms_current\":";   snprintf(fps_s, sizeof(fps_s), "%.2f", g_state->pack_current_a); j += fps_s; j += ',';
    j += "\"bms_soc\":";       snprintf(fps_s, sizeof(fps_s), "%.1f", g_state->soc_percent); j += fps_s; j += ',';
    j += "\"bms_temp_min\":";  j += (int)g_state->batt_temp_min_c;      j += ',';
    j += "\"bms_temp_max\":";  j += (int)g_state->batt_temp_max_c;      j += ',';
    j += "\"bms_hv_seen\":";   j += g_state->seen_bms_hv;              j += ',';
    j += "\"bms_soc_seen\":";  j += g_state->seen_bms_soc;             j += ',';
    j += "\"bms_thermal_seen\":"; j += g_state->seen_bms_thermal;      j += ',';
    j += "\"ban_shield\":";       j += g_state->ban_shield               ? "true" : "false"; j += ',';
    j += "\"ban_shield_armed\":"; j += g_state->ban_shield_armed          ? "true" : "false"; j += ',';
    j += "\"ban_shield_blocks\":"; j += g_state->ban_shield_blocks;                           j += ',';
    j += "\"wifi_ssid\":\"";
    String ap_ssid = String(wifi_ap_ssid());
    for (size_t i = 0; i < ap_ssid.length(); i++) {
        char c = ap_ssid[i];
        if (c == '"' || c == '\\') j += '\\';
        j += c;
    }
    j += "\",";
    j += "\"wifi_pass\":\"";
    String ap_pass = String(wifi_ap_pass());
    for (size_t i = 0; i < ap_pass.length(); i++) {
        char c = ap_pass[i];
        if (c == '"' || c == '\\') j += '\\';
        j += c;
    }
    j += "\",";
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

    // Parse the short command JSON without a heavy JSON library.
    // Expected format: {"cmd":"mode"}  {"cmd":"nag","value":true}  etc.
    char buf[128] = {};
    size_t n = (length < sizeof(buf) - 1) ? length : sizeof(buf) - 1;
    memcpy(buf, payload, n);

    if (strstr(buf, "\"mode\"")) {
        bool want = (strstr(buf, "true") != nullptr);
        g_state->op_mode = want ? OpMode_Active : OpMode_ListenOnly;
        if (g_can) g_can->setListenOnly(!want);
        Serial.printf("[Web] \u2192 %s\n", want ? "TX Mode" : "Listen-Only");
    } else if (strstr(buf, "\"fsd_inject\"")) {
        g_state->fsd_inject_enabled = (strstr(buf, "true") != nullptr);
        Serial.printf("[Web] FSD Inject: %s\n", g_state->fsd_inject_enabled ? "ON" : "OFF");
        nvs_settings_save(g_state);
    } else if (strstr(buf, "\"nag\"")) {
        g_state->nag_killer = (strstr(buf, "true") != nullptr);
        Serial.printf("[Web] NAG Killer: %s\n", g_state->nag_killer ? "ON" : "OFF");
        nvs_settings_save(g_state);
    } else if (strstr(buf, "\"tlssc_restore\"")) {
        g_state->tlssc_restore = (strstr(buf, "true") != nullptr);
        Serial.printf("[Web] TLSSC Restore: %s\n", g_state->tlssc_restore ? "ON" : "OFF");
        nvs_settings_save(g_state);
    } else if (strstr(buf, "\"force_fsd\"")) {
        g_state->force_fsd = (strstr(buf, "true") != nullptr);
        Serial.printf("[Web] Force FSD: %s\n", g_state->force_fsd ? "ON" : "OFF");
        nvs_settings_save(g_state);
    } else if (strstr(buf, "\"auto_wake\"")) {
        g_state->auto_activate_on_wake = (strstr(buf, "true") != nullptr);
        Serial.printf("[Web] Auto-Activate on Wake: %s\n", g_state->auto_activate_on_wake ? "ON" : "OFF");
        nvs_settings_save(g_state);
    } else if (strstr(buf, "\"chime\"")) {
        g_state->suppress_speed_chime = (strstr(buf, "true") != nullptr);
        Serial.printf("[Web] Speed Chime Suppress: %s\n", g_state->suppress_speed_chime ? "ON" : "OFF");
        nvs_settings_save(g_state);
    } else if (strstr(buf, "\"emerg_veh\"")) {
        g_state->emergency_vehicle_detect = (strstr(buf, "true") != nullptr);
        Serial.printf("[Web] Emergency Vehicle Detect: %s\n", g_state->emergency_vehicle_detect ? "ON" : "OFF");
        nvs_settings_save(g_state);
    } else if (strstr(buf, "\"dump\"")) {
      bool want = (strstr(buf, "true") != nullptr);
      if (want) can_dump_start();
      else      can_dump_stop();
      Serial.printf("[Web] CAN Dump: %s\n", want ? "START" : "STOP");
    } else if (strstr(buf, "\"bms_enable\"")) {
        g_state->bms_enabled = (strstr(buf, "true") != nullptr);
        if (!g_state->bms_enabled) {
            g_state->bms_seen = false;
            g_state->pack_voltage_v = 0.0f;
            g_state->pack_current_a = 0.0f;
            g_state->soc_percent = 0.0f;
            g_state->batt_temp_min_c = 0;
            g_state->batt_temp_max_c = 0;
        }
        Serial.printf("[Web] BMS: %s\n", g_state->bms_enabled ? "Enabled" : "Disabled");
        nvs_settings_save(g_state);
    } else if (strstr(buf, "\"ban_shield\"")) {
        g_state->ban_shield = (strstr(buf, "true") != nullptr);
        if (!g_state->ban_shield) {
            g_state->ban_shield_armed = false;
            g_state->ban_shield_blocks = 0;
            memset(g_state->ban_shield_snapshot_valid, 0, sizeof(g_state->ban_shield_snapshot_valid));
        }
        Serial.printf("[Web] BAN Shield: %s\n", g_state->ban_shield ? "Enabled" : "Disabled");
        nvs_settings_save(g_state);
        } else if (strstr(buf, "\"dump\"")) {
          bool want = (strstr(buf, "true") != nullptr);
          if (want) can_dump_start();
          else      can_dump_stop();
          Serial.printf("[Web] CAN Dump: %s\n", want ? "START" : "STOP");
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

static void handle_reboot() {
    g_http.send(200, "text/plain", "Rebooting...");
    g_http.client().flush();
    delay(200);
    ESP.restart();
}

static String extract_json_str(const String &body, const char *key) {
    String needle = String("\"") + key + "\":\"";
    int idx = body.indexOf(needle);
    if (idx < 0) return "";
    int start = idx + needle.length();
    int end   = body.indexOf('"', start);
    if (end < 0) return "";
    return body.substring(start, end);
}

static void handle_wificreds() {
    if (g_http.method() == HTTP_GET) {
        // Return current SSID only (never expose password)
        String j = "{\"ssid\":\""; j += wifi_ap_ssid(); j += "\"}";
        g_http.send(200, "application/json", j);
        return;
    }
    // POST — save new credentials
    String body = g_http.arg("plain");
    String ssid = extract_json_str(body, "ssid");
    String pass = extract_json_str(body, "pass");
    if (ssid.length() < 1 || ssid.length() > 32) {
        g_http.send(400, "application/json",
            "{\"ok\":false,\"msg\":\"Invalid SSID (1-32 chars)\"}" );
        return;
    }
    if (pass.length() > 0 && pass.length() < 8) {
        g_http.send(400, "application/json",
            "{\"ok\":false,\"msg\":\"Password too short (min 8)\"}" );
        return;
    }
    nvs_wifi_save(ssid.c_str(), pass.c_str());
    g_http.send(200, "application/json",
        "{\"ok\":true,\"msg\":\"Saved \\u2014 reboot to apply\"}" );
}

// ── Public API ────────────────────────────────────────────────────────────────
void web_dashboard_init(FSDState *state, CanDriver *can) {
    g_state       = state;
    g_can         = can;
    g_start_ms    = millis();
    g_last_fps_ms = millis();
    g_last_rx     = state ? state->rx_count : 0;
    g_last_can_seen_ms = (state && state->rx_count > 0) ? millis() : 0;

    g_http.on("/",           HTTP_GET,  handle_root);
    g_http.on("/api/status", HTTP_GET,  handle_status);
    g_http.on("/sdformat",   HTTP_GET,  handle_sdformat);
    g_http.on("/reboot",     HTTP_GET,  handle_reboot);
    g_http.on("/wificreds",  HTTP_GET,  handle_wificreds);
    g_http.on("/wificreds",  HTTP_POST, handle_wificreds);
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
