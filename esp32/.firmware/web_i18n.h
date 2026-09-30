#pragma once
// web_i18n.h — Chinese (zh-CN) translation layer for the ESP32 web dashboard.
//
// Design: the upstream English strings are NEVER modified. This file only adds
// a runtime translator (dictionary + a 中文/EN toggle button injected into the
// page header). web_dashboard.cpp includes this header and injects
// WEB_I18N_HTML just before </body> — a 2-line change, no C++ logic touched.
//
// Behavior:
//  - First visit: follows the browser language (zh-* -> Chinese, else English).
//  - Toggle persists in localStorage ("teslaFsdLang").
//  - Switching to Chinese translates live; switching back to English reloads.
//  - WebSocket-driven dynamic content is translated via MutationObserver.
//  - Upstream strings missing from DICT simply stay English (graceful).
//
// To update translations, edit the generator (gen_i18n.py) and re-run it.

#define WEB_I18N_HTML R"rawliteral(
<script>
(function(){
"use strict";
/* English UI text -> Chinese. Keys match DOM textContent after collapsing
   whitespace (same normalization the translator applies). */
var DICT = {
 "Connection lost — retrying…": "连接已断开 — 正在重试…",
 "⚠️ 2026.14.x firmware enforcement active.": "⚠️ 2026.14.x 固件限制已生效。",
 "⚠️ IN-CAR AUTOPARK — CAN TX PAUSED": "⚠️ 车内自动泊车 — CAN 发送已暂停",
 "⚠️ OTA UPDATE IN PROGRESS — CAN TX SUSPENDED": "⚠️ OTA 更新进行中 — CAN 发送已挂起",
 "⚠️ Signal Map DAS id not seen on this bus.": "⚠️ 当前总线上未发现 Signal Map 的 DAS id。",
 "⚠️ Volatile storage — download events before power-off; they are lost on reboot.": "⚠️ 易失性存储 — 断电前请下载事件记录，重启后会丢失。",
 "— the standard parser can't read AP-state on this bus.": "— 标准解析器无法在此总线上读取 AP 状态。",
 "The configured DAS id isn't arriving, so AP-state can't be read and the nag killer is paused. Set": "配置的 DAS id 一直没有到来，无法读取 AP 状态，NAG 消除已暂停。设为",
 "for auto, or fix the mapping / tap.": "以自动识别，或修正映射 / 点按检测。",
 "Tesla added a preflight check in 2026.14.x that disables autosteer the moment any CAN frame touches": "特斯拉在 2026.14.x 加入了预检：任何 CAN 帧一碰到",
 ". Symptom on the dash:": "。仪表盘上的症状：",
 "\"Autopilot turning off\"": "\"自动驾驶正在关闭\"",
 "Autopilot turning off": "自动驾驶正在关闭",
 "appears within a second of stalk engagement, then AP immediately disengages. Listen-Only mode is safe.": "在拨杆接管约一秒后出现，随后 AP 立即退出。仅监听模式是安全的。",
 "(delay injection until AP is engaged) is on the Flipper build only right now — on the ESP32, engage AP from the stalk first, then turn on injection. Dismiss if you're on pre-14.x firmware.": "（等 AP 接管后再延迟注入）目前仅 Flipper 版本支持 — 在 ESP32 上请先用拨杆接管 AP，再开启注入。如为 14.x 之前固件可忽略本提示。",
 "FSD Status": "FSD 状态",
 "AP Status": "AP 状态",
 "NAG Killer": "NAG 消除",
 "Controls": "控制",
 "CAN Bus": "CAN 总线",
 "Black-box": "黑匣子",
 "CAN Dump": "CAN 抓包",
 "HTTP CAN Log": "HTTP CAN 日志",
 "BMS Display": "BMS 显示",
 "WiFi Configuration": "WiFi 配置",
 "OTA Firmware Update": "OTA 固件更新",
 "SD Card": "SD 卡",
 "Storage": "存储",
 "Device": "设备",
 "Hardware": "硬件",
 "Firmware": "固件",
 "Activate": "激活",
 "Deactivate": "停用",
 "⚑ MARK NOW": "⚓ 立即标记",
 "RE-CHECK": "重新检测",
 "Tap Check": "点按检测",
 "SAVE & RESTART WIFI": "保存并重启 WiFi",
 "RESTART DEVICE": "重启设备",
 "Restart device?": "确定重启设备吗？",
 "The device will reboot immediately and the web connection will drop briefly.": "设备将立即重启，网页连接会短暂中断。",
 "Cancel": "取消",
 "Dismiss": "忽略",
 "Got it": "知道了",
 "Sign In": "登录",
 "Connect to WiFi": "连接 WiFi",
 "SELECT FIRMWARE (.bin)": "选择固件（.bin）",
 "Upload a .bin firmware file. Device will reboot after a successful update.": "上传 .bin 固件文件，更新成功后设备将重启。",
 "Error: Please select a .bin firmware file": "错误：请选择 .bin 固件文件",
 "STREAM LOG AND SAVE": "开始串流并保存",
 "Ready to collect a candump file in this browser.": "已就绪，可在此浏览器中采集 candump 文件。",
 "FORMAT SD CARD": "格式化 SD 卡",
 "DELETE ALL EVENTS": "删除全部事件",
 "Delete all recorded events from the device?": "确定删除设备上所有已记录的事件吗？",
 "No events recorded yet.": "暂无已记录的事件。",
 "Save mapping": "保存映射",
 "Apply this profile": "应用此配置",
 "Expand to setup": "展开设置",
 "Live": "实时",
 "Idle": "空闲",
 "Active": "已激活",
 "Disabled": "已禁用",
 "Off": "关闭",
 "No": "否",
 "Yes": "是",
 "NEW": "新增",
 "Ready": "就绪",
 "Recording": "录制中",
 "Streaming": "串流中",
 "Collecting": "采集中",
 "Preparing...": "准备中…",
 "Demo": "演示",
 "Dev": "开发版",
 "Mode": "模式",
 "Listen-Only": "仅监听",
 "AP Branch/Tier (experimental)": "AP 分支/层级（实验性）",
 "Continuous AP": "持续 AP",
 "Force FSD": "强制 FSD",
 "FSD activate": "FSD 激活",
 "Force HW3": "强制 HW3",
 "Force HW4": "强制 HW4",
 "Force Legacy": "强制旧版",
 "China Mode": "中国模式",
 "Right-Hand Drive (RHD)": "右舵驾驶（RHD）",
 "RHD markets only — do NOT enable while driving on the right.": "仅限右舵市场 — 在靠右行驶地区行车时请勿启用。",
 "FSD Unlock": "FSD 解锁",
 "Summon EU Unlock": "欧盟版召唤解锁",
 "Track Mode (experimental)": "赛道模式（实验性）",
 "Compressor Overclock": "压缩机超频",
 "Post-drive Cooling": "停车后散热",
 "Precondition": "电池预处理",
 "battery preheat trigger (0x082)": "电池预热触发（0x082）",
 "max cooling": "最大制冷",
 "Continue on Green": "绿灯自动通行",
 "Suppress Chime": "屏蔽提示音",
 "Stability Assist": "稳定辅助",
 "Handling Balance": "操控平衡",
 "(stable → rotation)": "（稳定 → 甩尾）",
 "Stealth Mode (Hidden)": "隐身模式（隐藏）",
 "Telemetry Off (experimental)": "关闭遥测（实验性）",
 "Soft Engage": "柔和接管",
 "Soft Engage (14.x, exp.)": "柔和接管（14.x，实验性）",
 "Instant Engage (exp.)": "即时接管（实验性）",
 "Minimal Inject (exp.)": "最小注入（实验性）",
 "AP-First": "AP 优先",
 "AP-First (14.x)": "AP 优先（14.x）",
 "Nag Burst (14.x, exp.)": "NAG 连发（14.x，实验性）",
 "Nag EPAS-faithful (14.x, exp.)": "NAG EPAS 高还原（14.x，实验性）",
 "Abort Guard (14.x, exp.)": "中断保护（14.x，实验性）",
 "TLSSC Restore": "TLSSC 恢复",
 "pairs with TLSSC": "与 TLSSC 联动",
 "Signal Map (advanced, 14.x)": "信号映射（高级，14.x）",
 "Override where the nag killer reads AP-state / hands-on / steering. Leave DAS id": "覆盖 NAG 消除读取 AP 状态 / 手握检测 / 转向信号的位置。DAS id 留空则",
 "DAS id (0x..)": "DAS id（0x..）",
 "Steer id (0x..)": "转向 id（0x..）",
 "Steer hi/lo byte": "转向高/低字节",
 "AP-state byte/sh/mask": "AP 状态 字节/位移/掩码",
 "Hands-on byte/sh/mask": "手握检测 字节/位移/掩码",
 "Auto-detect": "自动检测",
 "Auto-detect needs 0x398 — many Model 3/Y never send it. Pick your car if detection is wrong.": "自动检测需要 0x398 — 很多 Model 3/Y 从不发送该帧，若识别错误请手动选择车型。",
 "Best guess:": "最佳推测：",
 "— confirm in Service Mode → CAN Port": "— 请在 Service Mode → CAN Port 中确认",
 "for auto-detect. byte 0-7, shift 0-7, mask hex.": "用于自动检测。字节 0-7，位移 0-7，掩码为十六进制。",
 "Experimental & non-persistent — injects a UI branch/tier hint only, reverts when injection stops; unverified and may be a ban signal. Off by default.": "实验性且不持久 — 仅注入 UI 分支/层级提示，停止注入后恢复；未经验证，可能成为封禁依据。默认关闭。",
 "Experimental & unverified — clears reachable telemetry flags only (not the Vehicle-bus ECU log-upload). Does NOT guarantee reduced detection.": "实验性且未经验证 — 仅清除可达的遥测标记（不影响车辆总线 ECU 的日志上传），不保证降低被检测概率。",
 "Experimental — Vehicle-bus; not car-validated. Defaults to rear-biased (rotation 100) + 30% stability — fun with a safety margin. Raise stability for stock feel.": "实验性 — 车辆总线；未经实车验证。默认为偏后驱（rotation 100）+ 30% 稳定性 — 在安全余量内体验乐趣，提高稳定性可获得接近原厂的感受。",
 "CAN Errors": "CAN 错误",
 "CAN Vehicle": "车辆 CAN",
 "Dump Status": "抓包状态",
 "Filter IDs": "过滤 ID",
 "RX Frames": "接收帧",
 "TX Frames": "发送帧",
 "Frames/s": "帧/秒",
 "Dropped": "丢弃",
 "Buffered": "已缓冲",
 "Filtered": "已过滤",
 "0 frames": "0 帧",
 "frames / rx-missed": "帧 / 接收丢失",
 "No CAN frames collected. Nothing saved.": "未采集到 CAN 帧，未保存任何内容。",
 "No frames seen — check wiring / that the car is awake.": "未收到任何帧 — 请检查接线 / 确认车辆已唤醒。",
 "no 0x370 on this tap — wrong bus for the nag killer": "此接入点没有 0x370 — NAG 消除接错了总线",
 "HW unconfirmed — 0x399 reading assumed; verdict may change once HW is detected.": "硬件未确认 — 暂按 0x399 读数判断，硬件识别后结论可能变化。",
 "Looks like variant": "疑似变体",
 "Connect to run a check, or press Re-check.": "连接后运行检测，或点击重新检测。",
 "Listens a few seconds and reports whether each feature can work on the bus this device is tapped into. Pure read-only — nothing is transmitted.": "监听数秒，报告各功能在当前接入总线上是否可用。纯只读 — 不发送任何数据。",
 "No CAN Traffic": "无 CAN 流量",
 "Body/comfort bus": "车身/舒适总线",
 "DAS state readable": "DAS 状态可读",
 "Records the key diagnostic CAN IDs around anomalies (aborts, bus-off, manual marks) to the device only — never uploaded. Use the toggle below to enable or disable.": "仅在设备本地记录异常（中断、bus-off、手动标记）前后的关键诊断 CAN ID — 永不上传。用下方开关启用或禁用。",
 "Auto-record": "自动录制",
 "HTTP CAN log is available only in Listen-Only mode.": "HTTP CAN 日志仅在仅监听模式下可用。",
 "Switch to Listen-Only mode before starting HTTP CAN log.": "启动 HTTP CAN 日志前，请先切换到仅监听模式。",
 "Log ready in phone memory:": "日志已就绪（手机内存）：",
 "Save/share requested for": "已请求保存/分享：",
 "Share cancelled or failed:": "分享已取消或失败：",
 "Server response:": "服务器响应：",
 "This browser does not support HTTP stream collection.": "此浏览器不支持 HTTP 串流采集。",
 ". Trying download link...": "。正在尝试下载链接…",
 "Connecting to HTTP stream...": "正在连接 HTTP 串流…",
 "connection closed": "连接已关闭",
 "Stream stopped:": "串流已停止：",
 "The device starts its own access point by default. Optionally set a network below; when a network name is set, the device tries to connect to it on boot and starts its own access point if it cannot connect.": "设备默认会启动自带热点。也可在下方设置网络；设置网络名称后，设备启动时会优先连接该网络，失败则回退到自带热点。",
 "Network Name": "网络名称",
 "Network Password": "网络密码",
 "Password": "密码",
 "Access Point": "接入点",
 "Enter the admin username and the WiFi AP password.": "请输入管理员用户名和 WiFi 热点密码。",
 "Authentication Required": "需要身份验证",
 "Username": "用户名",
 "Authentication failed": "身份验证失败",
 "Authorization": "授权",
 "No OTA Partition": "无 OTA 分区",
 "No OTA partition": "无 OTA 分区",
 "OTA Partition": "OTA 分区",
 "Partition Safety": "分区安全",
 "Ignore OTA": "忽略 OTA",
 "Deep Sleep (sec)": "深度睡眠（秒）",
 "Display Brightness (%)": "显示亮度（%）",
 "Display Timeout (s)": "显示超时（秒）",
 "Administration": "管理",
 "Voltage": "电压",
 "Current": "电流",
 "Temp": "温度",
 "SOC": "电量",
 "Battery": "电池",
 "Uptime": "运行时间",
 "BMS Frames": "BMS 帧",
 "BMS Status": "BMS 状态",
 "Free & open source ·": "免费开源 ·",
 "ESP32 CAN Controller ·": "ESP32 CAN 控制器 ·",
 "support the research": "支持本研究",
 "mirror read-back": "镜像回读",
 "optional": "可选",
 "default": "默认",
 "single": "单路",
 "dual-CAN": "双路 CAN",
 "Legacy": "旧版",
 "del": "删除",
 "Status": "状态",
 "State": "状态",
 "Stream": "串流",
 "Stage": "阶段",
 "Stage2": "阶段2",
 "Listening on the bus…": "正在监听总线… ",
 "Reachable frames:": "可达帧：",
 "(presence only — not proof injection actuates them).": "（仅表示存在 — 不能证明注入能实际生效）。",
 "Waiting Frames": "等待帧",
 "Waiting": "等待中",
 "Capturing": "抓拍中",
 "Armed": "已布防",
 "Detected": "已检测到",
 "Listening…": "监听中…",
 "Done": "完成",
 "ON": "开",
 "OFF": "关",
 "Device restart triggered": "已触发设备重启",
 "Network password must be empty or 8+ chars": "网络密码须为空或 8 位以上",
 "Password must be empty or 8+ chars": "密码须为空或 8 位以上",
 "SSID required": "需要填写 SSID",
 "OTA writes to the next app partition when available. The new image is kept only after 15 s of runtime; a crash or power cut before that restores the previous firmware. Keep USB reflashing available as a recovery path.": "OTA 会在可用时写入下一个 app 分区；新固件需连续运行 15 秒后才会被保留，在此之前崩溃或断电会回滚到上一版固件。请保留 USB 线刷作为恢复手段。",
 "WiFi Clients": "WiFi 客户端"
};

var LS_KEY = "teslaFsdLang";

function norm(s){ return String(s == null ? "" : s).replace(/\s+/g, " ").trim(); }

var KEYS = Object.keys(DICT).sort(function(a,b){ return b.length - a.length; });

function zhText(raw){
  var n = norm(raw);
  if(!n) return null;
  if(Object.prototype.hasOwnProperty.call(DICT, n)) return DICT[n];
  var i, k, tail, out, j, s2, head;
  // Longest-prefix match for templated strings, e.g. "Best guess: <dynamic>".
  for(i = 0; i < KEYS.length; i++){
    k = KEYS[i];
    if(k.length < 6 || n.indexOf(k) !== 0) continue;
    tail = n.slice(k.length);
    if(tail !== "" && /[A-Za-z0-9]/.test(tail.charAt(0))) continue; // word boundary
    out = DICT[k] + tail;
    // Suffix match on the remainder, e.g. "... -- confirm in Service Mode".
    for(j = 0; j < KEYS.length; j++){
      s2 = KEYS[j];
      if(s2.length < 6 || out.length <= s2.length) continue;
      if(out.slice(-s2.length) !== s2) continue;
      head = out.slice(0, out.length - s2.length);
      if(/[A-Za-z0-9]/.test(head.charAt(head.length - 1)) && /[A-Za-z0-9]/.test(s2.charAt(0))) continue;
      out = head + DICT[s2];
      break;
    }
    return out;
  }
  return null;
}

var SKIP_TAGS = {SCRIPT:1, STYLE:1, TEXTAREA:1, NOSCRIPT:1, CODE:1, PRE:1};
var ATTR_NAMES = ["placeholder", "title", "aria-label", "alt"];

function translateTextNode(node){
  var t = zhText(node.nodeValue);
  if(t !== null && t !== node.nodeValue) node.nodeValue = t;
}

function translateElement(el){
  var i, a, v, t;
  if(!el.getAttribute) return;
  for(i = 0; i < ATTR_NAMES.length; i++){
    a = ATTR_NAMES[i];
    v = el.getAttribute(a);
    if(v){
      t = zhText(v);
      if(t !== null && t !== v) el.setAttribute(a, t);
    }
  }
}

function walk(root){
  var els, i, n, walker;
  if(root.querySelectorAll){
    els = root.querySelectorAll("*");
    for(i = 0; i < els.length; i++) translateElement(els[i]);
  }
  if(root.nodeType === 1) translateElement(root);
  walker = document.createTreeWalker(root, NodeFilter.SHOW_TEXT, {
    acceptNode: function(node){
      var p = node.parentElement;
      if(p && SKIP_TAGS[p.tagName]) return NodeFilter.FILTER_REJECT;
      return NodeFilter.FILTER_ACCEPT;
    }
  });
  while((n = walker.nextNode())) translateTextNode(n);
}

function currentIsZh(){ return lang === "zh"; }

// Translate native dialogs (alert/confirm) issued by the page's own script.
(function(){
  if(window.__i18nAlertWrapped) return;
  window.__i18nAlertWrapped = true;
  var _alert = window.alert ? window.alert.bind(window) : null;
  var _confirm = window.confirm ? window.confirm.bind(window) : null;
  if(_alert) window.alert = function(msg){ _alert(currentIsZh() ? (zhText(msg) || msg) : msg); };
  if(_confirm) window.confirm = function(msg){ return _confirm(currentIsZh() ? (zhText(msg) || msg) : msg); };
})();

function applyLang(){
  if(!currentIsZh() || !document.documentElement) return;
  walk(document.documentElement);
}

var lang = "en";
try{
  var saved = window.localStorage.getItem(LS_KEY);
  if(saved === "zh" || saved === "en"){ lang = saved; }
  else if(((navigator.language || navigator.userLanguage) || "").toLowerCase().indexOf("zh") === 0){ lang = "zh"; }
}catch(e){}

var btn = document.createElement("button");
btn.id = "langToggle";
btn.type = "button";
function paintBtn(){ btn.textContent = currentIsZh() ? "EN" : "\u4e2d\u6587"; }
paintBtn();
btn.style.cssText = "position:absolute;left:0;top:26px;z-index:50;background:rgba(0,212,170,.12);"
  + "border:1px solid rgba(0,212,170,.45);color:#00d4aa;font-size:.72em;font-weight:700;"
  + "padding:5px 10px;border-radius:20px;cursor:pointer;letter-spacing:.05em";
btn.setAttribute("aria-label", "Switch language / \u5207\u6362\u8bed\u8a00");
btn.onclick = function(){
  lang = currentIsZh() ? "en" : "zh";
  try{ window.localStorage.setItem(LS_KEY, lang); }catch(e){}
  paintBtn();
  if(currentIsZh()){ applyLang(); }
  else { window.location.reload(); }
};

function mount(){
  var hdr = document.querySelector(".hdr");
  if(hdr){ hdr.appendChild(btn); }
  else if(document.body){ document.body.insertBefore(btn, document.body.firstChild); }
  applyLang();
}

if(document.readyState === "loading"){
  document.addEventListener("DOMContentLoaded", mount);
} else {
  mount();
}

// Translate WebSocket-driven live updates as they land in the DOM.
if("MutationObserver" in window){
  var mo = new MutationObserver(function(muts){
    var i, j, m, added, nd;
    if(!currentIsZh()) return;
    for(i = 0; i < muts.length; i++){
      m = muts[i];
      if(m.type === "characterData"){ translateTextNode(m.target); }
      else{
        added = m.addedNodes;
        for(j = 0; j < added.length; j++){
          nd = added[j];
          if(nd.nodeType === 3){ translateTextNode(nd); }
          else if(nd.nodeType === 1){ walk(nd); }
        }
      }
    }
  });
  var startWatch = function(){
    if(document.documentElement) mo.observe(document.documentElement,
      {childList: true, subtree: true, characterData: true});
  };
  if(document.readyState === "loading"){
    document.addEventListener("DOMContentLoaded", startWatch);
  } else {
    startWatch();
  }
}
})();
</script>
)rawliteral"
