# Tesla FSD Unlock for ESP32 (OBD-II Plug & Play)

> ESP32 port of [hypery11/flipper-tesla-fsd](https://github.com/hypery11/flipper-tesla-fsd) — same CAN logic, different hardware, with a built-in WiFi dashboard.

Unlock Tesla FSD with an ESP32 + CAN transceiver via OBD-II. No Flipper Zero needed — ~¥100 total cost.

> [!CAUTION]
> **✅ ESP32 Port — TESTED ON VEHICLE (Model 3 2022 HW3)**
>
> This firmware has been tested on a **Tesla Model 3 (2022, HW3)** and works well in real-world use. CAN logic is faithfully ported from hypery11/flipper-tesla-fsd.
>
> **If you test this on another vehicle model, please report your results in this PR thread.**

> [!IMPORTANT]
> The device boots in **Listen-Only mode** by default and will **not transmit any CAN frames** until the user explicitly switches to Active mode via the physical button or Web Dashboard UI. This ensures safe first-boot behavior.

---

## What's This?

This project takes the CAN bus logic from [hypery11's Flipper Zero FSD unlocker](https://github.com/hypery11/flipper-tesla-fsd) and ports it to the ESP32 platform (M5Stack ATOM Lite + ATOMIC CAN Base), adding a WiFi web dashboard for real-time monitoring and control.

### What Was Ported

All CAN protocol handling from hypery11's Flipper Zero implementation (`fsd_handler.c`), including:

- FSD activation bit manipulation (bit46 on `0x3FD`)
- HW3/HW4/Legacy auto-detection via `0x398` GTW_carConfig
- NAG Killer (EPAS `0x370` counter+1 echo with handsOnLevel spoofing)
- Speed profile mapping from follow-distance stalk
- OTA update detection and automatic TX suspension
- ISA speed warning chime suppression (HW4)
- TLSSC Restore via DAS config spoof (`0x331`)
- CRC/checksum recalculation after frame modification
- DLC length validation on all handlers

### What Was Added (New)

**WiFi Access Point + Web Dashboard** — inspired by [wjsall's ESP32 WiFi Web version](https://github.com/wjsall/tesla-fsd-controller) and [tuncasoftbildik's Tesla-style UI](https://github.com/tuncasoftbildik/tesla-can-mod):

- **WiFi AP mode** — connects without internet, SSID: `Tesla-FSD`, password: `12345678`
- **Tesla dark theme UI** — mobile-first responsive design (optimized for phone in portrait)
  - Tabbed interface: Dashboard, Controls, CAN Bus, Device
  - Dark background (#0a0a1a) with accent gradients, inspired by Tesla's in-car UI
  - All HTML/CSS/JS embedded in firmware (no external CDN dependencies)
- **Real-time WebSocket push** — 1 Hz state updates via WebSocket on port 81
- **Dashboard tab** — FSD status, operation mode, HW version, vehicle detection
- **Controls tab** — Toggle switches for all features + mode activation button
- **CAN Bus tab** — RX frame count, TX modified count, CRC errors, frames/second, NAG echo and TLSSC restore counters
- **Device tab** — firmware build date, uptime counter, WiFi client count, speed profile
- **OTA Warning Banner** — pulsing red alert when vehicle OTA update is detected (installing only)
- **Sleep Banner** — purple alert when car is asleep (no CAN traffic)
- **Connection Status** — green/red dot indicator with auto-reconnect on WebSocket disconnect
- **NVS Persistence** — all toggle settings saved to flash and restored on boot
- **Auto-Activate on Wake** — optional: automatically switch to Active mode on car wake or ESP32 boot
- **REST API** — `GET /api/status` returns full JSON state

### CAN Driver Abstraction

Dual CAN driver support (compile-time switch):
- **ESP32 TWAI** — for M5Stack ATOM Lite + ATOMIC CAN Base (CA-IS3050G transceiver)
- **MCP2515 SPI** — for generic ESP32 + MCP2515 CAN module setups

---

## Features

| Feature | CAN ID | HW Compat | Description |
|---------|--------|-----------|-------------|
| **FSD Unlock** | `0x3FD` mux0 | All (Legacy/HW3/HW4) | bit46 = 1 activates FSD |
| **NAG Killer** | `0x370` | All | Suppresses hands-on-wheel reminder via counter+1 echo |
| **Force FSD** | `0x3FD` mux0 | All | Bypass UI selection check — FSD always active |
| **Speed Profile** | `0x3FD` mux2 | All | Follow-distance stalk maps to speed offset |
| **TLSSC Restore** | `0x331` | All | DAS config spoof — restores tier via autopilot config |
| **ISA Chime Suppress** | `0x399` | **HW4 only** | Kills speed warning chime |
| **Emergency Vehicle Detect** | `0x3FD` mux0 bit59 | **HW4 only** | Enables emergency vehicle detection bit |
| **OTA Protection** | `0x318` | All | Auto-stops TX when OTA *installing* detected |
| **Car Sleep / Wake** | — | All | Detects car sleep (no CAN), auto-resumes on wake |
| **Auto-Activate on Wake** | — | All | Automatically switches to Active mode on wake / boot |
| **HW Auto-Detect** | `0x398` | All | Reads GTW_carConfig with fallback detection |
| **Listen-Only Mode** | — | All | Default on boot, passive monitoring only |
| **NVS Persistence** | — | All | Saves toggle settings to flash (survives reboot) |
| **WiFi Dashboard** | — | All | Real-time web UI at 192.168.4.1 |

### Persisted Settings (NVS)

These settings are saved to flash and restored automatically on boot:

| Setting | Default | Description |
|---------|---------|-------------|
| NAG Killer | ON | Suppress hands-on-wheel nag |
| Speed Chime Suppress | ON | ISA chime suppress (HW4 only) |
| Force FSD | OFF | Bypass FSD UI selection check |
| TLSSC Restore | OFF | DAS config spoof for tier restore |
| Auto-Activate on Wake | OFF | Auto-switch to Active on boot/wake |
| Emergency Vehicle Detect | OFF | Enable EVD bit (HW4 only) |

---

## Hardware

| Component | Description | Price |
|-----------|-------------|-------|
| [M5Stack ATOM Lite](https://docs.m5stack.com/en/core/ATOM%20Lite) | ESP32-PICO-D4, 24×24mm | ~¥60 |
| [ATOMIC CAN Base](https://docs.m5stack.com/en/atom/Atomic%20CAN%20Base) | CA-IS3050G CAN transceiver | ~¥40 |
| OBD-II male plug + 30cm cable | Connects to vehicle Party CAN bus | ~¥15 |

**Total: ~¥100** (vs Flipper Zero + CAN Add-On ~¥1500)

### Alternative Hardware

Any ESP32 board + CAN transceiver works. Change the build environment in `platformio.ini`:
- ESP32 + SN65HVD230 (TWAI driver, cheapest option)
- ESP32 + MCP2515 module (SPI driver, use `esp32-mcp2515` env)
- ESP32-C3/S3 Super Mini + SN65HVD230

---

## Wiring

### OBD-II (Primary — Plug & Play)

| OBD-II Pin | Function | Connect to |
|------------|----------|------------|
| Pin 6 | CAN High | ATOMIC CAN Base CAN-H |
| Pin 14 | CAN Low | ATOMIC CAN Base CAN-L |

Only 2 wires needed. Power via USB-C (car USB port or power bank).

### X179 Diagnostic Connector (Alternative)

Located in the rear center console area:
- 20-pin connector: Pin 13 (CAN-H), Pin 14 (CAN-L)
- 26-pin connector: Pin 18 (CAN-H), Pin 19 (CAN-L)

---

## CAN Bus Details

| CAN ID | Name | Purpose |
|--------|------|---------|
| `0x045` | STW_ACTN_RQ | Steering stalk (Legacy follow distance) |
| `0x318` | GTW_CAR_STATE | Vehicle state (OTA detection) |
| `0x331` | DAS_AP_CONFIG | DAS autopilot config (TLSSC restore target) |
| `0x370` | EPAS_STATUS | EPAS status (NAG killer target) |
| `0x398` | GTW_CAR_CONFIG | HW version detection |
| `0x399` | ISA_SPEED | Speed warning chime (HW4) |
| `0x3EE` | AP_LEGACY | Autopilot control (Legacy / HW1 / HW2) |
| `0x3F8` | FOLLOW_DIST | Follow distance / speed profile |
| `0x3FD` | AP_CONTROL | **Autopilot control (HW3/HW4) — core** |

Bus speed: **500 kbps**

---

## HW Support

| Tesla HW | Bits Modified | Speed Profile | Exclusive Features |
|----------|---------------|---------------|--------------------|
| Legacy (HW1/HW2) | bit46 | 3 levels (0-2) | — |
| HW3 | bit46 | 3 levels (0-2) + speed offset | — |
| HW4 (FSD V14+) | bit46 + bit60, bit47 | 5 levels (0-4) | ISA chime suppress, Emergency vehicle detect |

---

## Build & Flash

### Prerequisites
- [PlatformIO](https://platformio.org/) (CLI or VSCode extension)
- USB-C cable connected to M5Stack ATOM Lite

### Build
```bash
git clone https://github.com/hypery11/flipper-tesla-fsd.git
cd flipper-tesla-fsd/esp32
pio run -e m5stack-atom
```

### Flash
```bash
pio run -e m5stack-atom -t upload
```

### Monitor Serial Output
```bash
pio device monitor -b 115200
```

### Expected Boot Output
```
============================
 Tesla FSD Unlock — ESP32
============================
[FSD] Build: Apr 19 2026 12:00:00
[CAN] Driver: ESP32 TWAI (M5Stack ATOM Lite + ATOMIC CAN Base)
[NVS] Loaded: nag=1 chime=1 force=0 tlssc=0 auto_wake=0 emerg=0
[CAN] 500 kbps — Listen-Only
[BTN] Single click : toggle Listen-Only / Active
[BTN] Long press 3s: toggle NAG Killer
[LED] Blue=Listen  Green=Active  Yellow=OTA  Purple=Sleep  Red=Error
[WiFi] AP: "Tesla-FSD"  IP: 192.168.4.1
[WiFi] Dashboard: http://192.168.4.1
[Web] HTTP :80  WS :81 — ready
```

---

## Usage

### Physical Controls

1. Flash firmware to M5Stack ATOM Lite
2. Plug ATOMIC CAN Base onto ATOM Lite
3. Connect OBD-II cable: Pin 6 → CAN-H, Pin 14 → CAN-L
4. Plug into Tesla OBD-II port (under steering column, left side)
5. Power M5Stack via USB-C
6. Device starts in **Listen-Only mode** — verify CAN data on serial/dashboard
7. **Single click** button → switch to Active mode → FSD activates

| Button Action | Function |
|---------------|----------|
| Single click | Toggle Listen-Only ↔ Active mode |
| Long press (3s) | Toggle NAG Killer on/off |

### LED Status

| Color | State |
|-------|-------|
| 🔵 Blue | Listen-Only (passive monitoring) |
| 🟢 Green | Active (FSD enabled, transmitting) |
| 🟡 Yellow | OTA detected (TX suspended) |
| 🟣 Purple | Car asleep (no CAN traffic) |
| 🔴 Red | Error (no CAN signal after 5s) |

### WiFi Dashboard

1. Connect phone to WiFi: **Tesla-FSD** (password: **12345678**)
2. Open browser: **http://192.168.4.1**
3. Monitor and control everything from the tabbed web UI
4. REST API available at `http://192.168.4.1/api/status`

---

## Safety

- **OTA Protection** — automatically stops all CAN TX when a software update is *installing* (raw state = 2 on `0x318`); "update available" (state 1) does NOT suspend TX
- **Listen-Only default** — device will not modify any CAN frames until explicitly switched to Active mode
- **Sleep detection** — suspends TX when car goes to sleep (no CAN traffic for 3s), resumes on wake
- **Wiring diagnostics** — monitors rx_count and CRC error counter; red LED if no CAN traffic after 5s
- **DLC validation** — checks frame data length before parsing to prevent buffer overflows
- **Unplug = reset** — remove the device and restart the car to clear any modified state
- **WiFi isolated** — AP mode only, no internet connection, no data leaves the device

---

## Firmware Compatibility

This table is informational from field reports/upstream notes. The ESP32 code itself does not hardcode firmware-version checks.

| Tesla Firmware | HW3 | HW4 | Notes |
|----------------|-----|-----|-------|
| ≤ 2026.2.x | ✅ | ✅ | Full support |
| 2026.2.9.x | ✅ | ⚠️ | HW4 broken on these versions, use HW3 mode |
| 2026.8.3 | ✅ | ⚠️ | Not tested on HW4 |
| 2026.8.6+ | ⚠️ | ❌ | Region lock added — FSD model present but won't activate in some regions |

> **⚠️ Strongly recommended: disable automatic OTA updates** to stay on a compatible firmware version.

---

## Project Structure

```
esp32/
├── .firmware/
│   ├── main.cpp            — Init, button handling, CAN dispatch, main loop
│   ├── fsd_handler.cpp/h   — CAN protocol logic (ported from hypery11)
│   ├── can_driver.cpp/h    — CAN driver abstraction (TWAI / MCP2515)
│   ├── wifi_manager.cpp/h  — WiFi AP setup
│   ├── web_dashboard.cpp/h — HTTP server + WebSocket + embedded UI
│   ├── nvs_settings.cpp/h  — NVS persistence for toggle settings
│   ├── led.cpp/h           — NeoPixel LED status control
│   └── config.h            — CAN IDs, pin definitions, timing constants
├── platformio.ini          — Build configs (m5stack-atom / esp32-mcp2515)
└── README.md
```

---

## Credits

- **[hypery11/flipper-tesla-fsd](https://github.com/hypery11/flipper-tesla-fsd)** — Original Flipper Zero implementation. All CAN protocol logic ported from here.
- **[Tesla-OPEN-CAN-MOD](https://gitlab.com/Tesla-OPEN-CAN-MOD/tesla-open-can-mod)** — Original CAN signal research and documentation by Starmixcraft (Alex).
- **[wjsall/tesla-fsd-controller](https://github.com/wjsall/tesla-fsd-controller)** — ESP32 WiFi Web architecture reference.
- **[tuncasoftbildik/tesla-can-mod](https://github.com/tuncasoftbildik/tesla-can-mod)** — Tesla-inspired dark theme UI design reference.
- **[tesla-can-explorer](https://github.com/mikegapinski/tesla-can-explorer)** by @mikegapinski — CAN signal names and DBC definitions.

---

## License

GPL-3.0 — Same as the upstream projects.

---

## Disclaimer

> **⚠️ Tested on Model 3 2022 HW3 only. Other models/years/HW revisions are not yet validated.**

This project is for **educational and research purposes only**. Modifying vehicle CAN bus communication may:
- Void your vehicle warranty
- Violate local laws and regulations
- Cause unexpected vehicle behavior
- Create safety hazards

The authors are not responsible for any damage, legal consequences, or safety issues resulting from the use of this software. **Use entirely at your own risk.**
