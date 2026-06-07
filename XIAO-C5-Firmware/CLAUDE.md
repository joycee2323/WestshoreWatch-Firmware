# Westshore Watch Firmware — Cellular X1 Variant (XIAO ESP32-C5)

## Branch
**feature/cellular-x1-at-http** — native-AT-HTTP transport (current).
Branched off **feature/cellular-x1-ppp** (kept for reference). The PPP /
esp_modem path proved the wrong transport for this SIM7600G-H on this
network; uploads now use the modem's native AT HTTP(S) stack over UART.
None of these variant branches merge back to main — main continues to ship
the phone-paired X1 firmware.

## Project Overview
Standalone cellular drone detection node. Replaces the phone-paired
BLE-relay architecture with a SIM7600G-H 4G modem for direct HTTPS
upload to the backend. Built for Erik's personal deployment, not
customer hardware.

## Architecture
**Detection path:** WiFi promiscuous + BLE scan → raw_queue →
distributor → detect_queue → cellular_uploader → native AT HTTPS POST.

**Transport:** The modem stays in AT command mode for the whole session.
Bring-up runs AT+NETOPEN once + TLS config (sslversion=4, authmode=0,
enableSNI=1, ignorelocaltime=1 on SSL context 0). Each POST runs the
HTTPINIT → HTTPPARA → HTTPDATA → HTTPACTION → HTTPTERM sequence
(modem_http.c). GPS polling and HTTP POSTs share the one UART AT channel,
serialized by a recursive mutex in cellular_uart so no two AT transactions
interleave.

**Offline buffering:** When the cellular link is down, detections are
saved to a SPIFFS-backed ring buffer and drained on reconnect.

**GNSS:** Modem's built-in GNSS receiver provides node position,
attached to every detection upload as `node_position`.

## Hardware (XIAO ESP32-C5 + DFRobot SIM7600G-H TEL0162)

### Pin Assignments (XIAO header → ESP32-C5 GPIO)
| Header | GPIO | Function |
|--------|------|----------|
| D4 | GPIO23 | Modem UART TX (C5 → modem RX "R") |
| D5 | GPIO24 | Modem UART RX (modem TX "T" → C5) |
| D6 | GPIO11 | Modem PWRKEY (active-low pulse) |
| D7 | GPIO12 | Status LED red element |
| D8 | GPIO8 | Status LED green element |

**IMPORTANT:** XIAO Dx header labels ≠ GPIO numbers. D4 = GPIO23,
D5 = GPIO24, D6 = GPIO11, D7 = GPIO12, D8 = GPIO8. These are verified
against the official Seeed XIAO ESP32-C5 `pins_arduino.h` (symbol dump +
physical pin-toggle test) — NOT forum pinout tables. D4/D5 are the
board's default I2C SDA/SCL pads; UART works on them via the GPIO
matrix. The earlier "D4=GPIO6 / D5=GPIO7" was a wrong guess that left
the modem UART listening on an unconnected pin (zero RX bytes).

### Power
Battery → MH-CD42 → latching switch → 5V rail → C5 5V + modem VCC.
Always-on PPP mode (no PSM, no light sleep).

### Antennas
- C5: external u.FL (ALFA AOA-2458-59-TM via N-type bulkhead)
- Modem LTE: external u.FL (Bingfu via SMA bulkhead)
- Modem GNSS: external u.FL helix (Bingfu via SMA bulkhead)

## Status LED States
RED/YELLOW two-color LED — **there is no green element**. One element lit at a
time (driving both makes red dominate); blink rate distinguishes the
look-alike colors.

| State | Element | Meaning |
|-------|---------|---------|
| Off | — | System off / pre-init |
| Solid yellow | Yellow | Healthy: data session up, detections + heartbeats OK |
| Slow-blink yellow (~0.8 Hz) | Yellow | Warming up (boot/registering) or offline buffering |
| Fast-blink red (~3.3 Hz) | Red | Degraded: online + heartbeating but detection POSTs failing |
| Solid red | Red | Data session down / hardware fault |

**Wiring note:** GPIO8 (header D8) drives RED, GPIO12 (header D7) drives
YELLOW — verified on hardware. Common cathode, active HIGH. No green lead
exists; an earlier "green" assumption was wrong (commanding green lit the
yellow element). Red and yellow look similar, so the blink pattern — not the
hue — is what makes a fault unmistakable from healthy at a glance.

## Key Source Files (Cellular-Specific)
- `status_led.c/.h` — red/yellow LED driver (red=GPIO8, yellow=GPIO12;
  one element at a time + blink rate, no green)
- `cellular_uart.c/.h` — UART1 AT engine + recursive AT-channel mutex
  (lock/unlock, send_at, send_expect, write_raw, collect)
- `modem_manager.c/.h` — SIM7600 state machine: wake/SIM/registration →
  NETOPEN + TLS config → monitor loop (GPS poll + health checks)
- `modem_http.c/.h` — native SIM7600 AT HTTP(S) POST transaction
- `detection_queue.c/.h` — SPIFFS-backed offline ring buffer
- `cellular_uploader.c/.h` — batch + POST (via modem_http) with retry + drain
- `gnss_reader.c/.h` — GNSS position parsing from AT+CGPSINFO responses
- `modem_data_resume.cpp/.h` — PPP-only esp_modem shim; NOT compiled on this
  branch (CMakeLists omits it), kept for the PPP branch

### Existing Files (Unchanged from main)
- `ble_scanner.c/.h`, `odid_decoder.c/.h`, `ble_relay.c/.h`
- `wifi_scanner.c/.h`, `config_server.c/.h`, `nvs_config.c/.h`
- `output.c/.h`, `led.c/.h`, `dns_server.c/.h`, `ota_handler.c/.h`

### Modified from main (minimal diffs)
- `main.c` — cellular boot path (no output_task, no ble_relay_start)
- `CMakeLists.txt` — new source files + esp_modem/spiffs deps
- `sdkconfig.defaults` — PPP + SPIFFS config flags

## Modem State Machine
```
OFF → BOOTING → SIM_CHECK → NETWORK_SEARCH → REGISTERED →
NET_OPENING → ONLINE → (operational, monitor loop)
                          ↕
                        ERROR (auto-recovery: NETCLOSE + restart;
                               AT+CFUN=1,1 on AT-phase failure)
```
- 5 min stuck registering → AT+CFUN=1,1 reset, retry
- NETOPEN / registration lost in monitor loop → NETCLOSE + re-bring-up
- 10 min no HTTP response (any status) → full system reboot (main.c watchdog)

## NVS Configuration (namespace: "cell")
| Key | Default | Purpose |
|-----|---------|---------|
| device_id | cellular-x1-001 | Node ID for POST URL |
| api_key | (empty) | X-Node-API-Key header |
| cellular_apn | hologram | Carrier APN |
| backend_url | https://api.westshoredrone.com | Backend base URL |

## Backend Integration
- **Detections:** POST /api/nodes/:device_id/detections
  - Payload: `{drones: [{id, lat, lon, alt, spd, hdg, op_lat, op_lon}, ...],
    node_position: {lat, lon, alt_m, hdop}}`
  - Per-drone field names are CANONICAL and load-bearing: the backend keys on
    `drone.id` and does `if (!uas_id) continue;`, so it MUST be `id` (not
    `uas_id`) or the drone is silently dropped (200 / stored:0, never stored or
    broadcast). `alt`=geodetic alt, `spd`=horizontal speed, `hdg`=heading.
    Must match Android DetectionUploader.kt + routes/detections.js exactly.
- **Heartbeat:** POST /api/nodes/heartbeat (idle keep-alive, every 30 s)
  - Payload: `{connection_type:"cellular", firmware_version:"<ver>"}`
  - Matches the Sentinel's heartbeat. Backend marks the node `online` on
    ANY recent POST; the offline cron uses node_type defaults (x1 = 120 s),
    so an idle node needs the heartbeat or it flips offline between drones.
    30 s = 4× headroom, the same ratio the Sentinel runs.
- **Auth:** X-Node-API-Key header on both (NOT JWT), sent via the modem's
  AT+HTTPPARA "USERDATA"

## Build
```bash
idf.py set-target esp32c5
idf.py build
idf.py -p COMx flash monitor
```

## What NOT to Change
- BLE detection code (ble_scanner, ble_relay, odid_decoder)
- WiFi scanner / config portal
- NimBLE configuration
- OUI tables / drone model resolver
- Anything in the phone-paired data path (output.c, led.c)

These files stay identical to main to keep the diff clean and avoid
regressions if main-branch fixes need to be cherry-picked.
