# Westshore Watch Firmware — Cellular X1 Variant (XIAO ESP32-C5)

## Branch
**feature/cellular-x1** — permanent product variant branch. Does NOT
merge back to main. Main continues to ship the phone-paired X1 firmware.

## Project Overview
Standalone cellular drone detection node. Replaces the phone-paired
BLE-relay architecture with a SIM7600G-H 4G modem for direct HTTPS
upload to the backend. Built for Erik's personal deployment, not
customer hardware.

## Architecture
**Detection path:** WiFi promiscuous + BLE scan → raw_queue →
distributor → detect_queue → cellular_uploader → HTTPS POST via PPP.

**Offline buffering:** When PPP is down, detections are saved to a
SPIFFS-backed ring buffer and drained on reconnect.

**GNSS:** Modem's built-in GNSS receiver provides node position,
attached to every detection upload as `node_position`.

## Hardware (XIAO ESP32-C5 + DFRobot SIM7600G-H TEL0162)

### Pin Assignments (XIAO header → ESP32-C5 GPIO)
| Header | GPIO | Function |
|--------|------|----------|
| D4 | GPIO6 | Modem UART TX (C5 → modem RX) |
| D5 | GPIO7 | Modem UART RX (modem TX → C5) |
| D6 | GPIO11 | Modem PWRKEY (active-low pulse) |
| D7 | GPIO12 | Status LED red element |
| D8 | GPIO8 | Status LED green element |

**IMPORTANT:** XIAO Dx header labels ≠ GPIO numbers. D4 = GPIO6,
D5 = GPIO7, D6 = GPIO11, D7 = GPIO12, D8 = GPIO8.

### Power
Battery → MH-CD42 → latching switch → 5V rail → C5 5V + modem VCC.
Always-on PPP mode (no PSM, no light sleep).

### Antennas
- C5: external u.FL (ALFA AOA-2458-59-TM via N-type bulkhead)
- Modem LTE: external u.FL (Bingfu via SMA bulkhead)
- Modem GNSS: external u.FL helix (Bingfu via SMA bulkhead)

## Status LED States
| State | Color | Meaning |
|-------|-------|---------|
| Off | — | System off / pre-init |
| Solid green | Green | Healthy: PPP up, recent upload |
| Solid yellow | R+G | Warming up: boot / registering |
| Solid red | Red | Error: failed reg, HW fault |
| Blink yellow 1Hz | R+G | Offline, buffering detections |

## Key Source Files (Cellular-Specific)
- `status_led.c/.h` — bi-color LED driver (GPIO12/GPIO8)
- `cellular_uart.c/.h` — UART1 driver, AT command engine, PWRKEY control
- `modem_manager.c/.h` — SIM7600 state machine, PPP via esp_modem
- `detection_queue.c/.h` — SPIFFS-backed offline ring buffer
- `cellular_uploader.c/.h` — HTTPS POST with retry + offline drain
- `gnss_reader.c/.h` — GNSS position polling via AT commands

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
PPP_CONNECTING → PPP_CONNECTED → (operational)
                                   ↕
                                 ERROR (auto-recovery)
```
- 5 min stuck in any state → hard reset via PWRKEY
- 10 min no successful upload → full system reboot

## NVS Configuration (namespace: "cell")
| Key | Default | Purpose |
|-----|---------|---------|
| device_id | cellular-x1-001 | Node ID for POST URL |
| api_key | (empty) | X-Node-API-Key header |
| cellular_apn | hologram | Carrier APN |
| backend_url | https://api.westshoredrone.com | Backend base URL |

## Backend Integration
- **Endpoint:** POST /api/nodes/:device_id/detections
- **Auth:** X-Node-API-Key header (NOT JWT)
- **Payload:** `{drones: [...], node_position: {lat, lon, alt_m, hdop}}`

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
