# Westshore Watch Firmware (XIAO ESP32-C5) — CLAUDE.md

## Project Overview
Remote ID receiver firmware targeting the **Seeed Studio XIAO ESP32-C5**
dev board. Fork of the custom-PCB `C5-Firmware`; no C6 support, no
custom-PCB support. Detects FAA Remote ID broadcasts (Wi-Fi Beacon and
BLE) and streams detection JSON over UART / USB CDC to a connected
Android phone running the Westshore Watch X1 app.

## Architecture
**Detection path:** WiFi promiscuous + BLE scan → raw_queue → distributor
→ output_queue (UART JSON + BLE re-advertise) + relay_queue (BLE relay).

**No autonomous WiFi uploading.** The node does NOT connect to the
internet or push data to the backend. All cloud sync is handled by the
Android app (WestshoreWatch-App repo) which consumes the UART JSON
stream and uploads to the backend.

## Hardware (XIAO ESP32-C5)
- **MCU:** ESP32-C5, dual-band Wi-Fi 6 + BT 5 LE, 8 MB flash + 8 MB PSRAM
- **UART JSON:** GPIO11/12 (header D6/D7), 115200 baud
- **User LED "L":** GPIO27, **active LOW** (LOW = on)
- **Power LED:** hardware-wired, not firmware-controlled
- **Console:** USB Serial/JTAG (built-in, no GPIO needed)
- **Antenna:** Seeed FPC Antenna A-01 (2.4/5 GHz combo, u.FL). The M1 is
  dual-band in both radio (XIAO ESP32-C5, internal band switch, no GPIO) and
  antenna, so it detects both 2.4 GHz drones and 5 GHz Skydio (Standard Remote
  ID via WiFi Beacon on U-NII-3, ch149/153). Note: the A-01 is silkscreened
  "2.4G" but is a 2.4/5 GHz combo part per Seeed — the silkscreen is a
  model-line label, not a band limit.
  Any future antenna swap MUST remain a 2.4/5 GHz dual-band u.FL part. A
  2.4-GHz-only antenna disables Skydio detection; a 5-GHz-only antenna disables
  2.4 GHz drone detection. Verify any new antenna against known Beacon sources
  on both ch6 (2.4) and ch149 (5) before field use.
- **IDF version:** ESP-IDF v5.5 or later

## Wi-Fi Scan Behavior (dual-band scout)
The channel hopper in `wifi_scanner.c` runs a continuous 2.4 GHz sweep and,
on the C5, interjects a short 5 GHz "peek" so a single time-shared radio
catches both bands:
- **2.4 GHz primary:** the existing linear sweep, unchanged.
- **5 GHz peek:** every `FIVE_GHZ_INTERVAL_MS` (default **5000**) the radio
  switches to 5 GHz for one `FIVE_GHZ_DWELL_MS` dwell (default **400**),
  alternating ch **149/153** (Skydio Standard RID, U-NII-3), then returns to
  the 2.4 GHz sweep where it left off. ~8% of on-air time on 5 GHz; catches a
  loitering Skydio within ~10–20 s. Both constants live at the top of
  `wifi_scanner.c`; `FIVE_GHZ_INTERVAL_MS` is the main lever.
- **Band switch** is the C5 internal switch (`esp_wifi_set_band_mode()` before
  `esp_wifi_set_channel()`) — no GPIO. `country_code("US")` + HT20 are set at
  init (US regulatory domain is required for ch149/153).
- **Portal suppression:** the peek is skipped while a config-portal client is
  connected (`s_paused`), so it can't drag the radio off the softAP's ch6 and
  drop the portal beacon.
- **Guard:** all 5 GHz code is behind `#if CONFIG_IDF_TARGET_ESP32C5`, kept
  identical to the custom-PCB X1 source so the two scanners stay diff-clean.

## LED Behavior
- Steady **ON** when idle (no RID packet in last ~50 ms)
- Flash **OFF** for ~50 ms on every RID packet detected (WiFi or BLE)
- Event-driven via `led_flash_detection()` — non-blocking from callbacks
- Legacy `led_set_pattern()` and `led_set_detecting()` kept as API
  compat stubs (no-op / one-shot pulse respectively)

## Key Source Files
- `main.c` — startup, queue creation, task launch
- `wifi_scanner.c` — 2.4GHz promiscuous mode, channel hopping, config AP
- `ble_relay.c` — BLE scan (BT4 legacy + BT5 extended/Coded PHY)
- `ble_scanner.c` — BLE advertiser-only relay, fires LED events on RID RX
- `odid_decoder.c` — Remote ID packet decoder
- `output.c` — UART JSON serializer; emits hw_rev boot banner
- `nvs_config.c` — NVS-backed config persistence
- `config_server.c` — HTTP config UI at 192.168.4.1; shows hw_rev
- `config.h` — all compile-time constants, `HW_REVISION` = `xiao_esp32c5_v1`
- `ota_handler.c` — OTA update support (two OTA slots)
- `led.c` — event-driven steady-on + flash-off LED controller

## Config Portal
- Soft-AP SSID: `WestshoreWatch-XXXX` (password: `westshore1`)
- HTTP UI: `http://192.168.4.1`
- Tabs: General / Reception / UART / System / Firmware
- **Always-on from boot** — no button gating (the custom PCB's GPIO9
  3-second-hold selector does not exist on this fork)
- System tab displays `HW revision: xiao_esp32c5_v1`

## Build
```bash
idf.py set-target esp32c5
idf.py build
idf.py -p COMx flash monitor
```

M1 Skydio (5 GHz) detection: firmware + antenna both dual-band ready.
Bench-confirm 5 GHz range vs the Alfa X1 before quoting coverage.

## Hardware Revision
Compile-time constant `HW_REVISION` (in `main/config.h`) = `"xiao_esp32c5_v1"`.
Surfaced in:
- UART boot banner (`hw_rev` JSON field, one-shot on Android app connect)
- Config portal System tab (read-only row)

## Intentionally Absent
These features live in the custom-PCB builds but are NOT in this fork:
- External status LED beyond the onboard user LED
- External buttons / GPIO9 boot-button mode selector
- Battery-voltage monitoring
- HTTPS uploader / `/api/detections` POST / node self-registration
- SNTP time sync
All backend sync is the Android app's responsibility.

## Important Notes
- Flash Encryption and Secure Boot intentionally disabled to preserve
  USB-C flashing
- Custom partition table (`partitions.esp32c5.csv`) required for two OTA
  slots — always use `erase-flash` when changing partition layout
- `build/` is gitignored — never commit compiled output
- Always provide complete file replacements, not partial diffs
