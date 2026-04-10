# AirAware Firmware — CLAUDE.md

## Project Overview
ESP32-C6 firmware for the AirAware X1 Remote ID receiver node by Westshore Drone Services. Detects FAA Remote ID broadcasts (WiFi Beacon and BLE) and streams detection JSON over UART to a connected Android phone running the AirAware X1 app.

## Architecture
**Detection path:** WiFi promiscuous + BLE scan → raw_queue → distributor → output_queue (UART JSON) + relay_queue (BLE relay)

**No autonomous WiFi uploading.** The node does NOT connect to the internet or push data to the backend. All cloud sync is handled by the Android app (AirAware-App repo) which consumes the UART JSON stream and uploads to the backend.

## Hardware
- **MCU:** ESP32-C6-WROOM-1 (internal antenna)
- **UART JSON:** GPIO16/17, 115200 baud
- **Status LED:** IO8 (orange)
- **Config portal button:** GPIO9 (3-second hold post-boot)
- **IDF version:** ESP-IDF v5.5.3

## Known Nodes
- **COM5:** MAC `98:A3:16:7D:26:61`, api_key `99c169eb...` — stable baseline
- **COM7:** MAC `98:A3:16:7D:26:34`, api_key `4d94e21e...` — secondary node

## Key Source Files
- `main.c` — startup, queue creation, task launch
- `wifi_scanner.c` — 2.4GHz promiscuous mode, channel hopping, config AP
- `ble_relay.c` — BLE scan (BT4 legacy + BT5 extended/Coded PHY), Holystone module detection
- `ble_scanner.c` — BLE advertiser-only relay
- `odid_decoder.c` — Remote ID packet decoder
- `output.c` — UART JSON serializer
- `nvs_config.c` — NVS-backed config persistence
- `config_server.c` — HTTP config UI at 192.168.4.1
- `config.h` — all compile-time constants and defaults
- `ota_handler.c` — OTA update support (two 1.75MB slots)

## Detection Logic
- MAC accumulator for drone deduplication
- OUI filter to suppress relay loop false positives
- BLE broadcast module detection via `ble_gap_ext_disc()` for BT4 legacy and BT5 extended/Coded PHY
- Holystone module: MAC `18:65:6A:00:29:05`, UUID `0xFFFA`, BT5 Coded + BT4 legacy

## Config Portal
- Soft-AP SSID: `AirAware-X1-XXXX` (password: `airaware1`)
- HTTP UI: `http://192.168.4.1`
- Tabs: General / Reception / UART / System / Firmware
- GPIO9 boot-button: 3-second hold post-boot = config mode
- Note: Config portal WiFi WPA handshake failures in congested RF environments — known issue, deferred

## Build
```bash
idf.py build
idf.py -p COM5 flash monitor
```

## Important Notes
- Flash Encryption and Secure Boot intentionally disabled to preserve USB-C flashing
- Custom partition table (`partitions.csv`) required for two OTA slots — always use `erase-flash` when changing partition layout
- `build/` is gitignored — never commit compiled output
- Always provide complete file replacements, not partial diffs
