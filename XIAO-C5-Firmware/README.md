# Westshore Watch Firmware — Seeed XIAO ESP32-C5

Remote ID receiver firmware targeting the **Seeed Studio XIAO ESP32-C5**
dev board. Fork of the custom-PCB `C5-Firmware`; no support for the
ESP32-C6 or the custom Westshore PCB in this tree.

Detects FAA Remote ID broadcasts (Wi-Fi beacon/action frames + BLE legacy
advertisements), streams detection JSON over USB CDC to a connected
Android phone running the Westshore Watch X1 app, and re-advertises over
BLE for the Android app's BLE relay. No autonomous internet uplink — all
cloud sync is the Android app's job.

## Hardware

- **Board**: Seeed Studio XIAO ESP32-C5 (dual-band Wi-Fi 6 + BT 5 LE,
  8 MB flash, 8 MB PSRAM).
- **User LED** "L": **GPIO27, active LOW** (LOW = on, HIGH = off). Per
  Seeed's pin mux table; confirmed against the Arduino LED\_BUILTIN
  example in the getting-started wiki.
- **Power LED**: hard-wired to 3V3, always lit when powered — not
  firmware-controlled.
- **Primary UART** (for JSON stream to the Android phone): header pins
  **D6 / D7 = GPIO11 / GPIO12**, 115200 8N1. Shared with USB Serial/JTAG
  CDC; Android app can read either.
## Antenna (IMPORTANT)

The small flat antenna that ships with the Seeed XIAO ESP32-C5 is
inadequate for reliable operation in this firmware. Field testing
showed the soft-AP beacons dropped below phone detection threshold
intermittently, making the config portal unreachable during normal
scanner operation.

**A better 2.4 GHz U.FL antenna is REQUIRED for field deployment.**
Verified working: any standard 2.4 GHz dipole or whip antenna with
U.FL (IPEX1) connector and >=2 dBi gain. Swap is hot-pluggable — no
firmware changes needed.

For optional 5 GHz WiFi NAN detection (most Remote ID traffic is BLE
or 2.4 GHz NAN on channel 6 regardless), a dual-band U.FL antenna
may be sourced separately.

## Behavior

- **Config portal**: always-on soft-AP from boot, no button gating.
  Connect to `WestshoreWatch-XXXX` (password: `westshore1`, where XXXX
  is the last two bytes of the device's AP MAC), then browse to
  `http://192.168.4.1`. Captive-portal DNS hijacks any lookup so phones
  auto-open the UI.
- **User LED**:
  - Steady **ON** when idle (no detection packet in the last ~50 ms).
  - Flashes **OFF** for ~50 ms on every Remote ID packet received
    (Wi-Fi *or* BLE). Visible flicker when drone traffic is present;
    solid glow otherwise.
- **UART boot banner**: first line emitted over UART0 is a JSON
  identity frame including `hw_rev` so the Android app can pin which
  board it's talking to:

  ```json
  {"info":"Westshore Drone Remote ID Receiver ready","fw":"1.2-westshore","hw_rev":"xiao_esp32c5_v1"}
  ```

- **Config portal System tab**: shows `HW revision: xiao_esp32c5_v1`
  read-only alongside firmware version, MAC, and IDF version.

## Build & flash

ESP-IDF **v5.5 or later** is required (XIAO ESP32-C5 support needs the
C5 target, which landed in v5.5).

```bash
idf.py set-target esp32c5
idf.py build
idf.py -p COMx flash monitor    # replace COMx with the XIAO's port
```

Switching into this folder from a different target requires `idf.py
fullclean` first (the `build/` directory is target-specific and is
gitignored anyway).

## Pin map summary

| Header | GPIO   | Use                                              |
|--------|--------|--------------------------------------------------|
| —      | 27     | User LED "L" (active LOW)                        |
| D6     | 11     | UART0 TX (JSON stream to Android app)            |
| D7     | 12     | UART0 RX                                         |
| D8     | 8      | UART1 TX (unused, reserved for parity)           |
| D9     | 9      | UART1 RX (unused, reserved for parity)           |
| USB-C  | —      | USB Serial/JTAG console (built-in, no GPIO)      |

No user buttons, no external peripherals, no battery monitoring on this
build — everything the custom PCB had beyond the LED and UART is
intentionally absent here.

## What this firmware does NOT do

- **No HTTPS uploader** to `/api/detections`. No backend HTTP client.
- **No node self-registration** endpoint.
- **No SNTP / time sync**.
- **No battery-voltage ADC**.
- **No external buttons** (no GPIO9 boot-button mode selector — the
  custom PCB had one, this fork does not).

All backend sync is handled by the Android app over the UART JSON
stream. See `CLAUDE.md` for architectural rationale.

## Hardware revision identifier

Compile-time constant `HW_REVISION` in `main/config.h`:

```c
#define HW_REVISION "xiao_esp32c5_v1"
```

Surfaced in:
- UART boot banner (`hw_rev` field, one-shot on connect).
- Config portal **System** tab (read-only row).
