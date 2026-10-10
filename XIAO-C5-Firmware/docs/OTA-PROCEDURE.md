# Cellular X1: firmware update procedure (portal OTA and USB)

Applies to the Cellular X1 (`XIAO-C5-Firmware/`, esp32c5) from 1.2.x on. A release folder
(`cellular-x1-ota-<version>/`) carries the files, their `SHA256SUMS` and that build's
version and ELF SHA256. This document doesn't repeat per-build hashes; always take them
from the release folder's `SHA256SUMS`.

There is **no remote OTA**: `esp_https_ota` is linked but never called. Every unit needs
someone on site, either on its Wi-Fi network (portal) or with a USB-C cable.

## 1. Check the file first (every time)
The device does **not** check which build it gets. `/ota` accepts any esp32c5 app image,
including the X1/M1 build, which has the same file name `westshore_remote_id.bin`, and older
versions. `esp_ota_end` checks only the image header, chip id and the image's own appended
SHA-256, and there is no signature check (secure boot is off). So the hash comparison below
is the only guard against flashing the wrong image.

- From the release folder, run `sha256sum -c SHA256SUMS` (Git Bash / WSL). Every line must
  say `OK`. Or:
- `Get-FileHash .\westshore_remote_id.bin -Algorithm SHA256` and compare **all 64
  characters** with the `westshore_remote_id.bin` line of `SHA256SUMS` (case doesn't matter).
  Don't compare just the first and last few characters.

Also note the node's current `firmware_version` (dashboard Nodes page, or
`nodes.firmware_version`), so you can tell when the new one is running.

## 2. Portal OTA (normal path)
1. Join the unit's soft-AP: SSID `WestshoreWatch-XXXX` (last 4 hex digits of its MAC),
   password `westshore1`. The portal is always on.
2. Open `http://192.168.4.1`. In the firmware section choose the release's
   `westshore_remote_id.bin` (the **app-only** image; never a merged/full-flash bin) and upload.
   It POSTs multipart to `/ota`.
3. Expect "Upload successful. The device is rebooting with the new firmware." The image is
   written to the **inactive** slot (ota_0 or ota_1, 3.5 MB each).
4. The unit reboots into the new slot in **pending-verify** state and marks itself valid
   **60 s** after boot (`OTA: image marked valid — rollback cancelled`).

Rollback:
- **Automatic** if the new image crashes or resets before the 60 s mark (rollback enabled,
  task watchdog panics).
- **After it marked itself valid**, re-upload the previous release's `westshore_remote_id.bin`
  the same way.

## 3. USB flash (bench, or recovery when the portal is unreachable)
Use the release's `usb-recovery/` set and the offsets in its `flasher_args.json`. Partition
layout (`partitions.esp32c5.csv`, 8 MB):

| Offset | Size | What | Written by the USB set? |
|---|---|---|---|
| 0x2000 | — | bootloader | yes |
| 0x8000 | — | partition table | yes |
| 0x9000 | 0x6000 | **nvs** (api_key, device_id, backend_url, cellular + portal settings) | **no** |
| 0xF000 | 0x1000 | phy_init | no |
| 0x10000 | 0x2000 | otadata (which slot boots) | yes (`ota_data_initial.bin` → boot ota_0) |
| 0x20000 | 0x380000 | ota_0 (app) | yes (`westshore_remote_id.bin`) |
| 0x3A0000 | 0x380000 | ota_1 (app) | no |
| 0x720000 | 0xE0000 | storage (SPIFFS detection spool) | no |

```powershell
python -m esptool --chip esp32c5 -p COMx -b 460800 --before default_reset --after hard_reset write_flash --flash_mode dio --flash_freq 80m --flash_size 8MB 0x2000 bootloader.bin 0x8000 partition-table.bin 0x10000 ota_data_initial.bin 0x20000 westshore_remote_id.bin
```

- `write_flash` writes only the listed regions, so **NVS is untouched** and the unit keeps its
  api_key and cellular settings. Never add `--erase-all`, and never run `erase_flash`: either
  wipes NVS.
- **The app goes at 0x20000, never 0x10000.** 0x10000 is otadata; an app written there corrupts
  boot selection. A wrong-offset `write_flash 0x10000` corrupted two Sentinel XIAOs in Sep 2026.
- `ota_data_initial.bin` resets boot to ota_0, which is where the USB set puts the app. Writing
  only the app to 0x20000 without it would leave a unit that last OTA'd into ota_1 booting the
  old image.

## 4. Check it worked
- **Serial banner** (USB): `App version: <release version>` and `ELF file SHA256:`. It must match
  the release README.
- **LED** (red/yellow, no green): solid yellow = healthy; slow-blink yellow = warming up /
  registering; fast-blink red = degraded (heartbeat OK, detections failing); solid red = fault.
- **Backend:**
  - `nodes.firmware_version` changes to the release version.
  - The firmware sends a heartbeat every 15 s, but only when no detection is waiting (1.2.3+).
  - `node_heartbeats` keeps at most one row per node per 30 s, shared with detection arrivals
    (backend e6731fc+), so rows appear about every 30 s.
- **Ghost node:** units on 1.2.1 or later already include 5ae36fc, so "X1 - ABS" should stay
  dark before and after.
- Wait **at least 5 min** with no reboot loop.
