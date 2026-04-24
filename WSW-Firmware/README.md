# Westshore Watch Firmware — ESP32-C5 / ESP32-C6

Firmware for the Westshore Watch X1 Remote ID receiver node by Westshore Drone Services.
A single source tree builds for two PCB revisions — pick the target before
the first build.

## Targets

| Target    | Module                  | Flash | Defaults file                  | Partition table          |
|-----------|-------------------------|-------|--------------------------------|--------------------------|
| `esp32c5` | ESP32-C5-WROOM-1-N8R8   | 8 MB  | `sdkconfig.defaults.esp32c5`   | `partitions.esp32c5.csv` |
| `esp32c6` | ESP32-C6-WROOM-1U-N4    | 4 MB  | `sdkconfig.defaults.esp32c6`   | `partitions.esp32c6.csv` |

ESP-IDF v5.5.3 or later is required (C5 production support landed in v5.5).

## Build & flash

```bash
# First-time setup for a chip — IDF picks up sdkconfig.defaults.<target>
idf.py set-target esp32c5     # or esp32c6
idf.py build

# Flash + monitor
idf.py -p COM5 flash monitor  # COM port varies per node
```

`idf.py set-target` regenerates the local `sdkconfig` from
`sdkconfig.defaults.<target>` — that file is gitignored and per-machine.
Switching targets always requires a fresh `set-target` (and an
`idf.py fullclean` is safest, since the build directory is target-specific).

Pin assignments live in `main/config.h` and are wrapped in
`#if CONFIG_IDF_TARGET_ESP32C5 / #elif CONFIG_IDF_TARGET_ESP32C6` blocks.
Adding a new chip requires a new defaults file, a new partition table,
and a new branch in that pin map.

See `CLAUDE.md` for project architecture, hardware notes, and known nodes.
