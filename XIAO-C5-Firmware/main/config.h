#pragma once

#include "sdkconfig.h"

/**
 * Westshore Drone Remote ID Receiver - Hardware Configuration (XIAO ESP32-C5)
 *
 * Target: Seeed Studio XIAO ESP32-C5 dev board.
 *   - User LED "L" on GPIO27 (active LOW: LOW=on, HIGH=off).
 *     Source: wiki.seeedstudio.com/xiao_esp32c5_getting_started — the
 *     board's LED_BUILTIN example uses digitalWrite(LOW) for "LED ON".
 *     Note: GPIO27 is a strapping pin on the ESP32-C5, but Seeed wires
 *     the onboard LED here and we only drive it after boot — safe.
 *   - Power LED: hardware-wired to 3V3, always on when powered, not
 *     firmware-controlled.
 *   - USB Serial/JTAG used for console (built-in peripheral, no GPIO).
 *   - No onboard user buttons, no external peripherals.
 *
 * UART primary TX/RX on XIAO header pins D6/D7, which map to
 * GPIO11/GPIO12 on this module (per the Seeed pin multiplex table).
 * That is NOT the same as the C5 silicon's UART0 IOMUX pins — the
 * XIAO module exposes a different set of headers than the bare-chip
 * WROOM-1-N8R8 footprint used by the custom-PCB C5-Firmware fork.
 *
 * Flash/PSRAM: 8MB + 8MB, same as WROOM-1-N8R8 — partition table
 * carries over from the baseline C5-Firmware.
 */

// ── Hardware revision identifier ───────────────────────────────────────────
// Surfaced in config portal status page and UART boot banner so the
// Android app can capture which board it's talking to.
#define HW_REVISION                 "xiao_esp32c5_v1"

// ── LED ─────────────────────────────────────────────────────────────────────
// XIAO ESP32-C5 user LED "L" on GPIO27, active LOW.
#define WSD_LED_GPIO                27
#define WSD_LED_ACTIVE_HIGH         0       // 0 = active LOW (LOW turns LED on)

// ── UART pin map ────────────────────────────────────────────────────────────
// XIAO ESP32-C5: header D6 = GPIO11 (UART TX), D7 = GPIO12 (UART RX)
// per Seeed's pin multiplex table.  Secondary UART on D8/D9 = GPIO8/9
// (unused, kept for parity with baseline).
#define WSD_UART_PRIMARY_TX         11
#define WSD_UART_PRIMARY_RX         12
#define WSD_UART_SECONDARY_TX       8
#define WSD_UART_SECONDARY_RX       9

// ── Primary UART (J2 / J3 main output, also USB-Serial via UART driver) ─────
#define WSD_UART_PRIMARY_NUM        UART_NUM_0
#define WSD_UART_PRIMARY_BAUD       115200
#define WSD_UART_BUF_SIZE           (4096)

// ── Secondary UART (J2 / J3 auxiliary port) ──────────────────────────────────
#define WSD_UART_SECONDARY_NUM      UART_NUM_1
#define WSD_UART_SECONDARY_BAUD     115200

// ── Wi-Fi scanner ─────────────────────────────────────────────────────────────
#define WSD_WIFI_CHANNEL_MIN        6       // 2.4 GHz channel range to scan
#define WSD_WIFI_CHANNEL_MAX        6
#define WSD_WIFI_DWELL_MS           500     // ms per channel before hopping

// ── BLE scanner ───────────────────────────────────────────────────────────────
#define WSD_BLE_SCAN_ITVL_MS        100     // scan interval (ms)
// 20% duty (was 100%). Running the legacy BLE scan at 100% duty alongside
// the Wi-Fi promiscuous scan starved the Wi-Fi RX path on the C5's shared
// radio (2026-05-11 side-by-side vs X1 reference: Sentinel lost every
// coalescer race). 20 ms window inside the 100 ms interval keeps BT4 RID
// coverage while leaving 80% of the on-air time for Wi-Fi RX. Re-evaluate
// if BLE detection rate drops noticeably in field deployment.
#define WSD_BLE_SCAN_WIN_MS         20      // scan window (ms)

// ── Output format ─────────────────────────────────────────────────────────────
#define WSD_OUTPUT_USB_CDC          1       // also echo to USB CDC/JTAG console
#define WSD_JSON_MAX_LEN            512

// ── Task priorities / stack sizes ─────────────────────────────────────────────
#define WSD_WIFI_SCAN_TASK_PRIO     5
#define WSD_WIFI_SCAN_STACK         4096
#define WSD_BLE_HOST_TASK_PRIO      4
#define WSD_BLE_HOST_STACK          6144
#define WSD_OUTPUT_TASK_PRIO        3
#define WSD_OUTPUT_STACK            4096

// ── Queue sizes ───────────────────────────────────────────────────────────────
#define WSD_DETECT_QUEUE_DEPTH      32
