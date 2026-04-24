#pragma once

#include "sdkconfig.h"

/**
 * Westshore Drone Remote ID Receiver - Hardware Configuration
 *
 * This file is target-aware. Pin assignments differ between the
 * ESP32-C6 and ESP32-C5 PCB revisions; pick the right set via
 * #if CONFIG_IDF_TARGET_ESP32C5 / #elif CONFIG_IDF_TARGET_ESP32C6.
 *
 * ── ESP32-C6 board (Westshore Drone v1.0, ESP32-C6-WROOM-1U) ───────────────
 *   GPIO mapping (KiCad: Erik_J_BlueMark_DronescoutBridge_v1.0):
 *     GPIO4  - Status LED D6 (orange, active HIGH, R8 470R series)
 *     GPIO5  - UART1 TX  (secondary JST, TxD.OUT)
 *     GPIO6  - UART1 RX  (secondary JST, RxD.OUT)
 *     GPIO8  - Strapping / pull-up net (do not drive)
 *     GPIO9  - Boot button SW2 (active LOW, internal pull-up)
 *     GPIO12 - USB D-  (do not use as GPIO)
 *     GPIO13 - USB D+  (do not use as GPIO)
 *     GPIO16 - UART0 TX  (primary JST, TxD.IN)
 *     GPIO17 - UART0 RX  (primary JST, RxD.IN)
 *
 * ── ESP32-C5 board (Westshore Drone, ESP32-C5-WROOM-1-N8R8) ────────────────
 *   GPIO range GPIO0–GPIO28. Strapping pins: GPIO2, 7, 8, 27, 28.
 *   USB D-/D+ moved to GPIO13/GPIO14 (was 12/13 on C6).
 *   MSPI flash pins are GPIO15–GPIO22 — RESERVED, touching them at
 *   runtime kills flash access (CPU lockup). UART0 therefore moves
 *   off GPIO16/17 onto GPIO5/6, and UART1 moves to GPIO7/10.
 *   Dual-band Wi-Fi 6 (2.4 + 5 GHz); firmware locks to 2.4 GHz at
 *   runtime via esp_wifi_set_band_mode() in wifi_scanner.c.
 *
 *     GPIO4  - Status LED D6 (orange, active HIGH, R8 470R series)
 *     GPIO5  - UART0 TX  (primary JST, TxD.IN)
 *     GPIO6  - UART0 RX  (primary JST, RxD.IN)
 *     GPIO7  - UART1 TX  (secondary JST, currently unused)
 *     GPIO9  - Boot button SW2 (active LOW, internal pull-up)
 *     GPIO10 - UART1 RX  (secondary JST, currently unused)
 *     GPIO13 - USB D-  (do not use as GPIO)
 *     GPIO14 - USB D+  (do not use as GPIO)
 *
 *  J2/J3 JST SH 4-pin pinout (both identical):
 *    Pin 1: +3.3V
 *    Pin 2: TX (from ESP UART0 primary TX)
 *    Pin 3: RX (to   ESP UART0 primary RX)
 *    Pin 4: GND
 */

// ── LED ─────────────────────────────────────────────────────────────────────
#define WSD_LED_GPIO                4       // Orange status LED
#define WSD_LED_ACTIVE_HIGH         1       // HIGH = LED on

// ── Buttons ──────────────────────────────────────────────────────────────────
#define WSD_BOOT_BTN_GPIO           9       // SW2 user/boot button (active LOW)

// ── UART pin map (target-specific) ───────────────────────────────────────────
#if CONFIG_IDF_TARGET_ESP32C5
    // C5: UART0 on GPIO5/6 (GPIO16/17 are MSPI flash pins, reserved).
    #define WSD_UART_PRIMARY_TX     5
    #define WSD_UART_PRIMARY_RX     6
    // C5: UART1 on GPIO7/10 (currently unused; pins TBD against final PCB).
    #define WSD_UART_SECONDARY_TX   7
    #define WSD_UART_SECONDARY_RX   10
#elif CONFIG_IDF_TARGET_ESP32C6
    // C6: UART0 on the original GPIO16/17 IOMUX pins.
    #define WSD_UART_PRIMARY_TX     16
    #define WSD_UART_PRIMARY_RX     17
    // C6: UART1 on GPIO5/6.
    #define WSD_UART_SECONDARY_TX   5
    #define WSD_UART_SECONDARY_RX   6
#else
    #error "Unsupported IDF target — add a UART pin map for this chip in main/config.h"
#endif

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
#define WSD_BLE_SCAN_WIN_MS         100     // scan window  (100% duty = passive)

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
