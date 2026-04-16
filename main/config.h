#pragma once

/**
 * Westshore Drone Remote ID Receiver - Hardware Configuration
 * PCB: Westshore Drone (ESP32-C5-WROOM-1-N8R8)
 *
 * ESP32-C5 notes:
 *   - GPIO range: GPIO0–GPIO28 (fewer pads than C6).
 *   - Strapping pins on C5: GPIO2, GPIO7, GPIO8, GPIO27, GPIO28
 *     (do not drive externally at boot without understanding defaults).
 *   - USB D-/D+ are GPIO13/GPIO14 on C5 (C6 used GPIO12/GPIO13).
 *   - MSPI flash/PSRAM pins (GPIO15–GPIO22) are RESERVED on C5:
 *       15=SPICS1(PSRAM), 16=SPICS0(flash), 17=SPIQ(flash MISO),
 *       18=SPIWP, 19=VDD_SPI, 20=SPIHD, 21=SPICLK, 22=SPID(flash MOSI).
 *     Touching any of these at runtime kills flash access → CPU_LOCKUP.
 *     This is a hard difference from C6, which kept flash on GPIO24–30.
 *   - Dual-band WiFi 6 (2.4 + 5 GHz); firmware locks to 2.4 GHz at
 *     runtime via esp_wifi_set_band_mode() in wifi_scanner.c.
 *
 * GPIO mapping (to be confirmed against final C5 PCB schematic):
 *
 *   GPIO4  - Status LED D6 (Orange, active HIGH, R8 470R series)
 *   GPIO5  - UART0 TX → J2/J3 primary JST connector (TxD.IN)
 *   GPIO6  - UART0 RX ← J2/J3 primary JST connector (RxD.IN)
 *   GPIO8  - Strapping / pull-up net (R1/R5 10k, do not drive)
 *   GPIO9  - Boot button SW2 (active LOW, internal pull-up)
 *   GPIO13 - USB D- (connected directly to USB-C, do not use as GPIO)
 *   GPIO14 - USB D+ (connected directly to USB-C, do not use as GPIO)
 *   EN     - Reset button SW1 (hardware reset)
 *
 * NOTE: UART0 was previously on GPIO16/17 (C6 PCB) — those pins are
 * reserved for flash on C5 so UART0 has been moved to GPIO5/6. UART1
 * is currently unused; if re-enabled, pick new pins (e.g. GPIO7/10) —
 * not GPIO5/6 which now belong to UART0.
 *
 *  J2/J3 JST SH 4-pin pinout (both identical):
 *    Pin 1: +3.3V
 *    Pin 2: TX (from ESP GPIO16 / GPIO5)
 *    Pin 3: RX (to ESP GPIO17 / GPIO6)
 *    Pin 4: GND
 */

// ── LED ─────────────────────────────────────────────────────────────────────
#define WSD_LED_GPIO                4       // Orange status LED
#define WSD_LED_ACTIVE_HIGH         1       // HIGH = LED on

// ── Buttons ──────────────────────────────────────────────────────────────────
#define WSD_BOOT_BTN_GPIO           9       // SW2 user/boot button (active LOW)

// ── Primary UART (J2 / J3 main output, also USB-Serial via UART driver) ─────
// Note: on C5 the original C6 pins (GPIO16/17) are MSPI flash pins — moved
// to GPIO5/6. UART0's direct-IOMUX pins on C5 are GPIO11/12, but those are
// already in use for the boot-time console and USB D-/D+ respectively.
#define WSD_UART_PRIMARY_NUM        UART_NUM_0
#define WSD_UART_PRIMARY_TX         5
#define WSD_UART_PRIMARY_RX         6
#define WSD_UART_PRIMARY_BAUD       115200
#define WSD_UART_BUF_SIZE           (4096)

// ── Secondary UART (J2 / J3 auxiliary port) ──────────────────────────────────
// Pins TBD for C5 — UART1 not currently initialized anywhere in firmware.
// If re-enabled, pick free GPIOs outside 15–22 (MSPI) and outside 13/14 (USB).
#define WSD_UART_SECONDARY_NUM      UART_NUM_1
#define WSD_UART_SECONDARY_TX       7
#define WSD_UART_SECONDARY_RX       10
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
