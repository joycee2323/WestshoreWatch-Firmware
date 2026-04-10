#pragma once

/**
 * Westshore Drone Remote ID Receiver - Hardware Configuration
 * PCB: Westshore Drone v1.0 (ESP32-C6-WROOM-1U)
 *
 * GPIO mapping derived from KiCad schematic (Erik_J_BlueMark_DronescoutBridge_v1.0):
 *
 *   GPIO4  - Status LED D6 (Orange, active HIGH, R8 470R series)
 *   GPIO5  - UART1 TX → J2/J3 secondary JST connector (TxD.OUT)
 *   GPIO6  - UART1 RX ← J2/J3 secondary JST connector (RxD.OUT)
 *   GPIO8  - Strapping / pull-up net (R1/R5 10k, do not drive)
 *   GPIO9  - Boot button SW2 (active LOW, internal pull-up)
 *   GPIO12 - USB D- (IO12/D-, connected directly to USB-C, do not use as GPIO)
 *   GPIO13 - USB D+ (IO13/D+, connected directly to USB-C, do not use as GPIO)
 *   GPIO16 - UART0 TX → J2/J3 primary JST connector (TxD.IN)
 *   GPIO17 - UART0 RX ← J2/J3 primary JST connector (RxD.IN)
 *   EN     - Reset button SW1 (hardware reset)
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
#define WSD_UART_PRIMARY_NUM        UART_NUM_0
#define WSD_UART_PRIMARY_TX         16
#define WSD_UART_PRIMARY_RX         17
#define WSD_UART_PRIMARY_BAUD       115200
#define WSD_UART_BUF_SIZE           (4096)

// ── Secondary UART (J2 / J3 auxiliary port) ──────────────────────────────────
#define WSD_UART_SECONDARY_NUM      UART_NUM_1
#define WSD_UART_SECONDARY_TX       5
#define WSD_UART_SECONDARY_RX       6
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
