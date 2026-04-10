#pragma once
#include "esp_err.h"

/* ─────────────────────────────────────────────────────────────────────────────
 * Westshore Remote ID — Wi-Fi AP configuration server
 *
 * wifi_scanner_start() owns WiFi init and starts the AP.
 * config_server_start_http() registers the HTTP handlers on top of that.
 *
 * The SSID is built at runtime by wifi_scanner: "AirAware-X1-XXXX"
 * ───────────────────────────────────────────────────────────────────────────── */

/* AP password — set to "" for open network, or set a password */
#define CFG_AP_PASS     "airaware1"

/* AP IP address (default gateway assigned to connecting clients) */
#define CFG_AP_IP       "192.168.4.1"

/**
 * Start the HTTP config server.
 * Call AFTER wifi_scanner_start() — WiFi must already be running.
 * Non-blocking: HTTP server runs in its own tasks.
 */
esp_err_t config_server_start_http(void);
