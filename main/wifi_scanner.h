#pragma once
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

/**
 * Start the WiFi scanner in APSTA mode.
 *
 * Initialises the WiFi driver, starts the config soft-AP (always-on),
 * enables promiscuous mode for ODID detection, and begins channel hopping.
 *
 * The soft-AP SSID is built at runtime: "AirAware-X1-XXXX" where XXXX
 * is the last 4 hex digits of the device MAC address.
 */
esp_err_t wifi_scanner_start(QueueHandle_t output_queue);

/**
 * Stop the scanner (promiscuous + channel hopping only).
 * The config AP stays up. Call before a WiFi upload window.
 */
void wifi_scanner_pause(void);

/**
 * Resume promiscuous scanning and channel hopping after a pause.
 */
void wifi_scanner_resume(void);

/**
 * Fully stop the scanner and deinit WiFi (call on shutdown only).
 */
void wifi_scanner_stop(void);
