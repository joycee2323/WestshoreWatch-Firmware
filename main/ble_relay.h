#pragma once
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "odid_decoder.h"

/**
 * BLE Wireless Relay
 *
 * Two independent non-connectable extended advertisers share the NimBLE host:
 *
 *   Handle 0 — ODID relay broadcast (legacy PDU, AD type 0x16, UUID 0xFFFA).
 *              Re-emits decoded Remote ID detections so any standard app
 *              (OpenDroneID OSM, DroneScanner, DroneScout pro) picks them up.
 *
 *   Handle 2 — AirAware detection advertiser (extended PDU, Manufacturer
 *              Specific Data, company ID 0x08FF). Broadcasts a compact JSON
 *              detection payload to the AirAware Android app. Advertising
 *              payload is refreshed each time ble_detection_advertise() is
 *              called with a new JSON blob.
 */

/**
 * Initialize NimBLE host, install sync/reset callbacks, and start the host
 * task. Idempotent.
 */
esp_err_t ble_relay_init(void);

/**
 * Start the relay task. Call after ble_relay_init(). Only used when the
 * device is in relay mode. If the host has not yet synced, task creation
 * is deferred to the sync callback.
 */
esp_err_t ble_relay_start(QueueHandle_t detect_queue);

/**
 * Push a fresh detection JSON blob out as a manufacturer-specific BLE
 * advertisement on handle 2 (company ID 0x08FF). Stops the current
 * advertisement if active, updates the AD payload, and restarts. No-op if
 * the NimBLE host has not yet synchronized or the handle has not been
 * configured. Safe to call from any task.
 */
void ble_detection_advertise(const char *json, size_t len);

void ble_relay_stop(void);
