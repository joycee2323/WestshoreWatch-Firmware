#pragma once
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "odid_decoder.h"

/**
 * Start the output task.
 * Reads odid_detection_t items from the queue, formats them as
 * newline-delimited JSON, and writes to UART0 (and USB CDC).
 *
 * Output JSON schema (all fields always present, null/0 when unavailable):
 * {
 *   "ts":       <int>,        // FreeRTOS tick count at detection time
 *   "src":      <string>,     // "BT_LEGACY" | "BT5" | "WIFI_B" | "WIFI_N"
 *   "rssi":     <int>,        // dBm
 *   "mac":      <string>,     // "AA:BB:CC:DD:EE:FF"
 *   "id_type":  <int>,        // 0=none 1=serial 2=caa 3=utm 4=session
 *   "ua_type":  <int>,        // 0=none … 15=other
 *   "uas_id":   <string>,     // UAS/serial identifier string
 *   "status":   <int>,        // 0=undeclared 1=ground 2=airborne 3=emergency
 *   "lat":      <float>,      // degrees (0.0 if no location)
 *   "lon":      <float>,      // degrees
 *   "alt_baro": <float>,      // metres MSL
 *   "alt_geo":  <float>,      // metres WGS84
 *   "height":   <float>,      // metres AGL
 *   "speed":    <float>,      // m/s horizontal
 *   "vspeed":   <float>,      // m/s vertical (+ = up)
 *   "heading":  <int>,        // degrees true
 *   "description": <string>,  // Self-ID description (empty if not present)
 *   "op_lat":   <float>,      // Operator latitude
 *   "op_lon":   <float>,      // Operator longitude
 *   "op_alt":   <float>,      // Operator altitude (geo)
 *   "operator_id": <string>   // Operator identifier
 * }
 */
esp_err_t output_task_start(QueueHandle_t detect_queue);

/* Cache this node's own device_id (12-char uppercase hex, no separators,
 * derived from ESP_MAC_BT) once at boot. Call early from main() — before
 * ble_relay_start() — so it's ready for every consumer (the compact
 * detection JSON's "device_id" field, and the idle bridge-beacon UAS_ID)
 * regardless of FreeRTOS task scheduling order. Calling it again (e.g.
 * output_task()'s own startup) is harmless/idempotent. */
void output_cache_device_id(void);

/* This node's own device_id, as cached by output_cache_device_id(). Returns
 * a pointer to a static null-terminated buffer — empty string until
 * output_cache_device_id() has run at least once. */
const char *output_get_device_id_hex(void);
