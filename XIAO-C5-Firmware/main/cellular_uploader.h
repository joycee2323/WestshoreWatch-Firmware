#pragma once

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "odid_decoder.h"

/**
 * HTTPS detection uploader over the cellular link.
 *
 * Reads detections from the in-memory FreeRTOS queue, batches them, and POSTs
 * to the backend using the modem's native AT HTTP(S) stack (modem_http) — no
 * PPP.  When the cellular link is down, pushes overflow into the SPIFFS-backed
 * detection_queue and drains it when the link returns.
 *
 * Endpoint: POST /api/nodes/:device_id/detections
 * Auth:     X-Node-API-Key header (sent via AT+HTTPPARA "USERDATA")
 */

/**
 * Create the uploader's RAM queue (items are upload_frame_t: a detection plus
 * its receive time). Call once, before the distributor task runs.
 * @return the queue, or NULL on allocation failure
 */
QueueHandle_t cellular_uploader_create_queue(int depth);

/**
 * Hand one detection to the uploader (distributor task only — not
 * reentrant). Stamps the receive time; if the queue is full the OLDEST queued
 * frame is dropped so the newest one is never the one lost.
 */
void cellular_uploader_submit(const odid_detection_t *det);

/**
 * Start the uploader task.
 * @param detect_queue  the queue returned by cellular_uploader_create_queue()
 */
esp_err_t cellular_uploader_start(QueueHandle_t detect_queue);

/** Timestamp (ticks) of last successful upload (2xx), or 0 if never. */
TickType_t cellular_uploader_last_success(void);

/** Timestamp (ticks) of last HTTP response (any status), or 0 if never.
 *  Used by upload watchdog: reboot only when network is totally dead
 *  (no response at all), not when backend returns 4xx/5xx. */
TickType_t cellular_uploader_last_response(void);

/** Timestamp (ticks) of the last detection-batch POST ATTEMPT, or 0 if never.
 *  Detection-specific (NOT refreshed by heartbeats). */
TickType_t cellular_uploader_last_det_attempt(void);

/** Timestamp (ticks) of the last SUCCESSFUL (2xx) detection-batch POST, or 0.
 *  With last_det_attempt, lets the LED flag "online but detections failing":
 *  recently attempted yet not recently succeeded → degraded. */
TickType_t cellular_uploader_last_det_success(void);
