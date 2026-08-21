#pragma once

#include "odid_decoder.h"
#include "esp_err.h"

/**
 * Best-effort UART telemetry emitter for an external status-screen board
 * (SparkFun Thing Plus C6), added alongside — never inside — the
 * detection/upload path.
 *
 * Fire-and-forget by design: TX-only on a dedicated UART, one small bounded
 * queue, drop-on-full. A stalled, disconnected, or absent display board has
 * zero effect on detection or cellular upload — this module never blocks a
 * caller and never touches the modem UART, the modem AT mutex, or any
 * detection/upload data structure.
 *
 * Entirely compiled out when WSD_DISPLAY_EMIT (config.h) is 0 — both
 * functions below become no-ops, so callers never need an #ifdef.
 */

/** Bring up the emitter UART + queue + background tasks. Safe to call once
 *  at boot; always returns ESP_OK (a no-op) when WSD_DISPLAY_EMIT is 0. */
esp_err_t display_emit_init(void);

/**
 * Copy only the fields needed for a "D," line out of *det into the emit
 * queue. Non-blocking: drops silently if the queue is already full. Intended
 * to be called from distributor_task as a tee, AFTER the existing
 * detect_queue passthrough — never gates it.
 */
void display_emit_submit_detection(const odid_detection_t *det);
