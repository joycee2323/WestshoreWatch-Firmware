#pragma once

#include "esp_err.h"
#include "odid_decoder.h"

/**
 * SPIFFS-backed detection ring buffer.
 *
 * Holds up to DETECTION_QUEUE_CAPACITY detections in a circular file.
 * Survives reboots — on boot, any buffered detections are available for
 * drain.  FIFO with oldest-drop when full.
 *
 * Thread-safe: a FreeRTOS mutex guards all file operations.
 */

#define DETECTION_QUEUE_CAPACITY  256

/** Mount SPIFFS partition and open (or create) the ring buffer file. */
esp_err_t detection_queue_init(void);

/** Enqueue a detection.  Drops oldest if full. */
esp_err_t detection_queue_push(const odid_detection_t *det);

/** Dequeue the oldest detection.  Returns ESP_ERR_NOT_FOUND if empty. */
esp_err_t detection_queue_pop(odid_detection_t *det);

/** Dequeue the NEWEST detection (LIFO end).  Returns ESP_ERR_NOT_FOUND if
 *  empty.  Same on-disk format as pop(); used to replay newest first. */
esp_err_t detection_queue_pop_newest(odid_detection_t *det);

/** Number of detections currently buffered. */
int detection_queue_count(void);
