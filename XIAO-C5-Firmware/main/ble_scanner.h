#pragma once
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "odid_decoder.h"

/**
 * Initialize and start the BLE Remote ID scanner.
 * Scans for both legacy (BT4) and extended (BT5) advertisements.
 * Detected events are posted to the provided output queue.
 *
 * @param output_queue  FreeRTOS queue that receives odid_detection_t items.
 * @return              ESP_OK on success.
 */
esp_err_t ble_scanner_start(QueueHandle_t output_queue);

/**
 * Stop the BLE scanner and free resources.
 */
void ble_scanner_stop(void);
