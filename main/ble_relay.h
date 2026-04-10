#pragma once
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "odid_decoder.h"

/**
 * BLE Wireless Relay
 *
 * Receives decoded Remote ID detections and re-broadcasts them as standard
 * BLE legacy advertisements (AD type 0x16, UUID 0xFFFA) at low power.
 *
 * Any standard Remote ID app (OpenDroneID OSM, DroneScanner, DroneScout pro)
 * will see these as normal drone Remote ID signals — no pairing or setup needed.
 *
 * Transmit power: minimum (-24 dBm), effective range ~25 metres.
 * Advertising interval: ~200ms per message.
 */
esp_err_t ble_relay_start(QueueHandle_t detect_queue);
void      ble_relay_stop(void);
