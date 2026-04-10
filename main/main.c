#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"

#include "config.h"
#include "nvs_config.h"
#include "odid_decoder.h"
#include "wifi_scanner.h"
#include "output.h"
#include "led.h"
#include "ble_relay.h"

static const char *TAG = "MAIN";

static QueueHandle_t raw_queue    = NULL;
static QueueHandle_t output_queue = NULL;
static QueueHandle_t relay_queue  = NULL;

/* Distribute one detection to all downstream queues */
static void distributor_task(void *arg)
{
    odid_detection_t det;
    while (true) {
        if (xQueueReceive(raw_queue, &det, portMAX_DELAY) == pdTRUE) {
            xQueueSend(output_queue, &det, 0);
            xQueueSend(relay_queue,  &det, 0);
        }
    }
}

void app_main(void)
{
    /* NVS init */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    led_init();
    led_set_pattern(LED_PATTERN_BOOT);

    /* Load config — falls back to defaults on first boot */
    wsd_config_load(&g_config);

    /* ── NORMAL OPERATION MODE ──────────────────────────────────────────────
     *
     * Config portal is always-on as a background soft-AP.
     * Connect to AirAware-X1-XXXX (password: airaware1) → http://192.168.4.1
     *
     * ────────────────────────────────────────────────────────────────────── */
    ESP_LOGI(TAG, "===========================================");
    ESP_LOGI(TAG, " AirAware X1 — Remote ID Sensor Node v1.1");
    ESP_LOGI(TAG, " ESP32-C6  |  IDF %s", esp_get_idf_version());
    ESP_LOGI(TAG, "===========================================");
    ESP_LOGI(TAG, " Mode:     %s",
             g_config.mode == WSD_MODE_RELAY ? "BLE relay" : "UART only");
    ESP_LOGI(TAG, " Channels: %d-%d",
             g_config.ch_2g_start, g_config.ch_2g_stop);
    ESP_LOGI(TAG, " Upload:   %s  SSID: '%s'",
             g_config.upload_en ? "enabled" : "disabled",
             g_config.wifi_ssid[0] ? g_config.wifi_ssid : "(not set)");
    ESP_LOGI(TAG, " Node:     %s",
             g_config.node_name[0] ? g_config.node_name : "(unnamed)");

    /* Create detection queues */
    raw_queue    = xQueueCreate(WSD_DETECT_QUEUE_DEPTH, sizeof(odid_detection_t));
    output_queue = xQueueCreate(WSD_DETECT_QUEUE_DEPTH, sizeof(odid_detection_t));
    relay_queue  = xQueueCreate(WSD_DETECT_QUEUE_DEPTH, sizeof(odid_detection_t));

    if (!raw_queue || !output_queue || !relay_queue) {
        ESP_LOGE(TAG, "Queue creation failed — halting");
        led_set_pattern(LED_PATTERN_ERROR);
        while (true) vTaskDelay(pdMS_TO_TICKS(1000));
    }

    /* Distributor: fan-out detections to all consumers */
    xTaskCreate(distributor_task, "distributor", 2048, NULL,
                WSD_OUTPUT_TASK_PRIO + 2, NULL);

    /* UART JSON output — always on */
    err = output_task_start(output_queue);
    if (err != ESP_OK)
        ESP_LOGW(TAG, "Output task failed: %d", err);

    /* WiFi scanner — also starts the always-on config AP and HTTP server */
    err = wifi_scanner_start(raw_queue);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Wi-Fi scanner failed: %d — halting", err);
        led_set_pattern(LED_PATTERN_ERROR);
        while (true) vTaskDelay(pdMS_TO_TICKS(1000));
    }

    vTaskDelay(pdMS_TO_TICKS(500));

    /* BLE relay — always on (independent of WiFi upload) */
    if (g_config.mode == WSD_MODE_RELAY) {
        err = ble_relay_start(relay_queue);
        if (err != ESP_OK)
            ESP_LOGW(TAG, "BLE relay failed: %d", err);
    }

    led_set_pattern(LED_PATTERN_SCANNING);

    ESP_LOGI(TAG, "System ready");
    ESP_LOGI(TAG, "  Wi-Fi scanner:  ch %d-%d, %dms dwell",
             g_config.ch_2g_start, g_config.ch_2g_stop, WSD_WIFI_DWELL_MS);
    ESP_LOGI(TAG, "  BLE relay:      %s",
             g_config.mode == WSD_MODE_RELAY ? "active" : "disabled");
    ESP_LOGI(TAG, "  Config portal:  AirAware-X1-XXXX → http://192.168.4.1");

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
