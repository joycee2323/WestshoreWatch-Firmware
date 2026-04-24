#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_app_desc.h"
#include "esp_ota_ops.h"
#include "esp_timer.h"
#include "nvs_flash.h"

#include "config.h"
#include "nvs_config.h"
#include "odid_decoder.h"
#include "wifi_scanner.h"
#include "output.h"
#include "led.h"
#include "ble_relay.h"

static const char *TAG = "MAIN";

/* Seconds of healthy runtime before we tell the bootloader this image is good.
 * If we crash before this fires, otadata stays in pending-verify and the
 * bootloader rolls back to the previous slot on next boot. */
#define WSD_OTA_VALIDATE_DELAY_S 60

static void ota_mark_valid_cb(void *arg)
{
    (void)arg;
    esp_err_t err = esp_ota_mark_app_valid_cancel_rollback();
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "OTA: image marked valid — rollback cancelled");
    } else {
        ESP_LOGW(TAG, "OTA: mark-valid returned 0x%x (%s) — "
                 "likely not a pending-verify image, safe to ignore",
                 err, esp_err_to_name(err));
    }
}

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

    ESP_LOGI(TAG, "boot: calling led_init");
    esp_err_t led_err = led_init();
    if (led_err != ESP_OK) {
        ESP_LOGE(TAG, "boot: led_init returned 0x%x (%s)",
                 led_err, esp_err_to_name(led_err));
    } else {
        ESP_LOGI(TAG, "boot: led_init returned OK");
    }
    led_set_pattern(LED_PATTERN_BOOT);

    /* Load config — falls back to defaults on first boot */
    wsd_config_load(&g_config);

    /* ── NORMAL OPERATION MODE ──────────────────────────────────────────────
     *
     * Config portal is always-on as a background soft-AP.
     * Connect to WestshoreWatch-XXXX (password: westshore1) → http://192.168.4.1
     *
     * ────────────────────────────────────────────────────────────────────── */
    const esp_app_desc_t *app_desc = esp_app_get_description();
    ESP_LOGI(TAG, "===========================================");
    ESP_LOGI(TAG, " Westshore Watch X1 — Remote ID Sensor Node %s",
             app_desc->version);
    ESP_LOGI(TAG, " ESP32-C5  |  IDF %s", esp_get_idf_version());
    ESP_LOGI(TAG, "===========================================");
    ESP_LOGI(TAG, " Mode:     %s",
             g_config.mode == WSD_MODE_RELAY ? "BLE relay" : "UART only");
    ESP_LOGI(TAG, " Channels: %d-%d",
             g_config.ch_2g_start, g_config.ch_2g_stop);
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
    ESP_LOGI(TAG, "boot: starting output_task");
    err = output_task_start(output_queue);
    if (err != ESP_OK)
        ESP_LOGW(TAG, "Output task failed: %d", err);

    /* WiFi scanner — also starts the always-on config AP and HTTP server */
    ESP_LOGI(TAG, "boot: starting wifi_scanner");
    err = wifi_scanner_start(raw_queue);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Wi-Fi scanner failed: %d — halting", err);
        led_set_pattern(LED_PATTERN_ERROR);
        while (true) vTaskDelay(pdMS_TO_TICKS(1000));
    }
    ESP_LOGI(TAG, "boot: wifi_scanner up");

    vTaskDelay(pdMS_TO_TICKS(500));

    /* BLE stack init — unconditional so the detection advertiser (handle 2)
     * is available regardless of relay mode. */
    ESP_LOGI(TAG, "boot: ble_relay_init");
    err = ble_relay_init();
    if (err != ESP_OK)
        ESP_LOGW(TAG, "ble_relay_init failed: %d", err);
    ESP_LOGI(TAG, "boot: ble_relay_init done");

    /* BLE relay — only when mode == RELAY */
    if (g_config.mode == WSD_MODE_RELAY) {
        ESP_LOGI(TAG, "boot: ble_relay_start");
        err = ble_relay_start(relay_queue);
        if (err != ESP_OK)
            ESP_LOGW(TAG, "BLE relay failed: %d", err);
        ESP_LOGI(TAG, "boot: ble_relay_start done");
    }

    led_set_pattern(LED_PATTERN_SCANNING);

    ESP_LOGI(TAG, "System ready");
    ESP_LOGI(TAG, "  Wi-Fi scanner:  ch %d-%d, %dms dwell",
             g_config.ch_2g_start, g_config.ch_2g_stop, WSD_WIFI_DWELL_MS);
    ESP_LOGI(TAG, "  BLE relay:      %s",
             g_config.mode == WSD_MODE_RELAY ? "active" : "disabled");
    ESP_LOGI(TAG, "  Detection adv:  handle 2, company 0x08FF");
    ESP_LOGI(TAG, "  Config portal:  WestshoreWatch-XXXX → http://192.168.4.1");

    /* One-shot timer: if we survive WSD_OTA_VALIDATE_DELAY_S with all core
     * services up, commit the current image and cancel pending rollback.
     * If we crash before the timer fires, the bootloader will revert on the
     * next boot. */
    esp_timer_handle_t ota_validate_timer = NULL;
    const esp_timer_create_args_t targs = {
        .callback = &ota_mark_valid_cb,
        .name     = "ota_validate",
    };
    if (esp_timer_create(&targs, &ota_validate_timer) == ESP_OK) {
        esp_timer_start_once(ota_validate_timer,
                             (uint64_t)WSD_OTA_VALIDATE_DELAY_S * 1000000ULL);
        ESP_LOGI(TAG, "OTA: will mark image valid in %ds",
                 WSD_OTA_VALIDATE_DELAY_S);
    } else {
        ESP_LOGW(TAG, "OTA: failed to arm validate timer");
    }

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
