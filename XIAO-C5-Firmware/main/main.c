#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_app_desc.h"
#include "esp_ota_ops.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"
#include "nvs_flash.h"

#include "config.h"
#include "nvs_config.h"
#include "odid_decoder.h"
#include "wifi_scanner.h"
#include "led.h"
#include "ble_relay.h"
#include "status_led.h"
#include "modem_manager.h"
#include "detection_queue.h"
#include "cellular_uploader.h"
#include "gnss_reader.h"

static const char *TAG = "MAIN";

/* Seconds of healthy runtime before we tell the bootloader this image is good.
 * If we crash before this fires, otadata stays in pending-verify and the
 * bootloader rolls back to the previous slot on next boot. */
#define WSD_OTA_VALIDATE_DELAY_S 60

/* Upload watchdog — full system reboot if no successful upload in 10 min */
#define UPLOAD_WDT_TIMEOUT_MS    (10 * 60 * 1000)

/* "Degraded" LED: data session up + heartbeats fine, but detection POSTs are
 * failing. Trigger when we've ATTEMPTED a detection batch recently yet have no
 * recent detection SUCCESS. Detection attempts only happen when drones are in
 * range, so an idle node never trips this. */
#define DET_ATTEMPT_RECENT_MS    (60 * 1000)
#define DET_SUCCESS_STALE_MS     (30 * 1000)

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
static QueueHandle_t detect_queue = NULL;

/* Distribute one detection to the cellular upload queue.
 * Phone-paired mode used output_queue + relay_queue; this variant
 * feeds a single detect_queue that drains via HTTPS over cellular. */
static void distributor_task(void *arg)
{
    odid_detection_t det;
    while (true) {
        if (xQueueReceive(raw_queue, &det, portMAX_DELAY) == pdTRUE) {
            xQueueSend(detect_queue, &det, 0);
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

    /* Bi-color status LED (replaces onboard user LED for this variant) */
    ESP_LOGI(TAG, "boot: calling status_led_init");
    err = status_led_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "boot: status_led_init failed: %s", esp_err_to_name(err));
    }
    status_led_set(STATUS_LED_WARMING);   /* slow-blink yellow: booting */

    /* Load config — falls back to defaults on first boot */
    wsd_config_load(&g_config);

    /* ── CELLULAR X1 MODE ──────────────────────────────────────────────────
     *
     * Standalone cellular uplink via SIM7600G-H modem.
     * BLE relay + UART JSON output + config portal are NOT started.
     * Detection path: WiFi/BLE scan → detect_queue → HTTPS POST
     *
     * ────────────────────────────────────────────────────────────────────── */
    const esp_app_desc_t *app_desc = esp_app_get_description();
    ESP_LOGI(TAG, "===========================================");
    ESP_LOGI(TAG, " Westshore Watch Cellular X1 %s", app_desc->version);
    ESP_LOGI(TAG, " ESP32-C5  |  IDF %s", esp_get_idf_version());
    ESP_LOGI(TAG, "===========================================");
    ESP_LOGI(TAG, " Mode:     CELLULAR (SIM7600G-H)");
    ESP_LOGI(TAG, " Channels: %d-%d",
             g_config.ch_2g_start, g_config.ch_2g_stop);
    ESP_LOGI(TAG, " Node:     %s",
             g_config.node_name[0] ? g_config.node_name : "(unnamed)");

    /* Create detection queues */
    raw_queue    = xQueueCreate(WSD_DETECT_QUEUE_DEPTH, sizeof(odid_detection_t));
    detect_queue = xQueueCreate(WSD_DETECT_QUEUE_DEPTH, sizeof(odid_detection_t));

    if (!raw_queue || !detect_queue) {
        ESP_LOGE(TAG, "Queue creation failed — halting");
        status_led_set(STATUS_LED_FAULT);   /* solid red: hardware fault */
        while (true) vTaskDelay(pdMS_TO_TICKS(1000));
    }

    /* SPIFFS-backed offline buffer */
    ESP_LOGI(TAG, "boot: detection_queue_init");
    err = detection_queue_init();
    if (err != ESP_OK)
        ESP_LOGW(TAG, "detection_queue_init failed: %s — offline buffering disabled",
                 esp_err_to_name(err));

    /* Distributor: fan-out raw detections to cellular upload queue */
    xTaskCreate(distributor_task, "distributor", 2048, NULL,
                WSD_OUTPUT_TASK_PRIO + 2, NULL);

    /* WiFi scanner — promiscuous-mode RID detection.
     * NOTE: config portal + soft-AP start with wifi_scanner.  This is fine
     * because the portal uses WiFi AP mode (not STA) — AP mode does not
     * interfere with BLE detection coverage.  The portal is useful for
     * bench debugging even on the cellular variant. */
    ESP_LOGI(TAG, "boot: starting wifi_scanner");
    err = wifi_scanner_start(raw_queue);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Wi-Fi scanner failed: %d — will retry then restart", err);
        status_led_set(STATUS_LED_FAULT);   /* solid red: hardware fault */
        for (int retry = 0; retry < 3; retry++) {
            vTaskDelay(pdMS_TO_TICKS(2000));
            err = wifi_scanner_start(raw_queue);
            if (err == ESP_OK) {
                ESP_LOGW(TAG, "Wi-Fi scanner recovered on retry %d", retry + 1);
                break;
            }
            ESP_LOGE(TAG, "Wi-Fi scanner retry %d failed: %d", retry + 1, err);
        }
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Wi-Fi scanner failed after retries — restarting chip");
            vTaskDelay(pdMS_TO_TICKS(500));
            esp_restart();
        }
    }
    ESP_LOGI(TAG, "boot: wifi_scanner up");

    vTaskDelay(pdMS_TO_TICKS(500));

    /* BLE scanner — passive scan for BLE-based Remote ID broadcasts.
     * We still init ble_relay for the NimBLE host but do NOT start the
     * relay task or ext-adv handles — no phone-gateway use case. */
    ESP_LOGI(TAG, "boot: ble_relay_init (scanner only, no relay)");
    err = ble_relay_init();
    if (err != ESP_OK)
        ESP_LOGW(TAG, "ble_relay_init failed: %d", err);
    /* Deliberately NOT calling ble_relay_start() — no BLE relay TX */
    ESP_LOGI(TAG, "boot: BLE scanner active, relay TX disabled");

    /* GNSS reader — init the position cache BEFORE the modem task starts.
     * modem_manager polls AT+CGPSINFO (pre-net, time-boxed, then again in its
     * monitor loop until a fix lands) and feeds responses here; serialized
     * with HTTP POSTs on the UART, off the upload critical path. */
    ESP_LOGI(TAG, "boot: starting gnss_reader");
    err = gnss_reader_start();
    if (err != ESP_OK)
        ESP_LOGW(TAG, "gnss_reader_start failed: %s", esp_err_to_name(err));

    /* Cellular modem — starts the SIM7600 state machine in a background task */
    ESP_LOGI(TAG, "boot: starting modem_manager");
    err = modem_manager_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "modem_manager_start failed: %s — restarting",
                 esp_err_to_name(err));
        vTaskDelay(pdMS_TO_TICKS(500));
        esp_restart();
    }

    /* HTTPS uploader — drains detect_queue via native modem AT HTTP */
    ESP_LOGI(TAG, "boot: starting cellular_uploader");
    err = cellular_uploader_start(detect_queue);
    if (err != ESP_OK)
        ESP_LOGW(TAG, "cellular_uploader_start failed: %s", esp_err_to_name(err));

    ESP_LOGI(TAG, "System ready — Cellular X1");
    ESP_LOGI(TAG, "  Wi-Fi scanner:  ch %d-%d, %dms dwell",
             g_config.ch_2g_start, g_config.ch_2g_stop, WSD_WIFI_DWELL_MS);
    ESP_LOGI(TAG, "  BLE scanner:    passive (relay TX disabled)");
    ESP_LOGI(TAG, "  Cellular:       SIM7600G-H via UART1");
    ESP_LOGI(TAG, "  Config portal:  WestshoreWatch-XXXX → http://192.168.4.1");

    /* OTA validate timer */
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

    /* Main loop: WDT feed + upload watchdog + LED recency check.
     *
     * Upload watchdog triggers reboot only when the network is totally dead
     * (no HTTP response at all for 10 min).  Backend 4xx/5xx errors do NOT
     * trigger reboot — the backend might be down during the event but we
     * don't want reboot loops. */
    esp_task_wdt_add(NULL);
    while (true) {
        esp_task_wdt_reset();

        /* Upload watchdog: reboot if no HTTP response (any status) in 10 min */
        TickType_t last_resp = cellular_uploader_last_response();
        if (last_resp > 0) {
            uint32_t elapsed_ms = (xTaskGetTickCount() - last_resp) * portTICK_PERIOD_MS;
            if (elapsed_ms > UPLOAD_WDT_TIMEOUT_MS) {
                ESP_LOGE(TAG, "UPLOAD WATCHDOG: no HTTP response in %lums — rebooting",
                         (unsigned long)elapsed_ms);
                vTaskDelay(pdMS_TO_TICKS(500));
                esp_restart();
            }
        }

        /* LED recency (only while the cellular data session is up):
         *   fast-blink red    — degraded: detections attempted recently but failing
         *   slow-blink yellow — no successful upload (detection OR heartbeat) in 2 min
         *   solid yellow      — healthy */
        if (modem_manager_is_connected()) {
            TickType_t now = xTaskGetTickCount();
            TickType_t det_try = cellular_uploader_last_det_attempt();
            TickType_t det_ok  = cellular_uploader_last_det_success();

            bool det_active  = det_try > 0 &&
                (now - det_try) * portTICK_PERIOD_MS < DET_ATTEMPT_RECENT_MS;
            bool det_failing = det_active &&
                (det_ok == 0 ||
                 (now - det_ok) * portTICK_PERIOD_MS > DET_SUCCESS_STALE_MS);

            if (det_failing) {
                status_led_set(STATUS_LED_DEGRADED);    /* fast-blink red */
            } else {
                TickType_t last_ok = cellular_uploader_last_success();
                if (last_ok > 0) {
                    uint32_t since_upload_ms = (now - last_ok) * portTICK_PERIOD_MS;
                    if (since_upload_ms > 2 * 60 * 1000) {
                        status_led_set(STATUS_LED_WARMING);   /* slow-blink yellow */
                    } else {
                        status_led_set(STATUS_LED_HEALTHY);   /* solid yellow */
                    }
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}
