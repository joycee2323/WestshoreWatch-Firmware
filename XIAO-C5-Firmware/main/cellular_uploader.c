#include "cellular_uploader.h"
#include "detection_queue.h"
#include "modem_manager.h"
#include "gnss_reader.h"
#include "odid_decoder.h"
#include "status_led.h"
#include "config.h"
#include "esp_log.h"
#include "esp_http_client.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "CELL_UP";

#define UPLOAD_INTERVAL_MS      2000
#define BATCH_MAX               8
#define JSON_BUF_SIZE           4096
#define URL_BUF_SIZE            256
#define RETRY_COUNT             3
#define RETRY_BASE_MS           1000

static QueueHandle_t   s_queue;
static volatile TickType_t s_last_success;
static volatile TickType_t s_last_response;  /* any HTTP reply, even 4xx/5xx */

TickType_t cellular_uploader_last_success(void) { return s_last_success; }
TickType_t cellular_uploader_last_response(void) { return s_last_response; }

/* ── NVS config ───────────────────────────────────────────────────────────── */
static char s_device_id[64];
static char s_api_key[128];
static char s_backend_url[128];

static void load_config(void)
{
    nvs_handle_t h;
    size_t len;

    if (nvs_open("cell", NVS_READONLY, &h) != ESP_OK) {
        ESP_LOGW(TAG, "NVS 'cell' namespace not found — using defaults");
        strcpy(s_device_id, "cellular-x1-001");
        strcpy(s_api_key, "");
        strcpy(s_backend_url, "https://api.westshoredrone.com");
        return;
    }

    len = sizeof(s_device_id);
    if (nvs_get_str(h, "device_id", s_device_id, &len) != ESP_OK)
        strcpy(s_device_id, "cellular-x1-001");

    len = sizeof(s_api_key);
    if (nvs_get_str(h, "api_key", s_api_key, &len) != ESP_OK)
        strcpy(s_api_key, "");

    len = sizeof(s_backend_url);
    if (nvs_get_str(h, "backend_url", s_backend_url, &len) != ESP_OK)
        strcpy(s_backend_url, "https://api.westshoredrone.com");

    nvs_close(h);
    ESP_LOGI(TAG, "config: device=%s backend=%s key=%s***",
             s_device_id, s_backend_url,
             strlen(s_api_key) > 4 ? s_api_key : "(empty)");
}

/* ── JSON serialization ───────────────────────────────────────────────────── */
static int format_detection_json(const odid_detection_t *det, char *buf, size_t sz)
{
    int n = 0;
    n += snprintf(buf + n, sz - n, "{");

    if (det->has_basic_id) {
        n += snprintf(buf + n, sz - n,
            "\"uas_id\":\"%s\",\"id_type\":%d,\"ua_type\":%d,",
            det->basic_id.uas_id, det->basic_id.id_type, det->basic_id.ua_type);
    }

    if (det->has_location) {
        n += snprintf(buf + n, sz - n,
            "\"lat\":%.7f,\"lon\":%.7f,"
            "\"alt_baro\":%.1f,\"alt_geo\":%.1f,\"height\":%.1f,"
            "\"speed_h\":%.1f,\"speed_v\":%.1f,\"heading\":%u,"
            "\"status\":%d,",
            det->location.lat, det->location.lon,
            det->location.alt_baro, det->location.alt_geo,
            det->location.height,
            det->location.speed_horiz, det->location.speed_vert,
            det->location.heading, det->location.status);
    }

    if (det->has_system) {
        n += snprintf(buf + n, sz - n,
            "\"op_lat\":%.7f,\"op_lon\":%.7f,\"op_alt\":%.1f,",
            det->system.operator_lat, det->system.operator_lon,
            det->system.operator_alt_geo);
    }

    n += snprintf(buf + n, sz - n,
        "\"rssi\":%d,\"source\":%d,"
        "\"mac\":\"%02x:%02x:%02x:%02x:%02x:%02x\"}",
        det->rssi, det->source,
        det->mac[0], det->mac[1], det->mac[2],
        det->mac[3], det->mac[4], det->mac[5]);

    return n;
}

static int build_payload(odid_detection_t *batch, int count, char *buf, size_t sz)
{
    int n = 0;
    n += snprintf(buf + n, sz - n, "{\"drones\":[");
    for (int i = 0; i < count; i++) {
        if (i > 0) n += snprintf(buf + n, sz - n, ",");
        n += format_detection_json(&batch[i], buf + n, sz - n);
    }
    n += snprintf(buf + n, sz - n, "]");

    /* Attach node position if GNSS has a fix */
    gnss_position_t pos;
    if (gnss_reader_get_position(&pos)) {
        n += snprintf(buf + n, sz - n,
            ",\"node_position\":{\"lat\":%.7f,\"lon\":%.7f,"
            "\"alt_m\":%.1f,\"hdop\":%.1f}",
            pos.lat, pos.lon, pos.alt_m, pos.hdop);
    }

    n += snprintf(buf + n, sz - n, "}");
    return n;
}

/* ── HTTP POST with retry ─────────────────────────────────────────────────── */
static bool post_detections(const char *json, int json_len)
{
    char url[URL_BUF_SIZE];
    snprintf(url, sizeof(url), "%s/api/nodes/%s/detections",
             s_backend_url, s_device_id);

    for (int attempt = 0; attempt < RETRY_COUNT; attempt++) {
        esp_http_client_config_t cfg = {
            .url            = url,
            .method         = HTTP_METHOD_POST,
            .timeout_ms     = 10000,
        };
        esp_http_client_handle_t client = esp_http_client_init(&cfg);
        if (!client) continue;

        esp_http_client_set_header(client, "Content-Type", "application/json");
        esp_http_client_set_header(client, "X-Node-API-Key", s_api_key);
        esp_http_client_set_post_field(client, json, json_len);

        esp_err_t err = esp_http_client_perform(client);
        int status = esp_http_client_get_status_code(client);
        esp_http_client_cleanup(client);

        if (err == ESP_OK) {
            /* Got an HTTP response — network is alive regardless of status code */
            s_last_response = xTaskGetTickCount();

            if (status >= 200 && status < 300) {
                ESP_LOGI(TAG, "POST %s → %d (%d bytes)", url, status, json_len);
                return true;
            }
        }

        ESP_LOGW(TAG, "POST attempt %d failed: err=%s status=%d",
                 attempt + 1, esp_err_to_name(err), status);

        if (attempt < RETRY_COUNT - 1) {
            int delay = RETRY_BASE_MS * (1 << attempt);
            vTaskDelay(pdMS_TO_TICKS(delay));
        }
    }
    return false;
}

/* ── Uploader task ────────────────────────────────────────────────────────── */
static void uploader_task(void *arg)
{
    static char json_buf[JSON_BUF_SIZE];
    odid_detection_t batch[BATCH_MAX];

    load_config();

    while (true) {
        /* Wait for at least one detection or timeout */
        odid_detection_t det;
        int count = 0;

        if (xQueueReceive(s_queue, &det, pdMS_TO_TICKS(UPLOAD_INTERVAL_MS)) == pdTRUE) {
            batch[count++] = det;
            /* Drain up to BATCH_MAX without blocking */
            while (count < BATCH_MAX &&
                   xQueueReceive(s_queue, &det, 0) == pdTRUE) {
                batch[count++] = det;
            }
        }

        /* Also drain any SPIFFS-buffered detections from previous offline period */
        while (count < BATCH_MAX &&
               detection_queue_pop(&det) == ESP_OK) {
            batch[count++] = det;
        }

        if (count == 0) continue;

        if (!modem_manager_is_connected()) {
            /* Offline — buffer to SPIFFS */
            ESP_LOGW(TAG, "PPP down — buffering %d detections to SPIFFS", count);
            for (int i = 0; i < count; i++) {
                detection_queue_push(&batch[i]);
            }
            status_led_set(STATUS_LED_BLINK_YELLOW);
            continue;
        }

        /* Build JSON payload and POST */
        int len = build_payload(batch, count, json_buf, sizeof(json_buf));
        if (post_detections(json_buf, len)) {
            s_last_success = xTaskGetTickCount();
            ESP_LOGI(TAG, "uploaded %d detections (%d buffered)",
                     count, detection_queue_count());
        } else {
            ESP_LOGE(TAG, "upload failed — buffering %d detections", count);
            for (int i = 0; i < count; i++) {
                detection_queue_push(&batch[i]);
            }
        }
    }
}

esp_err_t cellular_uploader_start(QueueHandle_t detect_queue)
{
    s_queue = detect_queue;
    BaseType_t ret = xTaskCreate(uploader_task, "cell_upload", 8192, NULL,
                                 WSD_OUTPUT_TASK_PRIO, NULL);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "failed to create uploader task");
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "uploader task started");
    return ESP_OK;
}
