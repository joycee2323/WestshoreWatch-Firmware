#include "cellular_uploader.h"
#include "detection_queue.h"
#include "modem_manager.h"
#include "modem_http.h"
#include "gnss_reader.h"
#include "odid_decoder.h"
#include "status_led.h"
#include "config.h"
#include "esp_log.h"
#include "esp_app_desc.h"
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

/* Idle heartbeat keeps the node 'online' when no drones are in range AND
 * carries the node's live GPS so a MOBILE node's map position stays current
 * between detections. Cadence is matched to the GNSS re-poll interval
 * (~15 s, modem_manager GNSS_REPOLL_EVERY) so heartbeat-driven position
 * granularity isn't bottlenecked below the rate the cache actually refreshes
 * — every fresh fix gets reported on the next beat.
 *
 * The backend marks an x1 node offline after 120 s of inactivity (server.js
 * offline cron), so 15 s keeps 8× headroom — comfortably safe. The extra
 * traffic is negligible: a ~67-byte POST every 15 s. Same
 * /api/nodes/heartbeat endpoint and X-Node-API-Key auth the Sentinel uses. */
#define HEARTBEAT_INTERVAL_MS   15000

static QueueHandle_t   s_queue;
static volatile TickType_t s_last_success;
static volatile TickType_t s_last_response;  /* any HTTP reply, even 4xx/5xx */
static volatile TickType_t s_last_heartbeat; /* tick of last heartbeat attempt */
/* Detection-POST health, tracked separately from s_last_success (which the
 * heartbeat also refreshes). Lets the LED show "online but detections failing"
 * — a degraded state the heartbeat would otherwise paper over. */
static volatile TickType_t s_last_det_attempt;  /* last detection batch POST tried */
static volatile TickType_t s_last_det_success;  /* last detection batch POST 2xx   */
static char s_fw_version[32];                /* app version for heartbeat payload */

TickType_t cellular_uploader_last_success(void) { return s_last_success; }
TickType_t cellular_uploader_last_response(void) { return s_last_response; }
TickType_t cellular_uploader_last_det_attempt(void) { return s_last_det_attempt; }
TickType_t cellular_uploader_last_det_success(void) { return s_last_det_success; }

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
/* Emit ONE drone object in the canonical schema the backend ingest path reads
 * (routes/nodes.js + routes/detections.js) and the Android node sends
 * (DetectionUploader.kt): {id, lat, lon, alt, spd, hdg, op_lat, op_lon}.
 *
 * CRITICAL: the backend keys on `drone.id` and does `if (!uas_id) continue;`,
 * so the field MUST be "id" (not "uas_id") or the drone is silently dropped
 * (HTTP 200, stored:0). Likewise altitude/speed/heading must be "alt"/"spd"/
 * "hdg". `alt` is the GEODETIC altitude (Android maps alt = altGeo). Fields
 * the backend ignores (id_type, baro alt, height, vertical speed, status,
 * rssi, mac, …) are omitted to match the canonical body exactly and keep the
 * cellular payload small. `ts` (ODID self-clock) and `nickname` are omitted —
 * the firmware doesn't have them; the backend treats them as null (same as the
 * Sentinel path), so the coalescer/stale gate behaves identically. */
static int format_detection_json(const odid_detection_t *det, char *buf, size_t sz)
{
    int n = 0;

    /* id is mandatory — without it the backend skips the drone. */
    n += snprintf(buf + n, sz - n, "{\"id\":\"%s\"",
                  det->has_basic_id ? det->basic_id.uas_id : "");

    if (det->has_location) {
        n += snprintf(buf + n, sz - n,
            ",\"lat\":%.7f,\"lon\":%.7f,\"alt\":%.1f,\"spd\":%.1f,\"hdg\":%u",
            det->location.lat, det->location.lon,
            det->location.alt_geo,        /* canonical alt = geodetic altitude */
            det->location.speed_horiz,    /* spd = horizontal speed */
            det->location.heading);       /* hdg */
    }

    if (det->has_system) {
        n += snprintf(buf + n, sz - n,
            ",\"op_lat\":%.7f,\"op_lon\":%.7f",
            det->system.operator_lat, det->system.operator_lon);
    }

    n += snprintf(buf + n, sz - n, "}");
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

    /* Attach node position if GNSS has a fix.
     * hdop is OMITTED when unknown (0) rather than sent as 0 — AT+CGPSINFO
     * carries no HDOP, and a literal 0 would read as "perfect accuracy" if
     * a backend ever persists this field. Absent field = unknown. A real
     * HDOP is always > 0, so this only emits hdop when genuinely measured. */
    gnss_position_t pos;
    if (gnss_reader_get_position(&pos)) {
        n += snprintf(buf + n, sz - n,
            ",\"node_position\":{\"lat\":%.7f,\"lon\":%.7f,\"alt_m\":%.1f",
            pos.lat, pos.lon, pos.alt_m);
        if (pos.hdop > 0.0f) {
            n += snprintf(buf + n, sz - n, ",\"hdop\":%.1f", pos.hdop);
        }
        n += snprintf(buf + n, sz - n, "}");
    }

    n += snprintf(buf + n, sz - n, "}");
    return n;
}

/* ── HTTP POST with retry (native modem AT HTTP transport) ────────────────────
 * Transport is the SIM7600's built-in AT HTTP(S) stack (modem_http), shared
 * with GPS polling on the one UART AT channel; modem_http holds the UART lock
 * for the whole HTTPINIT…HTTPTERM transaction so nothing interleaves. Same
 * endpoint, same X-Node-API-Key auth, same JSON body as the PPP uploader —
 * only the transport changed. */
static bool post_detections(const char *json, int json_len)
{
    char url[URL_BUF_SIZE];
    snprintf(url, sizeof(url), "%s/api/nodes/%s/detections",
             s_backend_url, s_device_id);

    /* Custom header line for AT+HTTPPARA "USERDATA" — same scheme the PPP
     * uploader set via esp_http_client_set_header(). Content-Type is sent
     * separately by modem_http via the CONTENT param. */
    char auth[160];
    snprintf(auth, sizeof(auth), "X-Node-API-Key: %s", s_api_key);

    for (int attempt = 0; attempt < RETRY_COUNT; attempt++) {
        modem_http_result_t r;
        esp_err_t err = modem_http_post(url, auth, json, json_len, &r);

        /* Any real HTTP reply (even 4xx/5xx) proves the link is alive — feeds
         * the upload watchdog so it only reboots on a totally dead network. A
         * modem-side (7xx) error is NOT a network reply, so don't count it. */
        if (r.http_status >= 0) {
            s_last_response = xTaskGetTickCount();
        }

        if (err == ESP_OK) {
            ESP_LOGI(TAG, "POST %s → HTTP %d (%d bytes, attempt %d/%d)",
                     url, r.http_status, json_len, attempt + 1, RETRY_COUNT);
            return true;
        }

        if (r.modem_err) {
            ESP_LOGW(TAG, "POST attempt %d/%d: modem error %d (715=TLS fail, etc.)",
                     attempt + 1, RETRY_COUNT, r.modem_err);
        } else if (r.http_status >= 0) {
            ESP_LOGW(TAG, "POST attempt %d/%d: HTTP %d",
                     attempt + 1, RETRY_COUNT, r.http_status);
        } else {
            ESP_LOGW(TAG, "POST attempt %d/%d: AT/transport failure (%s)",
                     attempt + 1, RETRY_COUNT, esp_err_to_name(err));
        }

        if (attempt < RETRY_COUNT - 1) {
            int delay = RETRY_BASE_MS * (1 << attempt);
            vTaskDelay(pdMS_TO_TICKS(delay));
        }
    }
    return false;
}

/* ── Heartbeat (idle keep-alive) ──────────────────────────────────────────────
 * POST /api/nodes/heartbeat with X-Node-API-Key auth. Body carries
 * {connection_type, firmware_version} plus the node's live GPS as top-level
 * lat/lon — this is a MOBILE node, so its map position must stay current
 * during quiet (no-detection) periods via the heartbeat, not just on
 * detections. lat/lon are the exact fields the backend heartbeat handler
 * consumes (it updates nodes.last_lat/last_lon for non-sentinel nodes);
 * absent position leaves the stored value untouched (backend COALESCE).
 *
 * The position comes from gnss_reader_get_position() — a pure read of the
 * cache the modem monitor loop re-polls every ~15s under the AT mutex. The
 * read touches no UART/AT channel itself, so no new locking is needed; the
 * POST below still serializes through modem_http_post()'s UART lock as
 * before. On no fix we omit lat/lon (gnss_reader keeps last-known-good once
 * it has one, so a momentary dropout still sends the last position).
 *
 * Routed through modem_http_post() so it shares the native-AT-HTTP transport.
 * A successful beat refreshes the node's last_seen/online state and keeps the
 * status LED green + the upload watchdog fed while idle. */
static bool post_heartbeat(void)
{
    char url[URL_BUF_SIZE];
    snprintf(url, sizeof(url), "%s/api/nodes/heartbeat", s_backend_url);

    char auth[160];
    snprintf(auth, sizeof(auth), "X-Node-API-Key: %s", s_api_key);

    char body[224];
    gnss_position_t pos;
    int len;
    if (gnss_reader_get_position(&pos)) {
        /* 7 dp matches the detection path's node_position precision. */
        len = snprintf(body, sizeof(body),
            "{\"connection_type\":\"cellular\",\"firmware_version\":\"%s\","
            "\"lat\":%.7f,\"lon\":%.7f}",
            s_fw_version, pos.lat, pos.lon);
    } else {
        len = snprintf(body, sizeof(body),
            "{\"connection_type\":\"cellular\",\"firmware_version\":\"%s\"}",
            s_fw_version);
    }

    modem_http_result_t r;
    esp_err_t err = modem_http_post(url, auth, body, len, &r);

    if (r.http_status >= 0) {
        s_last_response = xTaskGetTickCount();
    }
    if (err == ESP_OK) {
        s_last_success = xTaskGetTickCount();   /* keep LED green when idle */
        ESP_LOGI(TAG, "heartbeat → HTTP %d", r.http_status);
        return true;
    }
    if (r.modem_err) {
        ESP_LOGW(TAG, "heartbeat failed: modem error %d", r.modem_err);
    } else {
        ESP_LOGW(TAG, "heartbeat failed: http=%d err=%s",
                 r.http_status, esp_err_to_name(err));
    }
    return false;
}

/* ── Uploader task ────────────────────────────────────────────────────────── */
static void uploader_task(void *arg)
{
    static char json_buf[JSON_BUF_SIZE];
    odid_detection_t batch[BATCH_MAX];

    load_config();

    /* Firmware version for the heartbeat payload (captured once). */
    const esp_app_desc_t *desc = esp_app_get_description();
    strlcpy(s_fw_version, desc ? desc->version : "unknown", sizeof(s_fw_version));

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

        /* Idle heartbeat: fires on its own interval regardless of detections,
         * so the node stays 'online' when no drone is in range. The loop wakes
         * at least every UPLOAD_INTERVAL_MS (queue timeout), so this is checked
         * ~every 2 s. Stamp the attempt time even on failure to avoid retrying
         * every loop; the 4× headroom absorbs an occasional miss. */
        if (modem_manager_is_connected()) {
            TickType_t now = xTaskGetTickCount();
            if (s_last_heartbeat == 0 ||
                (now - s_last_heartbeat) * portTICK_PERIOD_MS >= HEARTBEAT_INTERVAL_MS) {
                s_last_heartbeat = now;
                post_heartbeat();
            }
        }

        if (count == 0) continue;

        if (!modem_manager_is_connected()) {
            /* Offline — buffer to SPIFFS */
            ESP_LOGW(TAG, "cellular link down — buffering %d detections to SPIFFS", count);
            for (int i = 0; i < count; i++) {
                detection_queue_push(&batch[i]);
            }
            status_led_set(STATUS_LED_WARMING);   /* slow-blink yellow: offline buffering */
            continue;
        }

        /* Build JSON payload and POST */
        int len = build_payload(batch, count, json_buf, sizeof(json_buf));
        s_last_det_attempt = xTaskGetTickCount();
        if (post_detections(json_buf, len)) {
            TickType_t now = xTaskGetTickCount();
            s_last_success     = now;
            s_last_det_success = now;
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
