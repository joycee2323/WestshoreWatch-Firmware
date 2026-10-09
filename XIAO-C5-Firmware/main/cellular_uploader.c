#include "cellular_uploader.h"
#include "detection_queue.h"
#include "modem_manager.h"
#include "modem_http.h"
#include "gnss_reader.h"
#include "odid_decoder.h"
#include "status_led.h"
#include "config.h"
#include "upload_batch.h"
#include "esp_log.h"
#include "esp_app_desc.h"
#include "esp_system.h"
#include "esp_timer.h"
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

/* The heartbeat runs only after pending detections are sent and never retries.
 * Its own timing caps a slow or dead heartbeat at ~12 s (+2 s HTTPTERM) — the
 * old path used the detection timeouts and could hold the upload task ~65 s
 * on a lost +HTTPACTION URC, with frames queuing behind it. */
static const modem_http_opts_t HEARTBEAT_HTTP_OPTS = {
    .short_ms   = 3000,
    .data_ok_ms = 3000,
    .urc_ms     = 10000,
    .budget_ms  = 12000,
};

/* Frames drained per cycle: the whole RAM queue plus the one that woke us. */
#define DRAIN_MAX               (WSD_DETECT_QUEUE_DEPTH + 1)

static QueueHandle_t   s_queue;
static volatile uint32_t s_dropped_full;     /* oldest frames dropped: queue full */
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

/* ── Receive time ─────────────────────────────────────────────────────────── */
/* Uptime in ms, 32-bit (wraps after ~49.7 days; only differences are used). */
static uint32_t uptime_ms(void)
{
    return (uint32_t)pdTICKS_TO_MS(xTaskGetTickCount());
}

QueueHandle_t cellular_uploader_create_queue(int depth)
{
    s_queue = xQueueCreate(depth, sizeof(upload_frame_t));
    return s_queue;
}

void cellular_uploader_submit(const odid_detection_t *det)
{
    /* Static: upload_frame_t is too big for the distributor's 2 KB stack, and
     * the distributor is the only caller. */
    static upload_frame_t f, oldest;
    if (!s_queue || !det) return;
    f.det   = *det;
    f.rx_ms = uptime_ms();
    f.live  = true;
    if (xQueueSend(s_queue, &f, 0) != pdTRUE) {
        /* Full: drop the OLDEST frame so the newest is never the one lost. */
        if (xQueueReceive(s_queue, &oldest, 0) == pdTRUE) s_dropped_full++;
        xQueueSend(s_queue, &f, 0);
    }
}

/* ── JSON serialization ───────────────────────────────────────────────────── */
/* Drone objects come from upload_batch_format_drone(): the canonical schema
 * routes/nodes.js reads ({id, lat, lon, alt, spd, hdg, op_lat, op_lon}, field
 * names exactly as before — "id" is mandatory or the backend skips the drone)
 * plus two per-frame time fields:
 *   ts     — the drone's own ODID Location timestamp, tenths of a second since
 *            the top of the UTC hour (0..35999); omitted when the drone sends
 *            "unknown" (0xFFFF). The backend's stale gate (routes/nodes.js →
 *            services/odidStaleGate.js) uses it exactly as it does for the
 *            phone app: frames more than MAX_ODID_AGE_DS (5 min) old are
 *            rejected, as are repeats of an already-stored ts. It needs no
 *            node clock, survives the SPIFFS spool (it is inside the stored
 *            odid_detection_t) and a reboot. Failure case: a drone with a
 *            wrong or frozen clock — the gate's frozen rule covers frozen
 *            repeats; a wrong clock can make fresh frames look stale.
 *   age_ms — ms from receive (distributor) to payload build, live frames only;
 *            omitted for spool replays (the spool format has no receive time).
 *            Diagnostic: heard→POST-done ≈ age_ms + post_ms. */
static int build_payload(const upload_frame_t *batch, int count, char *buf, size_t sz,
                         uint32_t now, uint32_t *age_min, uint32_t *age_max)
{
    int n = 0;
    *age_min = UINT32_MAX; *age_max = 0;
    n += snprintf(buf + n, sz - n, "{\"drones\":[");
    for (int i = 0; i < count; i++) {
        if (i > 0) n += snprintf(buf + n, sz - n, ",");
        n += upload_batch_format_drone(&batch[i], now, buf + n, sz - n);
        if (batch[i].live) {
            uint32_t a = now - batch[i].rx_ms;
            if (a < *age_min) *age_min = a;
            if (a > *age_max) *age_max = a;
        }
    }
    if (*age_min == UINT32_MAX) *age_min = 0;
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
        esp_err_t err = modem_http_post_opts(url, auth, json, json_len, &r, NULL);

        /* Any real HTTP reply (even 4xx/5xx) proves the link is alive — feeds
         * the upload watchdog so it only reboots on a totally dead network. A
         * modem-side (7xx) error is NOT a network reply, so don't count it. */
        if (r.http_status >= 0) {
            s_last_response = xTaskGetTickCount();
        }

        if (err == ESP_OK) {
            ESP_LOGI(TAG, "POST %s → HTTP %d (%d bytes, attempt %d/%d) %lums",
                     url, r.http_status, json_len, attempt + 1, RETRY_COUNT,
                     (unsigned long)r.elapsed_ms);
            return true;
        }

        if (r.modem_err) {
            ESP_LOGW(TAG, "POST attempt %d/%d: modem error %d (715=TLS fail, etc.) stage=%s %lums",
                     attempt + 1, RETRY_COUNT, r.modem_err, r.stage, (unsigned long)r.elapsed_ms);
        } else if (r.http_status >= 0) {
            ESP_LOGW(TAG, "POST attempt %d/%d: HTTP %d %lums",
                     attempt + 1, RETRY_COUNT, r.http_status, (unsigned long)r.elapsed_ms);
        } else {
            ESP_LOGW(TAG, "POST attempt %d/%d: AT/transport failure (%s) stage=%s %lums",
                     attempt + 1, RETRY_COUNT, esp_err_to_name(err), r.stage,
                     (unsigned long)r.elapsed_ms);
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
static const char *reset_reason_str(esp_reset_reason_t r)
{
    switch (r) {
        case ESP_RST_POWERON:    return "POWERON";
        case ESP_RST_EXT:        return "EXT";
        case ESP_RST_SW:         return "SW";
        case ESP_RST_PANIC:      return "PANIC";
        case ESP_RST_INT_WDT:    return "INT_WDT";
        case ESP_RST_TASK_WDT:   return "TASK_WDT";
        case ESP_RST_WDT:        return "WDT";
        case ESP_RST_DEEPSLEEP:  return "DEEPSLEEP";
        case ESP_RST_BROWNOUT:   return "BROWNOUT";
        case ESP_RST_SDIO:       return "SDIO";
        case ESP_RST_USB:        return "USB";
        case ESP_RST_JTAG:       return "JTAG";
        case ESP_RST_EFUSE:      return "EFUSE";
        case ESP_RST_PWR_GLITCH: return "PWR_GLITCH";
        case ESP_RST_CPU_LOCKUP: return "CPU_LOCKUP";
        default:                 return "UNKNOWN";
    }
}

/* Heartbeat body: {connection_type, firmware_version, [lat, lon], reset_reason,
 * uptime_s}. reset_reason (why this boot happened — esp_reset_reason()) and
 * uptime_s are diagnostics: the backend heartbeat route reads only the fields
 * it knows and ignores these, so they are visible in request logs now and can
 * be stored later without a firmware change. One attempt, own short timing
 * (HEARTBEAT_HTTP_OPTS); a failure is dropped and logged with its stage. */
static bool post_heartbeat(void)
{
    char url[URL_BUF_SIZE];
    snprintf(url, sizeof(url), "%s/api/nodes/heartbeat", s_backend_url);

    char auth[160];
    snprintf(auth, sizeof(auth), "X-Node-API-Key: %s", s_api_key);

    char body[320];
    gnss_position_t pos;
    int len;
    const char *rst = reset_reason_str(esp_reset_reason());
    unsigned long up_s = (unsigned long)(esp_timer_get_time() / 1000000LL);
    if (gnss_reader_get_position(&pos)) {
        /* 7 dp matches the detection path's node_position precision. */
        len = snprintf(body, sizeof(body),
            "{\"connection_type\":\"cellular\",\"firmware_version\":\"%s\","
            "\"lat\":%.7f,\"lon\":%.7f,\"reset_reason\":\"%s\",\"uptime_s\":%lu}",
            s_fw_version, pos.lat, pos.lon, rst, up_s);
    } else {
        len = snprintf(body, sizeof(body),
            "{\"connection_type\":\"cellular\",\"firmware_version\":\"%s\","
            "\"reset_reason\":\"%s\",\"uptime_s\":%lu}",
            s_fw_version, rst, up_s);
    }

    modem_http_result_t r;
    esp_err_t err = modem_http_post_opts(url, auth, body, len, &r, &HEARTBEAT_HTTP_OPTS);

    if (r.http_status >= 0) {
        s_last_response = xTaskGetTickCount();
    }
    if (err == ESP_OK) {
        s_last_success = xTaskGetTickCount();   /* keep LED green when idle */
        ESP_LOGI(TAG, "heartbeat → HTTP %d %lums", r.http_status, (unsigned long)r.elapsed_ms);
        return true;
    }
    ESP_LOGW(TAG, "heartbeat failed: stage=%s http=%d modem_err=%d err=%s %lums (dropped, no retry)",
             r.stage, r.http_status, r.modem_err, esp_err_to_name(err),
             (unsigned long)r.elapsed_ms);
    return false;
}

/* ── Uploader task ────────────────────────────────────────────────────────── */
static void uploader_task(void *arg)
{
    static char json_buf[JSON_BUF_SIZE];
    /* Static: ~DRAIN_MAX frames is far more than the task stack. */
    static upload_frame_t drain[DRAIN_MAX];
    static upload_frame_t batch[BATCH_MAX];
    uint32_t dropped_reported = 0;

    load_config();

    /* Firmware version for the heartbeat payload (captured once). */
    const esp_app_desc_t *desc = esp_app_get_description();
    strlcpy(s_fw_version, desc ? desc->version : "unknown", sizeof(s_fw_version));

    while (true) {
        /* ── 1. Wait for a frame (or the 2 s tick), then drain the WHOLE queue.
         * Every cycle sends the newest frame per drone, so frames never sit
         * behind an 8-frame FIFO window as they did in 1.2.1. */
        int n = 0;
        if (xQueueReceive(s_queue, &drain[0], pdMS_TO_TICKS(UPLOAD_INTERVAL_MS)) == pdTRUE) {
            n = 1;
            while (n < DRAIN_MAX && xQueueReceive(s_queue, &drain[n], 0) == pdTRUE) n++;
        }

        upload_batch_stats_t st = {0};
        int count = upload_batch_collapse(drain, n, batch, BATCH_MAX, &st);
        uint32_t dropped_total = s_dropped_full;
        uint32_t dropped_now = dropped_total - dropped_reported;
        dropped_reported = dropped_total;
        bool connected = modem_manager_is_connected();

        if (!connected) {
            if (count > 0) {
                /* Offline — buffer to SPIFFS (newest frame per drone only). */
                ESP_LOGW(TAG, "cellular link down — buffering %d detections to SPIFFS", count);
                for (int i = 0; i < count; i++) {
                    detection_queue_push(&batch[i].det);
                }
                status_led_set(STATUS_LED_WARMING);   /* slow-blink yellow: offline buffering */
            }
            if (n > 0 || dropped_now > 0) {
                ESP_LOGI(TAG, "cycle: offline drained=%d drones=%d collapsed=%d no_id=%d over_cap=%d "
                         "q=%u spool=%d dropped_full=%lu",
                         n, count, st.collapsed, st.no_id, st.over_cap,
                         (unsigned)uxQueueMessagesWaiting(s_queue), detection_queue_count(),
                         (unsigned long)dropped_now);
            }
            continue;
        }

        /* ── 2. Fill free slots from the spool, NEWEST first. A spooled frame of
         * a drone already in the batch is older than the live one: dropped. */
        int live = count, spool_in = 0;
        while (count < BATCH_MAX && detection_queue_count() > 0) {
            odid_detection_t sp;
            if (detection_queue_pop_newest(&sp) != ESP_OK) break;
            int before = count;
            count = upload_batch_add_spool(batch, count, BATCH_MAX, &sp, &st);
            if (count > before) spool_in++;
        }

        /* ── 3. Detections first. */
        if (count > 0) {
            uint32_t t0 = uptime_ms(), age_min, age_max;
            int len = build_payload(batch, count, json_buf, sizeof(json_buf), t0, &age_min, &age_max);
            s_last_det_attempt = xTaskGetTickCount();
            bool ok = post_detections(json_buf, len);
            uint32_t post_ms = uptime_ms() - t0;
            if (ok) {
                TickType_t now = xTaskGetTickCount();
                s_last_success     = now;
                s_last_det_success = now;
                ESP_LOGI(TAG, "uploaded %d detections (%d buffered)",
                         count, detection_queue_count());
            } else {
                ESP_LOGE(TAG, "upload failed — buffering %d detections", count);
                for (int i = 0; i < count; i++) {
                    detection_queue_push(&batch[i].det);
                }
            }
            ESP_LOGI(TAG, "cycle: drained=%d drones=%d live=%d spool_in=%d collapsed=%d no_id=%d "
                     "over_cap=%d spool_skipped=%d q=%u spool=%d dropped_full=%lu "
                     "age_ms=%lu..%lu post_ms=%lu %s",
                     n, count, live, spool_in, st.collapsed, st.no_id, st.over_cap,
                     st.spool_skipped, (unsigned)uxQueueMessagesWaiting(s_queue),
                     detection_queue_count(), (unsigned long)dropped_now,
                     (unsigned long)age_min, (unsigned long)age_max,
                     (unsigned long)post_ms, ok ? "ok" : "FAILED");
        } else if (dropped_now > 0 || st.no_id > 0) {
            ESP_LOGI(TAG, "cycle: drained=%d drones=0 no_id=%d dropped_full=%lu",
                     n, st.no_id, (unsigned long)dropped_now);
        }

        /* ── 4. Heartbeat second: only when no detection is waiting, so it can
         * never delay one. While frames keep arriving the backend counts the
         * detection POSTs as liveness. One attempt, ~12 s cap; stamped even on
         * failure so a failing beat isn't retried every 2 s. */
        if (uxQueueMessagesWaiting(s_queue) == 0) {
            TickType_t now = xTaskGetTickCount();
            if (s_last_heartbeat == 0 ||
                (now - s_last_heartbeat) * portTICK_PERIOD_MS >= HEARTBEAT_INTERVAL_MS) {
                s_last_heartbeat = now;
                post_heartbeat();
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
