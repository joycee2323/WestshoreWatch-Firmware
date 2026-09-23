/* ═══════════════════════════════════════════════════════════════════════
 * TEMPORARY FIELD-TEST INSTRUMENTATION — see field_test_5ghz_log.h for the
 * strip-out instructions. Not permanent.
 * ═══════════════════════════════════════════════════════════════════════ */
#include "field_test_5ghz_log.h"
#include "esp_spiffs.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <sys/stat.h>

static const char *TAG = "FIELD_TEST_5G";

#define FIELD_TEST_BASE_PATH   "/ft5g"
#define FIELD_TEST_LOG_PATH    FIELD_TEST_BASE_PATH "/5ghz.log"
/* A few hundred KB, per spec — comfortably inside the 0xE0000 (896KB)
 * "storage" partition with headroom for SPIFFS metadata. No rotation: once
 * hit, logging just stops (see field_test_5ghz_log_record). */
#define FIELD_TEST_LOG_MAX_BYTES (300 * 1024)

static bool s_mounted    = false;
static bool s_cap_logged = false;

esp_err_t field_test_5ghz_log_init(void)
{
    esp_vfs_spiffs_conf_t conf = {
        .base_path              = FIELD_TEST_BASE_PATH,
        .partition_label        = "storage",  /* existing spiffs partition in partitions.esp32c5.csv */
        .max_files              = 2,
        .format_if_mount_failed = true,
    };
    esp_err_t err = esp_vfs_spiffs_register(&conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPIFFS mount failed: 0x%x (%s) — field-test logging disabled",
                 err, esp_err_to_name(err));
        return err;
    }
    s_mounted = true;

    struct stat st;
    size_t existing = (stat(FIELD_TEST_LOG_PATH, &st) == 0) ? (size_t)st.st_size : 0;
    ESP_LOGI(TAG, "Field-test 5GHz log ready at %s (%u bytes already present, cap %u)",
             FIELD_TEST_LOG_PATH, (unsigned)existing, (unsigned)FIELD_TEST_LOG_MAX_BYTES);
    return ESP_OK;
}

void field_test_5ghz_log_record(uint8_t channel, const odid_detection_t *det)
{
    if (!s_mounted) return;

    struct stat st;
    size_t cur = (stat(FIELD_TEST_LOG_PATH, &st) == 0) ? (size_t)st.st_size : 0;
    if (cur >= FIELD_TEST_LOG_MAX_BYTES) {
        if (!s_cap_logged) {
            ESP_LOGW(TAG, "Field-test 5GHz log hit its %u-byte cap — no longer appending",
                     (unsigned)FIELD_TEST_LOG_MAX_BYTES);
            s_cap_logged = true;
        }
        return;
    }

    FILE *fp = fopen(FIELD_TEST_LOG_PATH, "a");
    if (!fp) {
        ESP_LOGW(TAG, "Failed to open %s for append", FIELD_TEST_LOG_PATH);
        return;
    }

    uint32_t ts_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    const char *uas_id = (det && det->has_basic_id && det->basic_id.uas_id[0])
                          ? det->basic_id.uas_id : "-";

    /* Mirrors ble_relay.c's LOC_VALID macro (relay_task, ~line 684) exactly —
     * same has_location + lat/lon range + non-zero checks that gate whether
     * this decode would have been eligible for Pack-handle (handle 1)
     * advertising. Kept as a literal copy rather than a shared header since
     * the original is a function-local macro, #undef'd at the end of
     * relay_task — see this field-test file's own strip-out note. */
    bool loc_valid = det && det->has_location &&
                      det->location.lat >= -90.0f && det->location.lat <= 90.0f &&
                      det->location.lon >= -180.0f && det->location.lon <= 180.0f &&
                      (det->location.lat != 0.0f || det->location.lon != 0.0f);

    char latlon[32];
    if (det && det->has_location) {
        snprintf(latlon, sizeof(latlon), "%.6f/%.6f",
                 (double)det->location.lat, (double)det->location.lon);
    } else {
        snprintf(latlon, sizeof(latlon), "-");
    }

    fprintf(fp, "%lu,%u,%s,%d,%s\n", (unsigned long)ts_ms, (unsigned)channel,
             uas_id, loc_valid ? 1 : 0, latlon);
    fclose(fp);
}

esp_err_t field_test_5ghz_log_serve(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/plain");

    if (!s_mounted) {
        httpd_resp_send(req, "field-test log not mounted\n", HTTPD_RESP_USE_STRLEN);
        return ESP_OK;
    }

    FILE *fp = fopen(FIELD_TEST_LOG_PATH, "r");
    if (!fp) {
        httpd_resp_send(req, "field-test log empty (no 5GHz detections yet)\n",
                         HTTPD_RESP_USE_STRLEN);
        return ESP_OK;
    }

    char buf[512];
    size_t n;
    while ((n = fread(buf, 1, sizeof(buf), fp)) > 0) {
        if (httpd_resp_send_chunk(req, buf, n) != ESP_OK) {
            fclose(fp);
            return ESP_FAIL;
        }
    }
    fclose(fp);
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}
