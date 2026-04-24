#include "ota_handler.h"
#include "led.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_log.h"
#include "esp_http_server.h"
#include <string.h>

static const char *TAG = "OTA";

/* ─────────────────────────────────────────────────────────────────────────────
 * OTA upload handler — receives multipart/form-data firmware binary,
 * writes it to the inactive OTA partition, then reboots.
 *
 * The form must POST to /ota with enctype="multipart/form-data" and a
 * single file field named "firmware".
 * ───────────────────────────────────────────────────────────────────────────── */

/* Minimal multipart boundary parser state */
typedef enum {
    MP_SEEK_BOUNDARY = 0,
    MP_SEEK_HEADER_END,
    MP_DATA,
    MP_DONE,
} mp_state_t;

esp_err_t ota_upload_handler(httpd_req_t *req)
{
    esp_err_t err;

    /* Get content type and extract boundary */
    char content_type[128] = {0};
    if (httpd_req_get_hdr_value_str(req, "Content-Type",
                                    content_type, sizeof(content_type)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing Content-Type");
        return ESP_OK;
    }

    char *bp = strstr(content_type, "boundary=");
    if (!bp) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No boundary");
        return ESP_OK;
    }
    char boundary[72];
    snprintf(boundary, sizeof(boundary), "--%s", bp + 9);
    size_t blen = strlen(boundary);

    /* Find inactive OTA partition */
    const esp_partition_t *update_part =
        esp_ota_get_next_update_partition(NULL);
    if (!update_part) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                            "No OTA partition");
        return ESP_OK;
    }
    ESP_LOGI(TAG, "Writing to partition: %s", update_part->label);

    esp_ota_handle_t ota_handle = 0;
    err = esp_ota_begin(update_part, OTA_WITH_SEQUENTIAL_WRITES, &ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin failed: %d", err);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                            "OTA begin failed");
        return ESP_OK;
    }

    /* Stream and parse multipart body */
    char buf[1024];
    mp_state_t state = MP_SEEK_BOUNDARY;
    bool ota_started = false;
    int remaining = req->content_len;
    size_t written = 0;

    /* Accumulation buffer for boundary detection across chunks */
    static char acc[2048];
    size_t acc_len = 0;

    while (remaining > 0) {
        int to_read = remaining < (int)sizeof(buf) ? remaining : (int)sizeof(buf);
        int rd = httpd_req_recv(req, buf, to_read);
        if (rd <= 0) {
            ESP_LOGE(TAG, "Read error: %d", rd);
            goto fail;
        }
        remaining -= rd;

        /* Append to accumulation buffer */
        if (acc_len + rd > sizeof(acc)) {
            /* Flush safe portion if we have OTA data */
            if (state == MP_DATA && ota_started) {
                size_t safe = acc_len + rd - blen - 8;
                if ((int)safe > 0) {
                    err = esp_ota_write(ota_handle, acc, safe);
                    if (err != ESP_OK) goto fail;
                    written += safe;
                    memmove(acc, acc + safe, acc_len - safe);
                    acc_len -= safe;
                }
            }
        }
        memcpy(acc + acc_len, buf, rd);
        acc_len += rd;

        /* State machine */
        bool progress = true;
        while (progress) {
            progress = false;
            if (state == MP_SEEK_BOUNDARY) {
                char *p = memmem(acc, acc_len, boundary, blen);
                if (p) {
                    size_t skip = (p - acc) + blen;
                    memmove(acc, acc + skip, acc_len - skip);
                    acc_len -= skip;
                    state = MP_SEEK_HEADER_END;
                    progress = true;
                }
            } else if (state == MP_SEEK_HEADER_END) {
                /* Look for double CRLF ending headers */
                char *p = memmem(acc, acc_len, "\r\n\r\n", 4);
                if (p) {
                    size_t skip = (p - acc) + 4;
                    memmove(acc, acc + skip, acc_len - skip);
                    acc_len -= skip;
                    state = MP_DATA;
                    ota_started = true;
                    progress = true;
                }
            } else if (state == MP_DATA) {
                /* Check for boundary marker indicating end of file data */
                char end_boundary[80];
                snprintf(end_boundary, sizeof(end_boundary), "\r\n%s", boundary);
                size_t eblen = strlen(end_boundary);
                char *p = memmem(acc, acc_len, end_boundary, eblen);
                if (p) {
                    /* Write data up to end boundary */
                    size_t data_len = p - acc;
                    if (data_len > 0) {
                        err = esp_ota_write(ota_handle, acc, data_len);
                        if (err != ESP_OK) goto fail;
                        written += data_len;
                    }
                    state = MP_DONE;
                    progress = true;
                } else if (acc_len > eblen + 8) {
                    /* Safe to flush everything except potential boundary overlap */
                    size_t safe = acc_len - eblen - 8;
                    err = esp_ota_write(ota_handle, acc, safe);
                    if (err != ESP_OK) goto fail;
                    written += safe;
                    memmove(acc, acc + safe, acc_len - safe);
                    acc_len -= safe;
                }
            }
        }
    }

    if (state != MP_DONE || written == 0) {
        ESP_LOGE(TAG, "Incomplete upload — state=%d written=%zu", state, written);
        goto fail;
    }

    ESP_LOGI(TAG, "Upload complete — %zu bytes written", written);

    err = esp_ota_end(ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_end failed: %d", err);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                            "OTA validation failed — check binary");
        return ESP_OK;
    }

    err = esp_ota_set_boot_partition(update_part);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed: %d", err);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                            "OTA set boot failed");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "OTA success — rebooting to %s", update_part->label);

    httpd_resp_sendstr(req,
        "<html><body style='font-family:system-ui;padding:20px;max-width:400px'>"
        "<h2 style='color:#1976d2'>Firmware Updated!</h2>"
        "<p>Upload successful. The device is rebooting with the new firmware.</p>"
        "<p>Reconnect to <b>Westshore-RID</b> and browse to "
        "<b>http://192.168.4.1</b> to verify.</p>"
        "</body></html>");

    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_restart();
    return ESP_OK;

fail:
    esp_ota_abort(ota_handle);
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                        "OTA upload failed");
    return ESP_OK;
}

/* Current partition info for display in UI */
void ota_get_info(char *running_label, size_t label_len,
                  char *next_label, size_t next_len)
{
    const esp_partition_t *running = esp_ota_get_running_partition();
    const esp_partition_t *next    = esp_ota_get_next_update_partition(NULL);
    strlcpy(running_label, running ? running->label : "unknown", label_len);
    strlcpy(next_label,    next    ? next->label    : "unknown", next_len);
}
