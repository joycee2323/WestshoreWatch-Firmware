#include "modem_http.h"
#include "cellular_uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "MODEM_HTTP";

/* ── Timeouts ─────────────────────────────────────────────────────────────── */
#define AT_SHORT_MS          5000     /* HTTPINIT / HTTPPARA / HTTPTERM */
#define HTTPDATA_SEND_MS     10000    /* <timeout> arg to AT+HTTPDATA (body send) */
#define HTTPDATA_OK_MS       12000    /* wait for OK after writing the body (> SEND_MS) */
#define HTTPDATA_SETTLE_MS   100      /* pause after DOWNLOAD prompt before writing body */
#define HTTPACTION_URC_MS    60000    /* wait for the +HTTPACTION result URC */

#define LINE_BUF_SIZE        320      /* fits URL/USERDATA AT lines */
#define RESP_BUF_SIZE        256

/* Threshold separating modem-side errors from real HTTP statuses in the
 * +HTTPACTION URC.  Per the SIM7600 spec (and proven on LE20B04SIM7600G22),
 * codes >= 700 are modem-side (715 = TLS handshake fail, etc.), not HTTP. */
#define MODEM_ERR_FLOOR      700

/* AT+HTTPACTION=1 → POST */
#define HTTP_METHOD_POST     1

esp_err_t modem_http_post(const char *url, const char *headers,
                          const char *body, int body_len,
                          modem_http_result_t *out)
{
    if (!url || !body || body_len <= 0) return ESP_ERR_INVALID_ARG;

    modem_http_result_t res = { .http_status = -1, .modem_err = 0, .resp_len = -1 };
    char line[LINE_BUF_SIZE];
    char resp[RESP_BUF_SIZE];
    char tail[64] = {0};
    int method = 0, status = -1, dlen = -1;
    int slen = 0, wrote = 0, drained = 0;
    esp_err_t dr = ESP_FAIL;
    esp_err_t ret = ESP_FAIL;

    /* Hold the AT channel for the WHOLE transaction so a GPS poll (or any
     * other AT user) can never slip between HTTPDATA and HTTPACTION. */
    cellular_uart_lock();

    /* ── HTTPINIT ──────────────────────────────────────────────────────────
     * Errors if a prior HTTP service was left started (e.g. a POST that died
     * mid-sequence) — clear it with HTTPTERM and retry once. */
    if (cellular_uart_send_at("AT+HTTPINIT", resp, sizeof(resp), AT_SHORT_MS) != ESP_OK) {
        ESP_LOGW(TAG, "HTTPINIT failed — clearing stale session and retrying");
        cellular_uart_send_at("AT+HTTPTERM", resp, sizeof(resp), AT_SHORT_MS);
        if (cellular_uart_send_at("AT+HTTPINIT", resp, sizeof(resp), AT_SHORT_MS) != ESP_OK) {
            ESP_LOGE(TAG, "HTTPINIT failed twice — aborting POST");
            cellular_uart_unlock();
            if (out) *out = res;
            return ESP_FAIL;
        }
    }
    /* From here on, every exit must HTTPTERM (label term_out). */

    /* ── HTTPPARA: URL / SSLCFG / CONTENT / USERDATA ───────────────────────── */
    snprintf(line, sizeof(line), "AT+HTTPPARA=\"URL\",\"%s\"", url);
    if (cellular_uart_send_at(line, resp, sizeof(resp), AT_SHORT_MS) != ESP_OK) {
        ESP_LOGE(TAG, "HTTPPARA URL failed"); goto term_out;
    }

    /* Bind SSL context 0 (configured once at NETOPEN: sslversion/authmode/
     * enableSNI/ignorelocaltime). REQUIRED for the https:// endpoint. */
    if (cellular_uart_send_at("AT+HTTPPARA=\"SSLCFG\",0", resp, sizeof(resp), AT_SHORT_MS) != ESP_OK) {
        ESP_LOGE(TAG, "HTTPPARA SSLCFG failed"); goto term_out;
    }

    if (cellular_uart_send_at("AT+HTTPPARA=\"CONTENT\",\"application/json\"",
                              resp, sizeof(resp), AT_SHORT_MS) != ESP_OK) {
        ESP_LOGE(TAG, "HTTPPARA CONTENT failed"); goto term_out;
    }

    if (headers && headers[0]) {
        snprintf(line, sizeof(line), "AT+HTTPPARA=\"USERDATA\",\"%s\"", headers);
        if (cellular_uart_send_at(line, resp, sizeof(resp), AT_SHORT_MS) != ESP_OK) {
            ESP_LOGE(TAG, "HTTPPARA USERDATA failed"); goto term_out;
        }
    }

    /* ── HTTPDATA: announce length, wait for DOWNLOAD prompt, push body ──────
     * The modem waits for EXACTLY body_len bytes after DOWNLOAD, then returns
     * a synchronous OK.  We must write ONLY the body bytes — no CR/LF/NUL. */
    slen = (int)strlen(body);
    if (slen != body_len) {
        /* HTTPDATA <size> and the bytes we write MUST agree or the modem waits
         * forever. Both come from body_len, so a mismatch means the body isn't
         * NUL-terminated at body_len (a caller bug) — flag it loudly. */
        ESP_LOGW(TAG, "HTTPDATA byte-count mismatch: HTTPDATA=%d but strlen(body)=%d",
                 body_len, slen);
    }
    snprintf(line, sizeof(line), "AT+HTTPDATA=%d,%d", body_len, HTTPDATA_SEND_MS);
    if (cellular_uart_send_expect(line, "DOWNLOAD", "ERROR",
                                  resp, sizeof(resp), AT_SHORT_MS) != ESP_OK) {
        ESP_LOGE(TAG, "HTTPDATA: no DOWNLOAD prompt"); goto term_out;
    }
    /* Settle: some SIM7600 builds drop the leading payload bytes if data is
     * written the instant the DOWNLOAD prompt is emitted. */
    vTaskDelay(pdMS_TO_TICKS(HTTPDATA_SETTLE_MS));
    wrote = cellular_uart_write_raw((const uint8_t *)body, body_len);
    ESP_LOGI(TAG, "HTTPDATA: wrote %d/%d body bytes (strlen=%d), awaiting OK",
             wrote, body_len, slen);
    /* Post-body OK: some SIM7600 builds ECHO the whole payload back before the
     * OK, so a fixed capture buffer freezes mid-echo and never sees the OK
     * (worked for a 67-byte heartbeat, failed for a 2471-byte detection). Drain
     * with a rolling window that finds OK after any amount of echoed preamble;
     * watch ERROR too so a reject fails fast. drained tells us the echo size. */
    dr = cellular_uart_drain_until("OK", "ERROR", HTTPDATA_OK_MS, &drained);
    if (dr != ESP_OK) {
        ESP_LOGE(TAG, "HTTPDATA: %s after %d-byte body (drained %d bytes)",
                 dr == ESP_FAIL ? "ERROR reply" : "no OK (timeout)",
                 body_len, drained);
        goto term_out;
    }
    ESP_LOGI(TAG, "HTTPDATA: OK (drained %d bytes incl. echo)", drained);

    /* ── HTTPACTION=1 (POST) — OK is immediate; the result arrives later as a
     * +HTTPACTION: 1,<status>,<len> URC.  Do NOT flush between the OK and the
     * URC (cellular_uart_collect doesn't), so a fast URC is never dropped. ── */
    if (cellular_uart_send_at("AT+HTTPACTION=1", resp, sizeof(resp), AT_SHORT_MS) != ESP_OK) {
        ESP_LOGE(TAG, "HTTPACTION send failed"); goto term_out;
    }
    if (cellular_uart_collect("+HTTPACTION:", resp, sizeof(resp), HTTPACTION_URC_MS) != ESP_OK) {
        /* No URC at all → transport is dead (not an HTTP-level failure). */
        ESP_LOGE(TAG, "HTTPACTION: no result URC in %dms", HTTPACTION_URC_MS);
        ret = ESP_ERR_TIMEOUT;
        goto term_out;
    }
    /* The numeric tail (" 1,<status>,<len>\r\n") follows the marker we just
     * matched; read the rest of that line and parse it. */
    cellular_uart_collect("\n", tail, sizeof(tail), AT_SHORT_MS);
    if (sscanf(tail, " %d,%d,%d", &method, &status, &dlen) < 2) {
        ESP_LOGE(TAG, "HTTPACTION: unparseable URC tail '%s'", tail);
        goto term_out;
    }

    if (status >= MODEM_ERR_FLOOR) {
        res.modem_err = status;          /* 7xx — modem/TLS-side, not HTTP */
        ESP_LOGE(TAG, "HTTPACTION modem error %d (e.g. 715=TLS fail)", status);
    } else {
        res.http_status = status;
        res.resp_len    = dlen;
        ret = (status >= 200 && status < 300) ? ESP_OK : ESP_FAIL;
        ESP_LOGI(TAG, "HTTPACTION → HTTP %d (%d bytes)", status, dlen);
    }

term_out:
    cellular_uart_send_at("AT+HTTPTERM", resp, sizeof(resp), AT_SHORT_MS);
    cellular_uart_unlock();
    if (out) *out = res;
    return ret;
}
