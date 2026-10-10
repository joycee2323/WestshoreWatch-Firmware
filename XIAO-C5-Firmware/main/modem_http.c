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

/* HTTPTERM after a budget-limited POST: short, so a failed heartbeat still
 * finishes inside ~budget + 2 s. */
#define HTTPTERM_AFTER_BUDGET_MS 2000

static const modem_http_opts_t DEFAULT_OPTS = {
    .short_ms   = AT_SHORT_MS,
    .data_ok_ms = HTTPDATA_OK_MS,
    .urc_ms     = HTTPACTION_URC_MS,
    .budget_ms  = 0,
};

/* Deadline helper: the wait to use for a step, or 0 when the budget is spent. */
typedef struct {
    TickType_t start;
    uint32_t   budget_ms;
} deadline_t;

static uint32_t elapsed_ms(const deadline_t *d)
{
    return (uint32_t)((xTaskGetTickCount() - d->start) * portTICK_PERIOD_MS);
}

static uint32_t step_ms(const deadline_t *d, uint32_t want)
{
    if (d->budget_ms == 0) return want;
    uint32_t used = elapsed_ms(d);
    if (used >= d->budget_ms) return 0;
    uint32_t left = d->budget_ms - used;
    return want < left ? want : left;
}

esp_err_t modem_http_post(const char *url, const char *headers,
                          const char *body, int body_len,
                          modem_http_result_t *out)
{
    return modem_http_post_opts(url, headers, body, body_len, out, NULL);
}

/* Budget check before a step: give up (to term_out) when nothing is left. */
#define STEP(var, want)                                                     \
    do {                                                                    \
        (var) = step_ms(&dl, (want));                                       \
        if ((var) == 0) {                                                   \
            ESP_LOGW(TAG, "POST budget %lums spent at stage %s",            \
                     (unsigned long)o->budget_ms, res.stage);               \
            res.stage = "budget";                                           \
            ret = ESP_ERR_TIMEOUT;                                          \
            goto term_out;                                                  \
        }                                                                   \
    } while (0)

esp_err_t modem_http_post_opts(const char *url, const char *headers,
                               const char *body, int body_len,
                               modem_http_result_t *out,
                               const modem_http_opts_t *opts)
{
    if (!url || !body || body_len <= 0) return ESP_ERR_INVALID_ARG;

    const modem_http_opts_t *o = opts ? opts : &DEFAULT_OPTS;
    deadline_t dl = { .start = xTaskGetTickCount(), .budget_ms = o->budget_ms };
    uint32_t t = 0;
    modem_http_result_t res = { .http_status = -1, .modem_err = 0, .resp_len = -1,
                                .stage = "init", .elapsed_ms = 0 };
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
    /* HTTPINIT runs before any budget check can jump to term_out, so a spent
     * budget here unlocks and returns directly (nothing to HTTPTERM yet). */
    t = step_ms(&dl, o->short_ms);
    if (cellular_uart_send_at("AT+HTTPINIT", resp, sizeof(resp), t) != ESP_OK) {
        ESP_LOGW(TAG, "HTTPINIT failed — clearing stale session and retrying");
        t = step_ms(&dl, o->short_ms);
        if (t) cellular_uart_send_at("AT+HTTPTERM", resp, sizeof(resp), t);
        t = step_ms(&dl, o->short_ms);
        if (t == 0 || cellular_uart_send_at("AT+HTTPINIT", resp, sizeof(resp), t) != ESP_OK) {
            ESP_LOGE(TAG, "HTTPINIT failed twice — aborting POST");
            cellular_uart_unlock();
            res.elapsed_ms = elapsed_ms(&dl);
            if (out) *out = res;
            return ESP_FAIL;
        }
    }
    /* From here on, every exit must HTTPTERM (label term_out). */

    /* ── HTTPPARA: URL / SSLCFG / CONTENT / USERDATA ───────────────────────── */
    res.stage = "para";
    snprintf(line, sizeof(line), "AT+HTTPPARA=\"URL\",\"%s\"", url);
    STEP(t, o->short_ms);
    if (cellular_uart_send_at(line, resp, sizeof(resp), t) != ESP_OK) {
        ESP_LOGE(TAG, "HTTPPARA URL failed"); goto term_out;
    }

    /* Bind SSL context 0 (configured once at NETOPEN: sslversion/authmode/
     * enableSNI/ignorelocaltime). REQUIRED for the https:// endpoint. */
    STEP(t, o->short_ms);
    if (cellular_uart_send_at("AT+HTTPPARA=\"SSLCFG\",0", resp, sizeof(resp), t) != ESP_OK) {
        ESP_LOGE(TAG, "HTTPPARA SSLCFG failed"); goto term_out;
    }

    STEP(t, o->short_ms);
    if (cellular_uart_send_at("AT+HTTPPARA=\"CONTENT\",\"application/json\"",
                              resp, sizeof(resp), t) != ESP_OK) {
        ESP_LOGE(TAG, "HTTPPARA CONTENT failed"); goto term_out;
    }

    if (headers && headers[0]) {
        snprintf(line, sizeof(line), "AT+HTTPPARA=\"USERDATA\",\"%s\"", headers);
        STEP(t, o->short_ms);
        if (cellular_uart_send_at(line, resp, sizeof(resp), t) != ESP_OK) {
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
    res.stage = "data";
    snprintf(line, sizeof(line), "AT+HTTPDATA=%d,%d", body_len, HTTPDATA_SEND_MS);
    STEP(t, o->short_ms);
    if (cellular_uart_send_expect(line, "DOWNLOAD", "ERROR",
                                  resp, sizeof(resp), t) != ESP_OK) {
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
    res.stage = "data_ok";
    STEP(t, o->data_ok_ms);
    dr = cellular_uart_drain_until("OK", "ERROR", t, &drained);
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
    res.stage = "action";
    STEP(t, o->short_ms);
    if (cellular_uart_send_at("AT+HTTPACTION=1", resp, sizeof(resp), t) != ESP_OK) {
        ESP_LOGE(TAG, "HTTPACTION send failed"); goto term_out;
    }
    res.stage = "urc";
    STEP(t, o->urc_ms);
    if (cellular_uart_collect("+HTTPACTION:", resp, sizeof(resp), t) != ESP_OK) {
        /* No URC at all → transport is dead (not an HTTP-level failure). */
        ESP_LOGE(TAG, "HTTPACTION: no result URC in %lums", (unsigned long)t);
        ret = ESP_ERR_TIMEOUT;
        goto term_out;
    }
    /* The numeric tail (" 1,<status>,<len>\r\n") follows the marker we just
     * matched; read the rest of that line and parse it. */
    res.stage = "parse";
    STEP(t, o->short_ms);
    cellular_uart_collect("\n", tail, sizeof(tail), t);
    if (sscanf(tail, " %d,%d,%d", &method, &status, &dlen) < 2) {
        ESP_LOGE(TAG, "HTTPACTION: unparseable URC tail '%s'", tail);
        goto term_out;
    }

    res.stage = "done";
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
    cellular_uart_send_at("AT+HTTPTERM", resp, sizeof(resp),
                          o->budget_ms ? HTTPTERM_AFTER_BUDGET_MS : AT_SHORT_MS);
    cellular_uart_unlock();
    res.elapsed_ms = elapsed_ms(&dl);
    if (out) *out = res;
    return ret;
}

#undef STEP
