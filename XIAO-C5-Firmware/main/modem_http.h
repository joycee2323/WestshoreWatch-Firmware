#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stddef.h>

/**
 * Native SIM7600 AT HTTP(S) POST.
 *
 * Drives the modem's built-in HTTP(S) stack over the AT channel — no PPP, no
 * lwIP. The whole sequence (HTTPINIT → HTTPPARA → HTTPDATA → body →
 * HTTPACTION → HTTPTERM) runs as ONE transaction holding the cellular_uart
 * lock, so a concurrent GPS poll can never interleave between commands.
 *
 * Assumes the data session is already up: AT+NETOPEN succeeded and the TLS
 * context (sslversion / authmode / enableSNI / ignorelocaltime) is configured.
 * That one-time bring-up lives in modem_manager (modem_http_netup()).
 *
 * PROVEN AT recipe (LE20B04SIM7600G22) — params are case-sensitive lowercase;
 * enableSNI is REQUIRED for the Render/Cloudflare backend.
 */

typedef struct {
    int  http_status;   /* parsed HTTP status (e.g. 200), or -1 if none */
    int  modem_err;     /* modem-side error code >=700 (715=TLS fail, …), else 0 */
    int  resp_len;      /* response body length from +HTTPACTION, or -1 */
} modem_http_result_t;

/**
 * POST a body over HTTPS via the modem's native AT HTTP stack.
 *
 * @param url       full https:// URL
 * @param headers   extra header line(s) for AT+HTTPPARA "USERDATA"
 *                  (e.g. "X-Node-API-Key: <key>"); NULL/empty to omit
 * @param body      request body bytes (JSON)
 * @param body_len  number of body bytes
 * @param out       filled with the parsed +HTTPACTION result (may be NULL)
 *
 * @return ESP_OK   if a 2xx HTTP status was returned
 *         ESP_FAIL on any non-2xx HTTP status or modem-side (7xx) error
 *         ESP_ERR_* on an AT-sequence failure (timeout, no DOWNLOAD prompt…)
 *
 * On every exit the HTTP service is terminated (AT+HTTPTERM) so a failed POST
 * never wedges the next one.
 */
esp_err_t modem_http_post(const char *url, const char *headers,
                          const char *body, int body_len,
                          modem_http_result_t *out);
