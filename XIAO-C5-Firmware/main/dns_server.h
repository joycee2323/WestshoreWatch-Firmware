#pragma once
#include "esp_err.h"

/* ─────────────────────────────────────────────────────────────────────────────
 * Captive-portal DNS server.
 *
 * Binds UDP/53 on all interfaces and answers every type-A query with
 * `redirect_ip_str` (IPv4 dotted-quad), regardless of the queried name.
 * Non-A queries are echoed back with zero answers. This is the minimal
 * hijack pattern required so that iOS / Android / Windows captive-portal
 * probes resolve to the config AP gateway and trigger the "Sign in"
 * browser pop-up.
 *
 * Safe to call after esp_wifi_start() (softAP up, lwIP stack running).
 * ───────────────────────────────────────────────────────────────────────────── */

/**
 * Start the captive-portal DNS server task.
 * @param redirect_ip_str  IPv4 dotted-quad (e.g. "192.168.4.1")
 */
esp_err_t dns_server_start(const char *redirect_ip_str);

/** Request the DNS task to stop. The socket is closed on the next recv loop. */
void dns_server_stop(void);
