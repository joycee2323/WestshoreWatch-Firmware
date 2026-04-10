#include "config_server.h"
#include "nvs_config.h"
#include "ota_handler.h"
#include "led.h"
#include "esp_http_server.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_mac.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

static const char *TAG = "CFG_SRV";



/* Flash message shown at top of page after save */
static char s_flash[64] = {0};

/* ─────────────────────────────────────────────────────────────────────────────
 * URL decode helper
 * ───────────────────────────────────────────────────────────────────────────── */
static void url_decode(char *dst, const char *src, size_t max)
{
    size_t i = 0;
    while (*src && i < max - 1) {
        if (*src == '+') {
            dst[i++] = ' '; src++;
        } else if (*src == '%' && src[1] && src[2]) {
            char hex[3] = {src[1], src[2], 0};
            dst[i++] = (char)strtol(hex, NULL, 16);
            src += 3;
        } else {
            dst[i++] = *src++;
        }
    }
    dst[i] = '\0';
}

static bool form_get_field(const char *body, const char *key,
                            char *out, size_t out_len)
{
    size_t klen = strlen(key);
    const char *p = body;
    while (p && *p) {
        if (strncmp(p, key, klen) == 0 && p[klen] == '=') {
            p += klen + 1;
            const char *end = strchr(p, '&');
            size_t vlen = end ? (size_t)(end - p) : strlen(p);
            char raw[256] = {0};
            if (vlen >= sizeof(raw)) vlen = sizeof(raw) - 1;
            memcpy(raw, p, vlen);
            url_decode(out, raw, out_len);
            return true;
        }
        p = strchr(p, '&');
        if (p) p++;
    }
    out[0] = '\0';
    return false;
}

static bool form_has_field(const char *body, const char *key)
{
    size_t klen = strlen(key);
    const char *p = body;
    while (p && *p) {
        if (strncmp(p, key, klen) == 0 &&
            (p[klen] == '=' || p[klen] == '&' || p[klen] == '\0'))
            return true;
        p = strchr(p, '&');
        if (p) p++;
    }
    return false;
}

/* ─────────────────────────────────────────────────────────────────────────────
 * HTML helpers
 * ───────────────────────────────────────────────────────────────────────────── */
#define SEL(cond)  ((cond) ? "selected" : "")
#define CHK(cond)  ((cond) ? "checked"  : "")

static void build_channel_options(char *buf, size_t len, uint8_t selected)
{
    int n = 0;
    for (int ch = 1; ch <= 13; ch++) {
        n += snprintf(buf + n, len - n,
                      "<option value=\"%d\"%s>%d</option>",
                      ch, ch == selected ? " selected" : "", ch);
        if ((size_t)n >= len - 1) break;
    }
}

static void build_blacklist_html(char *buf, size_t len, const wsd_config_t *cfg)
{
    int n = 0;
    n += snprintf(buf + n, len - n, "<div id=\"bl-list\">");
    for (int i = 0; i < WSD_BLACKLIST_MAX; i++) {
        const char *val = (i < cfg->blacklist_count) ? cfg->blacklist[i] : "";
        n += snprintf(buf + n, len - n,
            "<div class=\"bl-row\">"
            "<input type=\"text\" name=\"bl_%d\" value=\"%s\" "
            "placeholder=\"Drone serial number\" maxlength=\"31\">"
            "</div>", i, val);
        if ((size_t)n >= len - 1) break;
    }
    n += snprintf(buf + n, len - n, "</div>");
}

/* ─────────────────────────────────────────────────────────────────────────────
 * HTML page CSS + structure
 * ───────────────────────────────────────────────────────────────────────────── */
static const char HTML_HEAD[] =
"<!DOCTYPE html><html lang=\"en\"><head>"
"<meta charset=\"UTF-8\">"
"<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
"<title>AirAware X1 — Configuration</title>"
"<style>"
"*{box-sizing:border-box;margin:0;padding:0}"
"body{font-family:system-ui,sans-serif;background:#f4f4f4;color:#222}"
".hdr{background:#0b1118;color:#fff;padding:14px 20px;display:flex;align-items:center;gap:14px}"
".hdr-text h1{font-size:1.25rem;color:#00d4ff;font-weight:700;letter-spacing:.3px}"
".hdr-text h2{font-size:.75rem;color:#ccc;font-weight:400;margin-top:1px}"
".hdr-text small{color:#888;font-size:.72rem}"
".nav{display:flex;background:#111c26;overflow-x:auto}"
".nav button{background:none;border:none;border-bottom:3px solid transparent;"
"color:#aaa;padding:12px 16px;cursor:pointer;font-size:.88rem;white-space:nowrap}"
".nav button.on{border-bottom-color:#00d4ff;color:#fff}"
".body{padding:18px;max-width:560px}"
".sec{margin-bottom:22px}"
".sec h3{font-size:.78rem;font-weight:700;color:#555;letter-spacing:.8px;"
"text-transform:uppercase;border-bottom:1px solid #ddd;padding-bottom:5px;margin-bottom:14px}"
".f{margin-bottom:16px}"
".f label{display:block;font-weight:500;font-size:.88rem;margin-bottom:3px}"
".f small{display:block;color:#666;font-size:.78rem;margin-top:3px;line-height:1.4}"
".f input[type=text],.f input[type=password],.f input[type=number],.f select{"
"width:100%;padding:7px 9px;border:1px solid #ccc;border-radius:4px;font-size:.88rem}"
".ck{display:flex;gap:10px;align-items:flex-start}"
".ck input{margin-top:2px;width:16px;height:16px;flex-shrink:0;cursor:pointer}"
".bl-row{display:flex;margin-bottom:6px}"
".bl-row input{flex:1;padding:6px 8px;border:1px solid #ccc;border-radius:4px;font-size:.83rem}"
".acts{display:flex;gap:10px;flex-wrap:wrap;padding-top:18px;border-top:1px solid #ddd;margin-top:6px}"
".btn{padding:9px 18px;border:none;border-radius:4px;cursor:pointer;font-size:.88rem;font-weight:500}"
".bp{background:#00a0bf;color:#fff}"
".bd{background:#c62828;color:#fff}"
".bo{background:#fff;border:1px solid #bbb;color:#333}"
".tab{display:none}.tab.on{display:block}"
".alert{background:#e8f5e9;border:1px solid #a5d6a7;color:#1b5e20;"
"padding:10px 14px;border-radius:4px;margin-bottom:14px;font-size:.88rem}"
".info-tbl td{padding:5px 0;font-size:.88rem}"
".info-tbl td:first-child{color:#666;width:130px}"
".aw-status{background:#e3f2fd;border:1px solid #90caf9;border-radius:4px;"
"padding:10px 14px;margin-bottom:14px;font-size:.84rem;color:#0d47a1}"
"</style></head><body>";

static const char HTML_SCRIPT[] =
"<script>"
"function tab(b,id){"
"document.querySelectorAll('.nav button').forEach(x=>x.classList.remove('on'));"
"document.querySelectorAll('.tab').forEach(x=>x.classList.remove('on'));"
"b.classList.add('on');document.getElementById(id).classList.add('on');"
"localStorage.setItem('wt',id)}"
"window.onload=function(){"
"var t=localStorage.getItem('wt')||'airaware';"
"var b=document.querySelector('[data-tab='+t+']');"
"if(b){tab(b,t);}else{tab(document.querySelector('.nav button'),'airaware');}}"
"</script></body></html>";

/* ─────────────────────────────────────────────────────────────────────────────
 * GET / — main config page
 * ───────────────────────────────────────────────────────────────────────────── */
static esp_err_t handler_root_get(httpd_req_t *req)
{
    wsd_config_t *c = &g_config;
    char mac_str[18];
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);
    snprintf(mac_str, sizeof(mac_str), "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    /* Build SSID for display */
    char ap_ssid[32];
    snprintf(ap_ssid, sizeof(ap_ssid), "AirAware-X1-%02X%02X", mac[4], mac[5]);

    httpd_resp_set_type(req, "text/html");
    httpd_resp_send_chunk(req, HTML_HEAD, HTTPD_RESP_USE_STRLEN);

    /* Header */
    char buf[4096];
    httpd_resp_send_chunk(req,
        "<div class=\"hdr\">"
        "<div class=\"hdr-text\">"
        "<h1>AIRAWARE X1</h1>"
        "<h2>Remote ID Sensor Node</h2>",
        HTTPD_RESP_USE_STRLEN);
    snprintf(buf, sizeof(buf),
        "<small>%s &nbsp;|&nbsp; Westshore Drone Services</small>"
        "</div></div>", mac_str);
    httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

    /* Nav — AirAware tab is first (most important for setup) */
    httpd_resp_send_chunk(req,
        "<div class=\"nav\">"
        "<button class=\"on\" data-tab=\"airaware\" onclick=\"tab(this,'airaware')\">AirAware</button>"
        "<button data-tab=\"general\" onclick=\"tab(this,'general')\">General</button>"
        "<button data-tab=\"reception\" onclick=\"tab(this,'reception')\">Reception</button>"
        "<button data-tab=\"uart\" onclick=\"tab(this,'uart')\">UART</button>"
        "<button data-tab=\"system\" onclick=\"tab(this,'system')\">System</button>"
        "<button data-tab=\"firmware\" onclick=\"tab(this,'firmware')\">Firmware</button>"
        "</div>", HTTPD_RESP_USE_STRLEN);

    /* Form wraps all tabs except Firmware */
    httpd_resp_send_chunk(req,
        "<div class=\"body\"><form id=\"settings-form\" method=\"POST\" action=\"/save\">",
        HTTPD_RESP_USE_STRLEN);

    /* Flash message */
    if (s_flash[0]) {
        snprintf(buf, sizeof(buf), "<div class=\"alert\">%s</div>", s_flash);
        httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);
        s_flash[0] = '\0';
    }

    /* ── AIRAWARE TAB ─────────────────────────────────────────────────────── */
    httpd_resp_send_chunk(req, "<div id=\"airaware\" class=\"tab on\">", HTTPD_RESP_USE_STRLEN);

    /* Upload status banner */
    if (c->upload_en && c->wifi_ssid[0] && c->server_url[0] && c->api_key[0]) {
        snprintf(buf, sizeof(buf),
            "<div class=\"aw-status\">"
            "&#9679; Cloud upload configured &mdash; Node: <b>%s</b> &mdash; "
            "Server: <b>%s</b>"
            "</div>",
            c->node_name[0] ? c->node_name : "(unnamed)",
            c->server_url);
        httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);
    }

    /* Node identity */
    httpd_resp_send_chunk(req, "<div class=\"sec\"><h3>Node Identity</h3>", HTTPD_RESP_USE_STRLEN);

    httpd_resp_send_chunk(req,
        "<div class=\"f\"><label>Node Name</label>"
        "<input type=\"text\" name=\"node_name\" value=\"",
        HTTPD_RESP_USE_STRLEN);
    httpd_resp_send_chunk(req, c->node_name, HTTPD_RESP_USE_STRLEN);
    httpd_resp_send_chunk(req,
        "\" maxlength=\"63\" placeholder=\"e.g. Zone 1 - North Gate\">"
        "<small>Display name shown in the AirAware dashboard.</small>"
        "</div>", HTTPD_RESP_USE_STRLEN);

    httpd_resp_send_chunk(req, "</div>", HTTPD_RESP_USE_STRLEN); /* /sec */

    /* WiFi uplink */
    httpd_resp_send_chunk(req, "<div class=\"sec\"><h3>WiFi Uplink Network</h3>", HTTPD_RESP_USE_STRLEN);
    httpd_resp_send_chunk(req,
        "<div class=\"f\"><small>Enter the WiFi network the X1 should connect to "
        "for uploading detections. This is separate from the AirAware-X1 config network.</small></div>",
        HTTPD_RESP_USE_STRLEN);

    httpd_resp_send_chunk(req,
        "<div class=\"f\"><label>WiFi Network (SSID)</label>"
        "<input type=\"text\" name=\"aw_ssid\" value=\"",
        HTTPD_RESP_USE_STRLEN);
    httpd_resp_send_chunk(req, c->wifi_ssid, HTTPD_RESP_USE_STRLEN);
    httpd_resp_send_chunk(req,
        "\" maxlength=\"63\" placeholder=\"Your site WiFi or phone hotspot name\">"
        "</div>", HTTPD_RESP_USE_STRLEN);

    httpd_resp_send_chunk(req,
        "<div class=\"f\"><label>WiFi Password</label>"
        "<input type=\"password\" name=\"aw_pass\" value=\"",
        HTTPD_RESP_USE_STRLEN);
    httpd_resp_send_chunk(req, c->wifi_pass, HTTPD_RESP_USE_STRLEN);
    httpd_resp_send_chunk(req,
        "\" maxlength=\"63\" placeholder=\"Leave blank for open network\">"
        "</div>", HTTPD_RESP_USE_STRLEN);

    httpd_resp_send_chunk(req, "</div>", HTTPD_RESP_USE_STRLEN); /* /sec */

    /* AirAware server */
    httpd_resp_send_chunk(req, "<div class=\"sec\"><h3>AirAware Server</h3>", HTTPD_RESP_USE_STRLEN);

    httpd_resp_send_chunk(req,
        "<div class=\"f\"><label>Server URL</label>"
        "<input type=\"text\" name=\"aw_url\" value=\"",
        HTTPD_RESP_USE_STRLEN);
    httpd_resp_send_chunk(req, c->server_url, HTTPD_RESP_USE_STRLEN);
    httpd_resp_send_chunk(req,
        "\" maxlength=\"127\" readonly style=\"background:#f0f0f0;color:#888;cursor:not-allowed\">"
        "<small>Provisioned at setup. Contact your administrator to change.</small>"
        "</div>", HTTPD_RESP_USE_STRLEN);

    httpd_resp_send_chunk(req,
        "<div class=\"f\"><label>Node API Key</label>"
        "<input type=\"text\" name=\"aw_key\" value=\"",
        HTTPD_RESP_USE_STRLEN);
    httpd_resp_send_chunk(req, c->api_key, HTTPD_RESP_USE_STRLEN);
    httpd_resp_send_chunk(req,
        "\" maxlength=\"71\" readonly style=\"background:#f0f0f0;color:#888;cursor:not-allowed\">"
        "<small>Provisioned at setup. Contact your administrator to change.</small>"
        "</div>", HTTPD_RESP_USE_STRLEN);

    snprintf(buf, sizeof(buf),
        "<div class=\"f\"><div class=\"ck\">"
        "<input type=\"checkbox\" name=\"aw_en\" value=\"1\" %s>"
        "<div><label>Enable cloud upload</label>"
        "<small>Upload detections to AirAware every 10 seconds. "
        "Requires WiFi network and API key above.</small>"
        "</div></div></div>", CHK(c->upload_en));
    httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

    httpd_resp_send_chunk(req, "</div>", HTTPD_RESP_USE_STRLEN); /* /sec */

    /* Node location */
    httpd_resp_send_chunk(req, "<div class=\"sec\"><h3>Node Location</h3>", HTTPD_RESP_USE_STRLEN);
    httpd_resp_send_chunk(req,
        "<div class=\"f\"><small>Set the physical location of this node. "
        "Used to show the node pin on the AirAware live map.</small></div>",
        HTTPD_RESP_USE_STRLEN);

    snprintf(buf, sizeof(buf),
        "<div class=\"f\"><label>Latitude</label>"
        "<input type=\"text\" id=\"lat_field\" name=\"node_lat\" value=\"%.6f\" "
        "placeholder=\"e.g. 41.881832\"></div>"
        "<div class=\"f\"><label>Longitude</label>"
        "<input type=\"text\" id=\"lon_field\" name=\"node_lon\" value=\"%.6f\" "
        "placeholder=\"e.g. -87.623177\"></div>",
        c->node_lat, c->node_lon);
    httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

    httpd_resp_send_chunk(req,
        "<div class=\"f\">"
        "<small>Tip: Open Google Maps, long-press your location, and copy the coordinates shown at the top.</small>"
        "</div>",
        HTTPD_RESP_USE_STRLEN);

    httpd_resp_send_chunk(req, "</div>", HTTPD_RESP_USE_STRLEN); /* /sec */
    httpd_resp_send_chunk(req, "</div>", HTTPD_RESP_USE_STRLEN); /* /airaware tab */

    /* ── GENERAL TAB ─────────────────────────────────────────────────────── */
    httpd_resp_send_chunk(req, "<div id=\"general\" class=\"tab\">", HTTPD_RESP_USE_STRLEN);
    httpd_resp_send_chunk(req, "<div class=\"sec\"><h3>Operation</h3>", HTTPD_RESP_USE_STRLEN);

    snprintf(buf, sizeof(buf),
        "<div class=\"f\"><label>Mode</label>"
        "<select name=\"mode\">"
        "<option value=\"0\" %s>Wireless relay</option>"
        "<option value=\"1\" %s>UART receiver</option>"
        "</select>"
        "<small>Wireless relay: receive Remote ID and rebroadcast over BLE. "
        "UART: output JSON telemetry only.</small></div>",
        SEL(c->mode == WSD_MODE_RELAY), SEL(c->mode == WSD_MODE_UART));
    httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

    snprintf(buf, sizeof(buf),
        "<div class=\"f\"><div class=\"ck\">"
        "<input type=\"checkbox\" name=\"ping_en\" value=\"1\" %s>"
        "<div><label>Relay ping</label>"
        "<small>Broadcast bridge identity every 10s so apps show the bridge icon.</small>"
        "</div></div></div>", CHK(c->relay_ping_en));
    httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

    httpd_resp_send_chunk(req,
        "<div class=\"f\"><label>Relay ping label</label>"
        "<input type=\"text\" name=\"ping_label\" value=\"",
        HTTPD_RESP_USE_STRLEN);
    httpd_resp_send_chunk(req, c->relay_ping_label, HTTPD_RESP_USE_STRLEN);
    httpd_resp_send_chunk(req,
        "\" maxlength=\"31\">"
        "<small>Identity string for relay ping. Default: DroneScout Bridge</small>"
        "</div>", HTTPD_RESP_USE_STRLEN);

    snprintf(buf, sizeof(buf),
        "<div class=\"f\"><div class=\"ck\">"
        "<input type=\"checkbox\" name=\"self_id_ovr\" value=\"1\" %s>"
        "<div><label>Insert self ID message</label>"
        "<small>Override Self-ID in relayed packets to indicate retransmission.</small>"
        "</div></div></div>", CHK(c->self_id_override));
    httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

    snprintf(buf, sizeof(buf),
        "<div class=\"f\"><div class=\"ck\">"
        "<input type=\"checkbox\" name=\"ble_en\" value=\"1\" %s>"
        "<div><label>Bluetooth legacy detection</label>"
        "<small>Also scan for Remote ID on BLE legacy (BT4) advertisements.</small>"
        "</div></div></div>", CHK(c->ble_legacy_en));
    httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

    snprintf(buf, sizeof(buf),
        "<div class=\"f\"><div class=\"ck\">"
        "<input type=\"checkbox\" name=\"led_en\" value=\"1\" %s>"
        "<div><label>Flash LED on detection</label>"
        "<small>LED flashes while a drone is being tracked.</small>"
        "</div></div></div>", CHK(c->flash_led_en));
    httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

    httpd_resp_send_chunk(req, "</div>", HTTPD_RESP_USE_STRLEN); /* /sec */

    /* Blacklist */
    httpd_resp_send_chunk(req,
        "<div class=\"sec\"><h3>Drone Blacklist</h3>"
        "<div class=\"f\"><small>Serial numbers to suppress from relay. Up to 10 entries.</small></div>",
        HTTPD_RESP_USE_STRLEN);
    char bl_buf[2048];
    build_blacklist_html(bl_buf, sizeof(bl_buf), c);
    httpd_resp_send_chunk(req, bl_buf, HTTPD_RESP_USE_STRLEN);
    httpd_resp_send_chunk(req, "</div>", HTTPD_RESP_USE_STRLEN); /* /sec */
    httpd_resp_send_chunk(req, "</div>", HTTPD_RESP_USE_STRLEN); /* /general tab */

    /* ── RECEPTION TAB ───────────────────────────────────────────────────── */
    httpd_resp_send_chunk(req, "<div id=\"reception\" class=\"tab\">", HTTPD_RESP_USE_STRLEN);
    httpd_resp_send_chunk(req, "<div class=\"sec\"><h3>Wi-Fi Scanning</h3>", HTTPD_RESP_USE_STRLEN);

    snprintf(buf, sizeof(buf),
        "<div class=\"f\"><label>Scanning strategy</label>"
        "<select name=\"wifi_strat\">"
        "<option value=\"0\" %s>Balanced</option>"
        "<option value=\"1\" %s>Follow (track only)</option>"
        "<option value=\"2\" %s>Fixed channel</option>"
        "</select><small>Balanced recommended.</small></div>",
        SEL(c->wifi_strategy == WSD_WIFI_STRAT_BALANCED),
        SEL(c->wifi_strategy == WSD_WIFI_STRAT_FOLLOW),
        SEL(c->wifi_strategy == WSD_WIFI_STRAT_FIXED));
    httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

    char ch_opts[512];
    build_channel_options(ch_opts, sizeof(ch_opts), c->ch_2g_start);
    httpd_resp_send_chunk(req,
        "<div class=\"f\"><label>2.4 GHz channel start</label>"
        "<select name=\"ch_start\">", HTTPD_RESP_USE_STRLEN);
    httpd_resp_send_chunk(req, ch_opts, HTTPD_RESP_USE_STRLEN);
    httpd_resp_send_chunk(req, "</select><small>Default: 1</small></div>", HTTPD_RESP_USE_STRLEN);

    build_channel_options(ch_opts, sizeof(ch_opts), c->ch_2g_stop);
    httpd_resp_send_chunk(req,
        "<div class=\"f\"><label>2.4 GHz channel stop</label>"
        "<select name=\"ch_stop\">", HTTPD_RESP_USE_STRLEN);
    httpd_resp_send_chunk(req, ch_opts, HTTPD_RESP_USE_STRLEN);
    httpd_resp_send_chunk(req, "</select><small>Default: 13</small></div>", HTTPD_RESP_USE_STRLEN);

    httpd_resp_send_chunk(req, "</div>", HTTPD_RESP_USE_STRLEN); /* /sec */

    httpd_resp_send_chunk(req, "<div class=\"sec\"><h3>Timeouts</h3>", HTTPD_RESP_USE_STRLEN);
    snprintf(buf, sizeof(buf),
        "<div class=\"f\"><label>Landing timeout (seconds)</label>"
        "<input type=\"number\" name=\"land_tout\" value=\"%d\" min=\"1\" max=\"60\">"
        "<small>Stop relay after drone lands. Default: 5</small></div>",
        c->land_timeout_s);
    httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

    snprintf(buf, sizeof(buf),
        "<div class=\"f\"><label>Signal loss timeout (seconds)</label>"
        "<input type=\"number\" name=\"silent_tout\" value=\"%d\" min=\"5\" max=\"120\">"
        "<small>Stop relay after signal loss. Default: 15</small></div>",
        c->silent_timeout_s);
    httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);
    httpd_resp_send_chunk(req, "</div></div>", HTTPD_RESP_USE_STRLEN); /* /sec /reception */

    /* ── UART TAB ─────────────────────────────────────────────────────────── */
    httpd_resp_send_chunk(req, "<div id=\"uart\" class=\"tab\">", HTTPD_RESP_USE_STRLEN);
    httpd_resp_send_chunk(req, "<div class=\"sec\"><h3>Serial Output</h3>", HTTPD_RESP_USE_STRLEN);
    snprintf(buf, sizeof(buf),
        "<div class=\"f\"><label>Baud rate</label>"
        "<select name=\"uart_baud\">"
        "<option value=\"9600\" %s>9600</option>"
        "<option value=\"19200\" %s>19200</option>"
        "<option value=\"38400\" %s>38400</option>"
        "<option value=\"57600\" %s>57600</option>"
        "<option value=\"115200\" %s>115200</option>"
        "</select><small>JSON output on GPIO16 (TX). Default: 115200</small></div>",
        SEL(c->uart_baud == 9600), SEL(c->uart_baud == 19200),
        SEL(c->uart_baud == 38400), SEL(c->uart_baud == 57600),
        SEL(c->uart_baud == 115200));
    httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);
    httpd_resp_send_chunk(req, "</div></div>", HTTPD_RESP_USE_STRLEN); /* /sec /uart */

    /* ── SYSTEM TAB ───────────────────────────────────────────────────────── */
    httpd_resp_send_chunk(req, "<div id=\"system\" class=\"tab\">", HTTPD_RESP_USE_STRLEN);
    httpd_resp_send_chunk(req, "<div class=\"sec\"><h3>Device Information</h3>", HTTPD_RESP_USE_STRLEN);
    snprintf(buf, sizeof(buf),
        "<table class=\"info-tbl\">"
        "<tr><td>Firmware</td><td>v1.1 AirAware</td></tr>"
        "<tr><td>MAC Address</td><td>%s</td></tr>"
        "<tr><td>IDF Version</td><td>%s</td></tr>"
        "<tr><td>Free heap</td><td>%lu bytes</td></tr>"
        "<tr><td>Config AP</td><td>%s</td></tr>"
        "<tr><td>AP Password</td><td>%s</td></tr>"
        "</table>",
        mac_str, esp_get_idf_version(),
        (unsigned long)esp_get_free_heap_size(),
        ap_ssid,
        CFG_AP_PASS[0] ? CFG_AP_PASS : "(open)");
    httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);
    httpd_resp_send_chunk(req, "</div></div>", HTTPD_RESP_USE_STRLEN); /* /sec /system */

    /* ── FIRMWARE TAB ────────────────────────────────────────────────────── */
    {
        char run_label[16], next_label[16];
        ota_get_info(run_label, sizeof(run_label), next_label, sizeof(next_label));
        httpd_resp_send_chunk(req, "<div id=\"firmware\" class=\"tab\">", HTTPD_RESP_USE_STRLEN);
        httpd_resp_send_chunk(req, "<div class=\"sec\"><h3>Firmware Update</h3>", HTTPD_RESP_USE_STRLEN);
        snprintf(buf, sizeof(buf),
            "<table class=\"info-tbl\" style=\"margin-bottom:16px\">"
            "<tr><td>Running slot</td><td>%s</td></tr>"
            "<tr><td>Update slot</td><td>%s</td></tr>"
            "</table>", run_label, next_label);
        httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);
        httpd_resp_send_chunk(req,
            "<div class=\"f\"><small>Select a <b>.bin</b> firmware file. "
            "Device will write to inactive OTA slot and reboot.</small></div>"
            "</div>", HTTPD_RESP_USE_STRLEN); /* /sec */

        httpd_resp_send_chunk(req,
            "<form method=\"POST\" action=\"/ota\" enctype=\"multipart/form-data\">"
            "<div class=\"f\">"
            "<input type=\"file\" name=\"firmware\" accept=\".bin\" "
            "style=\"width:100%;padding:7px;border:1px solid #ccc;border-radius:4px\">"
            "</div>"
            "<div class=\"acts\">"
            "<button type=\"submit\" class=\"btn bp\" "
            "onclick=\"return confirm('Upload firmware and reboot?')\">"
            "Upload &amp; Update</button>"
            "</div></form>",
            HTTPD_RESP_USE_STRLEN);
        httpd_resp_send_chunk(req, "</div>", HTTPD_RESP_USE_STRLEN); /* /firmware tab */
    }

    /* Close form + body */
    httpd_resp_send_chunk(req, "</form></div>", HTTPD_RESP_USE_STRLEN);

    /* Actions bar */
    httpd_resp_send_chunk(req,
        "<div class=\"body\" style=\"padding-top:0\">"
        "<div class=\"acts\">"
        "<button type=\"button\" class=\"btn bp\" "
        "onclick=\"document.getElementById('settings-form').submit()\">Save settings</button>"
        "<button type=\"button\" class=\"btn bo\" "
        "onclick=\"if(confirm('Reset to factory defaults?'))location='/factory'\">Factory defaults</button>"
        "<button type=\"button\" class=\"btn bd\" "
        "onclick=\"if(confirm('Reboot device?'))location='/reboot'\">Reboot</button>"
        "</div></div>",
        HTTPD_RESP_USE_STRLEN);

    httpd_resp_send_chunk(req, HTML_SCRIPT, HTTPD_RESP_USE_STRLEN);
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

/* ─────────────────────────────────────────────────────────────────────────────
 * POST /save
 * ───────────────────────────────────────────────────────────────────────────── */
static esp_err_t handler_save_post(httpd_req_t *req)
{
    int total = req->content_len;
    if (total <= 0 || total > 4096) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Bad request");
        return ESP_OK;
    }
    char *body = malloc(total + 1);
    if (!body) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OOM");
        return ESP_OK;
    }
    int received = httpd_req_recv(req, body, total);
    if (received <= 0) {
        free(body);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Read error");
        return ESP_OK;
    }
    body[received] = '\0';

    wsd_config_t *c = &g_config;
    char val[128];

    /* General */
    if (form_get_field(body, "mode", val, sizeof(val)))
        c->mode = (wsd_mode_t)atoi(val);
    c->relay_ping_en    = form_has_field(body, "ping_en");
    c->self_id_override = form_has_field(body, "self_id_ovr");
    c->ble_legacy_en    = form_has_field(body, "ble_en");
    c->flash_led_en     = form_has_field(body, "led_en");
    if (form_get_field(body, "ping_label", val, sizeof(val)) && val[0])
        strlcpy(c->relay_ping_label, val, sizeof(c->relay_ping_label));

    /* Reception */
    if (form_get_field(body, "wifi_strat", val, sizeof(val)))
        c->wifi_strategy = (wsd_wifi_strategy_t)atoi(val);
    if (form_get_field(body, "ch_start", val, sizeof(val))) {
        int v = atoi(val);
        if (v >= 1 && v <= 13) c->ch_2g_start = v;
    }
    if (form_get_field(body, "ch_stop", val, sizeof(val))) {
        int v = atoi(val);
        if (v >= 1 && v <= 13) c->ch_2g_stop = v;
    }
    if (form_get_field(body, "land_tout", val, sizeof(val))) {
        int v = atoi(val);
        if (v >= 1 && v <= 60) c->land_timeout_s = v;
    }
    if (form_get_field(body, "silent_tout", val, sizeof(val))) {
        int v = atoi(val);
        if (v >= 5 && v <= 120) c->silent_timeout_s = v;
    }

    /* UART */
    if (form_get_field(body, "uart_baud", val, sizeof(val))) {
        uint32_t b = atoi(val);
        if (b == 9600 || b == 19200 || b == 38400 ||
            b == 57600 || b == 115200)
            c->uart_baud = b;
    }

    /* Blacklist */
    c->blacklist_count = 0;
    for (int i = 0; i < WSD_BLACKLIST_MAX; i++) {
        char key[8];
        snprintf(key, sizeof(key), "bl_%d", i);
        char serial[WSD_SERIAL_MAX_LEN];
        if (form_get_field(body, key, serial, sizeof(serial)) && serial[0]) {
            strlcpy(c->blacklist[c->blacklist_count], serial, WSD_SERIAL_MAX_LEN);
            c->blacklist_count++;
        }
    }

    /* AirAware fields */
    if (form_get_field(body, "node_name", val, sizeof(val)))
        strlcpy(c->node_name, val, sizeof(c->node_name));
    if (form_get_field(body, "aw_ssid", val, sizeof(val)))
        strlcpy(c->wifi_ssid, val, sizeof(c->wifi_ssid));
    if (form_get_field(body, "aw_pass", val, sizeof(val)))
        strlcpy(c->wifi_pass, val, sizeof(c->wifi_pass));
    if (form_get_field(body, "aw_url", val, sizeof(val))) {
        /* Strip trailing slash */
        size_t len = strlen(val);
        if (len > 0 && val[len-1] == '/') val[len-1] = '\0';
        strlcpy(c->server_url, val, sizeof(c->server_url));
    }
    if (form_get_field(body, "aw_key", val, sizeof(val)))
        strlcpy(c->api_key, val, sizeof(c->api_key));
    c->upload_en = form_has_field(body, "aw_en");

    /* Node location */
    if (form_get_field(body, "node_lat", val, sizeof(val)) && val[0])
        c->node_lat = atof(val);
    if (form_get_field(body, "node_lon", val, sizeof(val)) && val[0])
        c->node_lon = atof(val);

    free(body);

    wsd_config_save(c);
    strlcpy(s_flash, "Settings saved. Reboot to apply changes.", sizeof(s_flash));

    httpd_resp_set_status(req, "303 See Other");
    httpd_resp_set_hdr(req, "Location", "/");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

/* ─────────────────────────────────────────────────────────────────────────────
 * GET /reboot
 * ───────────────────────────────────────────────────────────────────────────── */
static esp_err_t handler_reboot(httpd_req_t *req)
{
    httpd_resp_sendstr(req,
        "<html><body style='font-family:system-ui;background:#0b1118;color:#d8eaf4;padding:20px'>"
        "<h2 style='color:#00d4ff'>Rebooting...</h2>"
        "<p>Reconnect to the AirAware-X1 WiFi network to reconfigure.</p>"
        "</body></html>");
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
    return ESP_OK;
}

/* ─────────────────────────────────────────────────────────────────────────────
 * GET /factory
 * ───────────────────────────────────────────────────────────────────────────── */
static esp_err_t handler_factory(httpd_req_t *req)
{
    wsd_config_defaults(&g_config);
    wsd_config_save(&g_config);
    strlcpy(s_flash, "Factory defaults restored.", sizeof(s_flash));
    httpd_resp_set_status(req, "303 See Other");
    httpd_resp_set_hdr(req, "Location", "/");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

/* ─────────────────────────────────────────────────────────────────────────────
 * Start HTTP server only (WiFi already up from wifi_scanner_start)
 * Non-blocking — HTTP server runs in its own tasks
 * ───────────────────────────────────────────────────────────────────────────── */
esp_err_t config_server_start_http(void)
{
    ESP_LOGI(TAG, "Starting HTTP config server on http://%s", CFG_AP_IP);

    httpd_config_t hcfg = HTTPD_DEFAULT_CONFIG();
    hcfg.stack_size        = 12288;
    hcfg.max_uri_handlers  = 8;
    hcfg.recv_wait_timeout = 30;
    hcfg.send_wait_timeout = 30;

    httpd_handle_t server = NULL;
    esp_err_t err = httpd_start(&server, &hcfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "HTTP server start failed: %d", err);
        return err;
    }

    httpd_uri_t uris[] = {
        { .uri = "/",        .method = HTTP_GET,  .handler = handler_root_get  },
        { .uri = "/save",    .method = HTTP_POST, .handler = handler_save_post },
        { .uri = "/reboot",  .method = HTTP_GET,  .handler = handler_reboot    },
        { .uri = "/factory", .method = HTTP_GET,  .handler = handler_factory   },
        { .uri = "/ota",     .method = HTTP_POST, .handler = ota_upload_handler},
    };
    for (int i = 0; i < 5; i++)
        httpd_register_uri_handler(server, &uris[i]);

    ESP_LOGI(TAG, "Config server ready (non-blocking)");
    return ESP_OK;
}
