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

static char s_flash[64] = {0};

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

#define SEL(cond) ((cond) ? "selected" : "")
#define CHK(cond) ((cond) ? "checked"  : "")

/* Minified head + CSS. Classes: h=header n=nav p=page s=section f=field
 * c=checkbox-row r=blacklist-row a=actions bt=button bp/bd/bo=variants
 * t=tab o=active al=alert i=info-table. Tab IDs: g r u s f. */
static const char HEAD[] =
"<!DOCTYPE html><html><head><meta charset=utf-8>"
"<meta name=viewport content=\"width=device-width,initial-scale=1\">"
"<title>Westshore Watch X1</title><style>"
"*{box-sizing:border-box;margin:0;padding:0}"
"body{font:14px system-ui;background:#f4f4f4;color:#222}"
".h{background:#0b1118;color:#fff;padding:12px}"
".h h1{font-size:1.1rem;color:#00d4ff}"
".h small{color:#888;font-size:.7rem}"
".n{display:flex;background:#111c26;overflow-x:auto}"
".n button{background:0;border:0;border-bottom:3px solid transparent;color:#aaa;padding:10px 14px;font-size:.85rem;white-space:nowrap;cursor:pointer}"
".n .o{border-color:#00d4ff;color:#fff}"
".p{padding:14px;max-width:560px}"
".s{margin-bottom:16px}"
".s h3{font-size:.75rem;color:#555;text-transform:uppercase;border-bottom:1px solid #ddd;padding-bottom:4px;margin-bottom:10px}"
".f{margin-bottom:12px}"
".f label{display:block;font-size:.85rem;margin-bottom:2px}"
".f input,.f select{width:100%;padding:6px 8px;border:1px solid #ccc;border-radius:4px;font-size:.85rem}"
".f input[readonly]{background:#eee;color:#666}"
".c{display:flex;gap:8px;align-items:center}"
".c input{width:16px;height:16px}"
".r input{width:100%;margin-bottom:4px;padding:5px 8px;border:1px solid #ccc;border-radius:4px;font-size:.8rem}"
".a{display:flex;gap:8px;flex-wrap:wrap;padding-top:14px;border-top:1px solid #ddd;margin-top:10px}"
".bt{padding:8px 16px;border:0;border-radius:4px;cursor:pointer;font-size:.85rem}"
".bp{background:#00a0bf;color:#fff}"
".bd{background:#c62828;color:#fff}"
".bo{background:#fff;border:1px solid #bbb}"
".t{display:none}.t.o{display:block}"
".al{background:#e8f5e9;border:1px solid #a5d6a7;color:#1b5e20;padding:8px;border-radius:4px;margin-bottom:10px;font-size:.85rem}"
".i td{padding:4px 0;font-size:.85rem}"
".i td:first-child{color:#666;width:120px}"
"</style></head><body>";

static const char SCRIPT[] =
"<script>"
"function T(b,i){"
"document.querySelectorAll('.n button').forEach(x=>x.classList.remove('o'));"
"document.querySelectorAll('.t').forEach(x=>x.classList.remove('o'));"
"b.classList.add('o');document.getElementById(i).classList.add('o');"
"localStorage.setItem('w',i)}"
"onload=function(){"
"var t=localStorage.getItem('w')||'g',"
"b=document.querySelector('[data-t=\"'+t+'\"]');"
"T(b||document.querySelector('.n button'),b?t:'g')}"
"</script></body></html>";

static esp_err_t handler_root_get(httpd_req_t *req)
{
    ESP_LOGI(TAG, "GET %s", req->uri);

    wsd_config_t *c = &g_config;
    char mac_str[18];
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);
    snprintf(mac_str, sizeof(mac_str), "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    httpd_resp_set_type(req, "text/html");
    httpd_resp_send_chunk(req, HEAD, HTTPD_RESP_USE_STRLEN);

    char buf[2048];

    /* Header + nav + open form */
    snprintf(buf, sizeof(buf),
        "<div class=h><h1>WESTSHORE WATCH X1</h1><small>%s</small></div>"
        "<div class=n>"
        "<button class=o data-t=g onclick=\"T(this,'g')\">General</button>"
        "<button data-t=r onclick=\"T(this,'r')\">Reception</button>"
        "<button data-t=u onclick=\"T(this,'u')\">UART</button>"
        "<button data-t=s onclick=\"T(this,'s')\">System</button>"
        "<button data-t=f onclick=\"T(this,'f')\">Firmware</button>"
        "</div>"
        "<div class=p><form id=F method=POST action=/save>",
        mac_str);
    httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

    if (s_flash[0]) {
        snprintf(buf, sizeof(buf), "<div class=al>%s</div>", s_flash);
        httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);
        s_flash[0] = '\0';
    }

    /* General tab (default visible) */
    snprintf(buf, sizeof(buf),
        "<div id=g class=\"t o\">"
        "<div class=s><h3>Operation</h3>"
        "<div class=f><label>Mode</label>"
        "<select name=mode>"
        "<option value=0 %s>Wireless relay</option>"
        "<option value=1 %s>UART receiver</option>"
        "</select></div>"
        "<div class=f><div class=c>"
        "<input type=checkbox name=ping_en value=1 %s><label>Relay ping</label>"
        "</div></div>"
        "<div class=f><label>Ping label</label>"
        "<input name=ping_label value=\"%s\" maxlength=31></div>"
        "<div class=f><div class=c>"
        "<input type=checkbox name=self_id_ovr value=1 %s><label>Self ID override</label>"
        "</div></div>"
        "<div class=f><div class=c>"
        "<input type=checkbox name=ble_en value=1 %s><label>BLE legacy</label>"
        "</div></div>"
        "<div class=f><div class=c>"
        "<input type=checkbox name=led_en value=1 %s><label>Flash LED</label>"
        "</div></div></div>",
        SEL(c->mode == WSD_MODE_RELAY), SEL(c->mode == WSD_MODE_UART),
        CHK(c->relay_ping_en), c->relay_ping_label,
        CHK(c->self_id_override), CHK(c->ble_legacy_en), CHK(c->flash_led_en));
    httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

    /* Location */
    snprintf(buf, sizeof(buf),
        "<div class=s><h3>Location</h3>"
        "<div class=f><label>Latitude</label>"
        "<input name=node_lat value=\"%.6f\" maxlength=15></div>"
        "<div class=f><label>Longitude</label>"
        "<input name=node_lon value=\"%.6f\" maxlength=15></div>"
        "</div>",
        c->node_lat, c->node_lon);
    httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

    /* API Key — read-only display of current, plus writable input to replace.
     * Submitting the form with the "New" field empty leaves the key unchanged. */
    snprintf(buf, sizeof(buf),
        "<div class=s><h3>API Key</h3>"
        "<div class=f><label>Current</label>"
        "<input name=aw_key value=\"%s\" readonly></div>"
        "<div class=f><label>New (paste to replace)</label>"
        "<input name=aw_key_new value=\"\" maxlength=%d "
        "placeholder=\"leave blank to keep current\"></div>"
        "</div>",
        c->api_key, WSD_API_KEY_MAX - 1);
    httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

    /* Blacklist */
    httpd_resp_send_chunk(req, "<div class=s><h3>Blacklist</h3>",
                          HTTPD_RESP_USE_STRLEN);
    for (int i = 0; i < WSD_BLACKLIST_MAX; i++) {
        const char *val = (i < c->blacklist_count) ? c->blacklist[i] : "";
        snprintf(buf, sizeof(buf),
            "<div class=r><input name=bl_%d value=\"%s\" maxlength=31></div>",
            i, val);
        httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);
    }
    httpd_resp_send_chunk(req, "</div></div>", HTTPD_RESP_USE_STRLEN);

    /* Reception tab — strategy + channel selects */
    snprintf(buf, sizeof(buf),
        "<div id=r class=t>"
        "<div class=s><h3>WiFi Scanning</h3>"
        "<div class=f><label>Strategy</label>"
        "<select name=wifi_strat>"
        "<option value=0 %s>Balanced</option>"
        "<option value=1 %s>Follow</option>"
        "<option value=2 %s>Fixed</option>"
        "</select></div>"
        "<div class=f><label>Ch start</label><select name=ch_start>",
        SEL(c->wifi_strategy == WSD_WIFI_STRAT_BALANCED),
        SEL(c->wifi_strategy == WSD_WIFI_STRAT_FOLLOW),
        SEL(c->wifi_strategy == WSD_WIFI_STRAT_FIXED));
    httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

    for (int ch = 1; ch <= 13; ch++) {
        snprintf(buf, sizeof(buf), "<option value=%d %s>%d</option>",
                 ch, ch == c->ch_2g_start ? "selected" : "", ch);
        httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);
    }
    httpd_resp_send_chunk(req,
        "</select></div><div class=f><label>Ch stop</label><select name=ch_stop>",
        HTTPD_RESP_USE_STRLEN);
    for (int ch = 1; ch <= 13; ch++) {
        snprintf(buf, sizeof(buf), "<option value=%d %s>%d</option>",
                 ch, ch == c->ch_2g_stop ? "selected" : "", ch);
        httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);
    }
    httpd_resp_send_chunk(req, "</select></div></div>", HTTPD_RESP_USE_STRLEN);

    snprintf(buf, sizeof(buf),
        "<div class=s><h3>Timeouts</h3>"
        "<div class=f><label>Landing (s)</label>"
        "<input type=number name=land_tout value=%d min=1 max=60></div>"
        "<div class=f><label>Silent (s)</label>"
        "<input type=number name=silent_tout value=%d min=5 max=120></div>"
        "</div></div>",
        c->land_timeout_s, c->silent_timeout_s);
    httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

    /* UART tab */
    snprintf(buf, sizeof(buf),
        "<div id=u class=t>"
        "<div class=s><h3>Serial</h3>"
        "<div class=f><label>Baud</label>"
        "<select name=uart_baud>"
        "<option value=9600 %s>9600</option>"
        "<option value=19200 %s>19200</option>"
        "<option value=38400 %s>38400</option>"
        "<option value=57600 %s>57600</option>"
        "<option value=115200 %s>115200</option>"
        "</select></div></div></div>",
        SEL(c->uart_baud == 9600), SEL(c->uart_baud == 19200),
        SEL(c->uart_baud == 38400), SEL(c->uart_baud == 57600),
        SEL(c->uart_baud == 115200));
    httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

    /* System tab */
    snprintf(buf, sizeof(buf),
        "<div id=s class=t>"
        "<div class=s><h3>Device Info</h3>"
        "<table class=i>"
        "<tr><td>Firmware</td><td>v1.1</td></tr>"
        "<tr><td>MAC</td><td>%s</td></tr>"
        "<tr><td>IDF</td><td>%s</td></tr>"
        "<tr><td>Free heap</td><td>%lu</td></tr>"
        "</table></div></div>",
        mac_str, esp_get_idf_version(),
        (unsigned long)esp_get_free_heap_size());
    httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

    /* Close main settings form before firmware tab (which has its own form) */
    httpd_resp_send_chunk(req, "</form>", HTTPD_RESP_USE_STRLEN);

    /* Firmware tab */
    {
        char run_label[16], next_label[16];
        ota_get_info(run_label, sizeof(run_label), next_label, sizeof(next_label));
        snprintf(buf, sizeof(buf),
            "<div id=f class=t>"
            "<div class=s><h3>Firmware</h3>"
            "<table class=i>"
            "<tr><td>Running</td><td>%s</td></tr>"
            "<tr><td>Update</td><td>%s</td></tr>"
            "</table>"
            "<form method=POST action=/ota enctype=multipart/form-data>"
            "<div class=f><input type=file name=firmware accept=.bin></div>"
            "<div class=a><button type=submit class=\"bt bp\">Upload</button></div>"
            "</form></div></div>",
            run_label, next_label);
        httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);
    }

    /* Actions bar — Save button submits F form even though it's outside */
    httpd_resp_send_chunk(req,
        "<div class=a>"
        "<button type=button class=\"bt bp\" onclick=\"F.submit()\">Save</button>"
        "<button type=button class=\"bt bo\" onclick=\"confirm('Factory reset?')&&(location='/factory')\">Defaults</button>"
        "<button type=button class=\"bt bd\" onclick=\"confirm('Reboot?')&&(location='/reboot')\">Reboot</button>"
        "</div></div>",
        HTTPD_RESP_USE_STRLEN);

    httpd_resp_send_chunk(req, SCRIPT, HTTPD_RESP_USE_STRLEN);
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

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

    if (form_get_field(body, "mode", val, sizeof(val)))
        c->mode = (wsd_mode_t)atoi(val);
    c->relay_ping_en    = form_has_field(body, "ping_en");
    c->self_id_override = form_has_field(body, "self_id_ovr");
    c->ble_legacy_en    = form_has_field(body, "ble_en");
    c->flash_led_en     = form_has_field(body, "led_en");
    if (form_get_field(body, "ping_label", val, sizeof(val)) && val[0])
        strlcpy(c->relay_ping_label, val, sizeof(c->relay_ping_label));

    if (form_get_field(body, "node_lat", val, sizeof(val)))
        c->node_lat = atof(val);
    if (form_get_field(body, "node_lon", val, sizeof(val)))
        c->node_lon = atof(val);

    /* API key: prefer the writable "new" field when non-empty, otherwise
     * fall back to the read-only aw_key field which round-trips the current
     * value. Either way, only overwrite when we have something to store so
     * an empty form submission cannot wipe the key. */
    if (form_get_field(body, "aw_key_new", val, sizeof(val)) && val[0]) {
        strlcpy(c->api_key, val, sizeof(c->api_key));
    } else if (form_get_field(body, "aw_key", val, sizeof(val)) && val[0]) {
        strlcpy(c->api_key, val, sizeof(c->api_key));
    }

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

    if (form_get_field(body, "uart_baud", val, sizeof(val))) {
        uint32_t b = atoi(val);
        if (b == 9600 || b == 19200 || b == 38400 ||
            b == 57600 || b == 115200)
            c->uart_baud = b;
    }

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

    free(body);

    wsd_config_save(c);
    strlcpy(s_flash, "Settings saved. Reboot to apply.", sizeof(s_flash));

    httpd_resp_set_status(req, "303 See Other");
    httpd_resp_set_hdr(req, "Location", "/");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

static esp_err_t handler_reboot(httpd_req_t *req)
{
    httpd_resp_sendstr(req,
        "<html><body style=\"font:14px system-ui;padding:20px\">Rebooting...</body></html>");
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
    return ESP_OK;
}

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

/* Captive-portal probe handlers — answer what each OS expects so it stops
 * retrying and frees the socket pool for the config UI. */
static esp_err_t handler_generate_204(httpd_req_t *req)
{
    httpd_resp_set_status(req, "204 No Content");
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

static esp_err_t handler_hotspot_detect(httpd_req_t *req)
{
    static const char body[] =
        "<HTML><HEAD><TITLE>Success</TITLE></HEAD><BODY>Success</BODY></HTML>";
    httpd_resp_set_status(req, "200 OK");
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, body, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t handler_ncsi(httpd_req_t *req)
{
    static const char body[] = "Microsoft NCSI";
    httpd_resp_set_status(req, "200 OK");
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_send(req, body, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t not_found_handler(httpd_req_t *req, httpd_err_code_t err)
{
    (void)err;
    ESP_LOGI(TAG, "404 %s", req->uri);
    httpd_resp_set_status(req, "404 Not Found");
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_send(req, "Not Found", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

esp_err_t config_server_start_http(void)
{
    ESP_LOGI(TAG, "Starting HTTP config server on http://%s", CFG_AP_IP);

    httpd_config_t hcfg = HTTPD_DEFAULT_CONFIG();
    hcfg.stack_size        = 12288;
    hcfg.max_uri_handlers  = 16;
    hcfg.max_open_sockets  = 12;
    hcfg.recv_wait_timeout = 3;
    hcfg.send_wait_timeout = 3;
    hcfg.lru_purge_enable  = true;
    hcfg.uri_match_fn      = NULL;

    httpd_handle_t server = NULL;
    esp_err_t err = httpd_start(&server, &hcfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "HTTP server start failed: %d", err);
        return err;
    }

    httpd_uri_t uris[] = {
        { .uri = "/",                    .method = HTTP_GET,  .handler = handler_root_get       },
        { .uri = "/save",                .method = HTTP_POST, .handler = handler_save_post      },
        { .uri = "/reboot",              .method = HTTP_GET,  .handler = handler_reboot         },
        { .uri = "/factory",             .method = HTTP_GET,  .handler = handler_factory        },
        { .uri = "/ota",                 .method = HTTP_POST, .handler = ota_upload_handler     },
        { .uri = "/generate_204",        .method = HTTP_GET,  .handler = handler_generate_204   },
        { .uri = "/gen_204",             .method = HTTP_GET,  .handler = handler_generate_204   },
        { .uri = "/hotspot-detect.html", .method = HTTP_GET,  .handler = handler_hotspot_detect },
        { .uri = "/ncsi.txt",            .method = HTTP_GET,  .handler = handler_ncsi           },
        { .uri = "/connecttest.txt",     .method = HTTP_GET,  .handler = handler_ncsi           },
    };
    for (size_t i = 0; i < sizeof(uris) / sizeof(uris[0]); i++)
        httpd_register_uri_handler(server, &uris[i]);

    httpd_register_err_handler(server, HTTPD_404_NOT_FOUND, not_found_handler);

    ESP_LOGI(TAG, "Config server ready");
    return ESP_OK;
}
