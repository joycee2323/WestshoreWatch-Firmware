#include "modem_manager.h"
#include "cellular_uart.h"
#include "modem_data_resume.h"
#include "gnss_reader.h"
#include "status_led.h"
#include "config.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_modem_api.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

static const char *TAG = "MODEM_MGR";

/* ── Tuning ───────────────────────────────────────────────────────────────── */
#define STATE_STUCK_TIMEOUT_MS   (5 * 60 * 1000)
#define AT_RETRY_DELAY_MS        2000
#define AT_TIMEOUT_MS            5000
#define PPP_CONNECT_TIMEOUT_MS   (30 * 1000)
/* Pre-PPP GPS is best-effort and time-boxed so it never delays uploads:
 * power GPS, try briefly for a fix, then proceed to PPP regardless. A
 * stationary node only needs one fix; cold TTFF means first boot often
 * gets none — fine, node_position is simply omitted until a later AT
 * phase (reconnect) catches one. */
#define GNSS_PREPPP_ATTEMPTS     5       /* ~5 s max */
#define GNSS_PREPPP_INTERVAL_MS  1000
#define RESP_BUF_SIZE            256
/* Buffer for esp_modem_at()/at_raw() responses. The C-API strlcpy's up to
 * CONFIG_ESP_MODEM_C_API_STR_MAX bytes into the caller's buffer, so it MUST
 * be at least that large. Multi-line AT+CGDCONT? (one line per PDP context)
 * needs the headroom or later contexts get truncated. Keep in sync with
 * CONFIG_ESP_MODEM_C_API_STR_MAX (512). */
#define MODEM_AT_RESP_SIZE       512

/* ── esp_modem UART pins (must match cellular_uart) ───────────────────────── */
/* Verified XIAO ESP32-C5 mapping from the official pins_arduino.h:
 * D4=GPIO23 (C5 TX → modem R), D5=GPIO24 (modem T → C5 RX). */
#define CELL_UART_TX_GPIO   23
#define CELL_UART_RX_GPIO   24

/* ── State ────────────────────────────────────────────────────────────────── */
static volatile modem_state_t s_state = MODEM_STATE_OFF;
static esp_modem_dce_t       *s_dce   = NULL;
static esp_netif_t           *s_netif = NULL;
static volatile bool          s_ppp_connected = false;

modem_state_t modem_manager_get_state(void) { return s_state; }
bool modem_manager_is_connected(void)       { return s_ppp_connected; }

/* ── Helpers ──────────────────────────────────────────────────────────────── */
static void set_state(modem_state_t new_state)
{
    if (s_state == new_state) return;
    ESP_LOGI(TAG, "state %d → %d", (int)s_state, (int)new_state);
    s_state = new_state;

    switch (new_state) {
    case MODEM_STATE_PPP_CONNECTED: status_led_set(STATUS_LED_GREEN);       break;
    case MODEM_STATE_ERROR:         status_led_set(STATUS_LED_RED);          break;
    case MODEM_STATE_OFF:           status_led_set(STATUS_LED_OFF);          break;
    default:                        status_led_set(STATUS_LED_YELLOW);       break;
    }
}

static void read_nvs_string(const char *key, char *out, size_t max, const char *dflt)
{
    nvs_handle_t h;
    if (nvs_open("cell", NVS_READONLY, &h) == ESP_OK) {
        size_t len = max;
        if (nvs_get_str(h, key, out, &len) != ESP_OK) {
            strncpy(out, dflt, max);
            out[max - 1] = '\0';
        }
        nvs_close(h);
    } else {
        strncpy(out, dflt, max);
        out[max - 1] = '\0';
    }
}

/* ── PPP event handler ────────────────────────────────────────────────────── */
static void on_ip_event(void *arg, esp_event_base_t base,
                        int32_t event_id, void *data)
{
    if (event_id == IP_EVENT_PPP_GOT_IP) {
        ip_event_got_ip_t *e = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "PPP got IP: " IPSTR, IP2STR(&e->ip_info.ip));
        s_ppp_connected = true;
        set_state(MODEM_STATE_PPP_CONNECTED);
    } else if (event_id == IP_EVENT_PPP_LOST_IP) {
        ESP_LOGW(TAG, "PPP lost IP");
        s_ppp_connected = false;
        set_state(MODEM_STATE_ERROR);
    }
}

/* ── Full multi-line AT capture ───────────────────────────────────────────
 * esp_modem_at() returns only the LAST information line (generic_get_string
 * overwrites its output per line), so a multi-line reply like AT+CGDCONT?
 * (one +CGDCONT: line per context) collapses to whatever context is listed
 * last. esp_modem_command() hands us each chunk via a callback, so we
 * accumulate the WHOLE response here until the OK/ERROR result code. The
 * callback has no user-context arg, hence the file-scope accumulator (only
 * ever called from the single modem task). */
static char   s_at_acc[MODEM_AT_RESP_SIZE];
static size_t s_at_acc_len;

static esp_err_t at_accumulate_cb(uint8_t *data, size_t len)
{
    if (data && len && s_at_acc_len < sizeof(s_at_acc) - 1) {
        size_t room = sizeof(s_at_acc) - 1 - s_at_acc_len;
        size_t n = len < room ? len : room;
        memcpy(s_at_acc + s_at_acc_len, data, n);
        s_at_acc_len += n;
        s_at_acc[s_at_acc_len] = '\0';
    }
    /* Return TIMEOUT to keep reading; OK/FAIL to finish (see esp_modem_command). */
    if (strstr(s_at_acc, "OK\r") || strstr(s_at_acc, "\nOK"))   return ESP_OK;
    if (strstr(s_at_acc, "ERROR"))                              return ESP_FAIL;
    return ESP_ERR_TIMEOUT;
}

/* Run an AT command capturing the FULL multi-line response into out. */
static esp_err_t modem_at_full(const char *cmd_with_cr, char *out, size_t out_sz,
                               uint32_t timeout_ms)
{
    s_at_acc_len = 0;
    s_at_acc[0] = '\0';
    esp_err_t r = esp_modem_command(s_dce, cmd_with_cr, at_accumulate_cb, timeout_ms);
    if (out && out_sz) strlcpy(out, s_at_acc, out_sz);
    return r;
}

/* Parse the <stat> field from "+CREG: <n>,<stat>..." (or +CGREG).
 * Returns stat, or -1 if not parseable. */
static int parse_reg_stat(const char *resp, const char *prefix)
{
    const char *p = strstr(resp, prefix);
    if (!p) return -1;
    p = strchr(p, ':');
    if (!p) return -1;
    p = strchr(p + 1, ',');     /* skip <n> */
    if (!p) return -1;
    return atoi(p + 1);         /* <stat> */
}

/* Registered if circuit- OR packet-domain shows home (stat 1) or roaming
 * (stat 5). Hologram registers as roaming, so 5 must count as success. */
static bool registered_ok(void)
{
    char resp[RESP_BUF_SIZE];
    if (cellular_uart_send_at("AT+CREG?", resp, sizeof(resp), AT_TIMEOUT_MS) == ESP_OK) {
        int s = parse_reg_stat(resp, "+CREG:");
        if (s == 1 || s == 5) return true;
    }
    if (cellular_uart_send_at("AT+CGREG?", resp, sizeof(resp), AT_TIMEOUT_MS) == ESP_OK) {
        int s = parse_reg_stat(resp, "+CGREG:");
        if (s == 1 || s == 5) return true;
    }
    return false;
}

/* ── Pre-PPP AT command phase ─────────────────────────────────────────────── */
static bool at_phase(void)
{
    char resp[RESP_BUF_SIZE];
    TickType_t phase_start = xTaskGetTickCount();

    set_state(MODEM_STATE_BOOTING);
    ESP_LOGI(TAG, "waiting for modem auto-boot (TEL0162 PWRKEY tied to GND)");

    /* Autobaud wake burst + baud scan.
     * SIM7600 default AT+IPR=0 (autobaud) needs rapid AT\r cadence to lock
     * onto the host rate — the 2s polling loop below alone never syncs. */
    cellular_uart_wake_and_lock_baud();

    /* Wait for modem to respond to AT (modem takes 5-15s after VBAT applied) */
    bool at_ok = false;
    for (int i = 0; i < 15; i++) {
        vTaskDelay(pdMS_TO_TICKS(2000));
        if (cellular_uart_send_at("AT", resp, sizeof(resp), AT_TIMEOUT_MS) == ESP_OK) {
            at_ok = true;
            break;
        }
    }
    if (!at_ok) {
        ESP_LOGE(TAG, "modem not responding after 30s — check 5V on TEL0162 Power IN");
        return false;
    }
    ESP_LOGI(TAG, "modem responding to AT");

    /* Disable echo */
    cellular_uart_send_at("ATE0", resp, sizeof(resp), AT_TIMEOUT_MS);

    /* SIM check */
    set_state(MODEM_STATE_SIM_CHECK);
    for (int i = 0; i < 10; i++) {
        if (cellular_uart_send_at("AT+CPIN?", resp, sizeof(resp), AT_TIMEOUT_MS) == ESP_OK) {
            if (strstr(resp, "READY")) {
                ESP_LOGI(TAG, "SIM ready");
                break;
            }
        }
        if (i == 9) {
            ESP_LOGE(TAG, "SIM not ready after 10 attempts");
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(AT_RETRY_DELAY_MS));
    }

    /* Network registration */
    set_state(MODEM_STATE_NETWORK_SEARCH);
    for (int i = 0; i < 60; i++) {
        if (registered_ok()) {
            ESP_LOGI(TAG, "network registered (CREG/CGREG stat 1=home or 5=roaming)");
            break;
        }
        if (i == 59) {
            ESP_LOGE(TAG, "network registration failed after 60 attempts");
            return false;
        }
        if ((xTaskGetTickCount() - phase_start) * portTICK_PERIOD_MS > STATE_STUCK_TIMEOUT_MS) {
            ESP_LOGE(TAG, "registration stuck > 5 min — will attempt AT+CFUN=1,1");
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    /* Pre-PPP GPS: best-effort, time-boxed, on the UART we still own (no
     * mode switching). Power the receiver (SIM7600 dialect AT+CGPS=1 — NOT
     * SIM800's AT+CGNSPWR; returns ERROR if already on, harmless) and make
     * a short bounded AT+CGPSINFO poll. We do NOT block for a cold fix —
     * uploads must never wait on GPS. Stationary node → one cached fix is
     * reused forever, so skip entirely once we already have one. */
    if (!gnss_reader_have_fix()) {
        cellular_uart_send_at("AT+CGPS=1", resp, sizeof(resp), AT_TIMEOUT_MS);
        for (int i = 0; i < GNSS_PREPPP_ATTEMPTS; i++) {
            if (cellular_uart_send_at("AT+CGPSINFO", resp, sizeof(resp),
                                      AT_TIMEOUT_MS) == ESP_OK &&
                gnss_reader_submit_response(resp)) {
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(GNSS_PREPPP_INTERVAL_MS));
        }
        if (!gnss_reader_have_fix()) {
            ESP_LOGW(TAG, "no GPS fix in pre-PPP window — uploading without node_position");
        }
    }

    set_state(MODEM_STATE_REGISTERED);
    return true;
}

/* ── PPP phase via esp_modem ──────────────────────────────────────────────── */
static bool ppp_phase(void)
{
    set_state(MODEM_STATE_PPP_CONNECTING);

    /* Release our UART driver so esp_modem can take over */
    cellular_uart_deinit();

    char apn[64];
    read_nvs_string("cellular_apn", apn, sizeof(apn), "hologram");
    ESP_LOGI(TAG, "APN: %s", apn);

    /* IP_EVENT handlers are registered ONCE in modem_manager_start(), not
     * here — re-registering every ppp_phase leaked handlers and produced
     * "event: handler already registered, overwriting" on each restart. */

    esp_netif_config_t netif_cfg = ESP_NETIF_DEFAULT_PPP();
    s_netif = esp_netif_new(&netif_cfg);
    if (!s_netif) {
        ESP_LOGE(TAG, "esp_netif_new(PPP) failed");
        return false;
    }

    esp_modem_dte_config_t dte_config = ESP_MODEM_DTE_DEFAULT_CONFIG();
    dte_config.uart_config.port_num    = UART_NUM_1;
    dte_config.uart_config.tx_io_num   = CELL_UART_TX_GPIO;
    dte_config.uart_config.rx_io_num   = CELL_UART_RX_GPIO;
    dte_config.uart_config.baud_rate   = 115200;

    esp_modem_dce_config_t dce_config = ESP_MODEM_DCE_DEFAULT_CONFIG(apn);
    s_dce = esp_modem_new_dev(ESP_MODEM_DCE_SIM7600, &dte_config,
                              &dce_config, s_netif);
    if (!s_dce) {
        ESP_LOGE(TAG, "esp_modem_new_dev(SIM7600) failed");
        return false;
    }

    /* ── Command→DATA: explicit-cid dial + clean RESUME (SIM7600 + Hologram) ──
     * LCP/IPCP now run, but IPCP got 0.0.0.0: esp_modem's built-in dial is a
     * bare ATD*99# that lands on the modem's empty default context (no APN),
     * so the network assigns no address. esp_modem can't be told which cid to
     * dial (C config exposes only `apn`; PdpContext.context_id is hardwired
     * to 1; SIM7600 dials bare *99#). So we dial explicitly:
     *   1. Define cid 2 with the APN (IPv4).
     *   2. VERIFY with a FULL multi-line CGDCONT? readback (esp_modem_at only
     *      returns the last line, which is why earlier readbacks showed only
     *      cid 3 — false negative; modem_at_full() accumulates every line).
     *   3. Dial cid 2 explicitly (ATD*99***2#) — proven on the AT port to
     *      hold the APN and return CONNECT.
     *   4. RESUME_DATA_MODE — dte→DATA + netif.start(), NO mode-probe (unlike
     *      DETECT, whose LCP echo consumed the modem's initial LCP). */
    esp_log_level_set("lwip", ESP_LOG_DEBUG);   /* lwIP PPP debug also prints via printf */

    const int pdp_cid = 2;   /* a cid we control; bare *99# hits the empty default */
    char atresp[MODEM_AT_RESP_SIZE];
    char cgdcont[96];
    char expect[80];
    snprintf(cgdcont, sizeof(cgdcont), "AT+CGDCONT=%d,\"IP\",\"%s\"", pdp_cid, apn);
    snprintf(expect, sizeof(expect), "%d,\"IP\",\"%s\"", pdp_cid, apn);

    esp_err_t cgd = esp_modem_at(s_dce, cgdcont, atresp, AT_TIMEOUT_MS);
    ESP_LOGI(TAG, "CGDCONT=%d set -> %s", pdp_cid, esp_err_to_name(cgd));

    /* FULL readback — every +CGDCONT: line, not just the last. */
    if (modem_at_full("AT+CGDCONT?\r", atresp, sizeof(atresp), AT_TIMEOUT_MS) == ESP_OK) {
        ESP_LOGI(TAG, "CGDCONT? full readback:\n%s", atresp);
        ESP_LOGI(TAG, "cid %d holds APN at dial time? %s", pdp_cid,
                 strstr(atresp, expect) ? "YES" : "NO");
    } else {
        ESP_LOGW(TAG, "CGDCONT? full readback failed");
    }

    ESP_LOGI(TAG, "pre-dial: PPP netif=%p DCE=%p (IPv4-only; IP handlers at start)",
             (void *)s_netif, (void *)s_dce);

    /* Explicit-cid dial (the modem auto-activates the cid on ATD). */
    char dial[24];
    snprintf(dial, sizeof(dial), "ATD*99***%d#\r", pdp_cid);
    ESP_LOGI(TAG, "dialing cid %d explicitly: %s", pdp_cid, dial);
    esp_err_t derr = esp_modem_at_raw(s_dce, dial, atresp, "CONNECT", "ERROR", 10000);
    if (derr != ESP_OK) {
        ESP_LOGE(TAG, "ATD*99***%d# did not CONNECT (%s, resp='%s')",
                 pdp_cid, esp_err_to_name(derr), atresp);
        return false;
    }
    ESP_LOGI(TAG, "CONNECT on cid %d — resuming PPP (RESUME_DATA_MODE, no probe)", pdp_cid);

    esp_err_t err = modem_resume_data_mode(s_dce);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "RESUME_DATA_MODE failed: %s", esp_err_to_name(err));
        return false;
    }
    ESP_LOGI(TAG, "PPP resumed on cid %d — waiting for IPCP/IP (watch PPP debug)", pdp_cid);

    TickType_t start = xTaskGetTickCount();
    while (!s_ppp_connected) {
        vTaskDelay(pdMS_TO_TICKS(500));
        if ((xTaskGetTickCount() - start) * portTICK_PERIOD_MS > PPP_CONNECT_TIMEOUT_MS) {
            ESP_LOGE(TAG, "PPP connect timeout after 30s");
            return false;
        }
    }

    ESP_LOGI(TAG, "PPP connected — cellular link up");
    return true;
}

/* ── Main task ────────────────────────────────────────────────────────────── */
static void modem_task(void *arg)
{
    while (true) {
        s_ppp_connected = false;

        if (!at_phase()) {
            set_state(MODEM_STATE_ERROR);
            ESP_LOGE(TAG, "AT phase failed — sending AT+CFUN=1,1 reset");
            char reset_resp[RESP_BUF_SIZE];
            cellular_uart_send_at("AT+CFUN=1,1", reset_resp, sizeof(reset_resp), AT_TIMEOUT_MS);
            // TODO: add MOSFET-based power-cycle if AT+CFUN=1,1 is insufficient
            ESP_LOGW(TAG, "waiting 60s before retry...");
            vTaskDelay(pdMS_TO_TICKS(60000));
            continue;
        }

        if (!ppp_phase()) {
            set_state(MODEM_STATE_ERROR);
            ESP_LOGE(TAG, "PPP phase failed — full restart in 10s");
            if (s_dce) {
                esp_modem_destroy(s_dce);
                s_dce = NULL;
            }
            if (s_netif) {
                esp_netif_destroy(s_netif);
                s_netif = NULL;
            }
            cellular_uart_init();
            vTaskDelay(pdMS_TO_TICKS(10000));
            continue;
        }

        while (s_ppp_connected) {
            vTaskDelay(pdMS_TO_TICKS(5000));
        }

        ESP_LOGW(TAG, "PPP link lost — restarting modem sequence");
        if (s_dce) {
            esp_modem_destroy(s_dce);
            s_dce = NULL;
        }
        if (s_netif) {
            esp_netif_destroy(s_netif);
            s_netif = NULL;
        }
        cellular_uart_init();
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

esp_err_t modem_manager_start(void)
{
    esp_event_loop_create_default();
    esp_netif_init();

    /* Register PPP IP-event handlers ONCE for the lifetime of the task.
     * They key off global state (s_ppp_connected), not a specific netif, so
     * they survive netif create/destroy across PPP restarts — and aren't
     * re-registered (leaked) each ppp_phase. */
    esp_event_handler_register(IP_EVENT, IP_EVENT_PPP_GOT_IP, on_ip_event, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_PPP_LOST_IP, on_ip_event, NULL);

    esp_err_t err = cellular_uart_init();
    if (err != ESP_OK) return err;

    BaseType_t ret = xTaskCreate(modem_task, "modem_mgr", 8192, NULL,
                                 WSD_OUTPUT_TASK_PRIO + 1, NULL);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "failed to create modem_task");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "modem manager task started");
    return ESP_OK;
}
