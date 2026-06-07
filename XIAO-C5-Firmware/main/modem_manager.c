#include "modem_manager.h"
#include "cellular_uart.h"
#include "gnss_reader.h"
#include "status_led.h"
#include "config.h"
#include "esp_log.h"
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
#define NETOPEN_URC_MS           40000   /* wait for async +NETOPEN: 0 */
#define RESP_BUF_SIZE            256

/* Pre-net GPS is best-effort and time-boxed so it never delays bring-up:
 * power GPS, try briefly for a fix, then proceed regardless. A cold first
 * boot that gets nothing here is fine — the monitor loop keeps polling until
 * it lands one, then re-polls on a cadence (this is a MOBILE node, so its
 * position must keep tracking movement, not freeze at the boot fix). */
#define GNSS_PRENET_ATTEMPTS     5       /* ~5 s max */
#define GNSS_PRENET_INTERVAL_MS  1000

/* Monitor loop: poll cadence + how often to run the heavier health check
 * (NETOPEN? + registration).  GPS is polled every tick until the first fix,
 * then re-polled every GNSS_REPOLL_EVERY ticks so a mobile node's reported
 * position keeps up with where it actually is. */
#define MONITOR_POLL_MS          5000
#define HEALTH_CHECK_EVERY       3       /* ~15 s between health checks */
#define GNSS_REPOLL_EVERY        3       /* re-poll GPS every ~15 s after first fix */

/* ── State ────────────────────────────────────────────────────────────────── */
static volatile modem_state_t s_state  = MODEM_STATE_OFF;
static volatile bool          s_net_up = false;

modem_state_t modem_manager_get_state(void) { return s_state; }
bool modem_manager_is_connected(void)       { return s_net_up; }

/* ── Helpers ──────────────────────────────────────────────────────────────── */
static void set_state(modem_state_t new_state)
{
    if (s_state == new_state) return;
    ESP_LOGI(TAG, "state %d → %d", (int)s_state, (int)new_state);
    s_state = new_state;

    switch (new_state) {
    case MODEM_STATE_ONLINE: status_led_set(STATUS_LED_HEALTHY); break;  /* solid yellow */
    case MODEM_STATE_ERROR:  status_led_set(STATUS_LED_FAULT);   break;  /* solid red    */
    case MODEM_STATE_OFF:    status_led_set(STATUS_LED_OFF);     break;
    default:                 status_led_set(STATUS_LED_WARMING); break;  /* slow-blink yellow */
    }
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

/* Power the GNSS receiver (SIM7600 dialect AT+CGPS=1 — returns ERROR if
 * already on, harmless). */
static void gnss_power_on(void)
{
    char resp[RESP_BUF_SIZE];
    cellular_uart_send_at("AT+CGPS=1", resp, sizeof(resp), AT_TIMEOUT_MS);
}

/* One bounded AT+CGPSINFO poll, feeding any fix to the gnss_reader cache.
 * send_at takes the UART mutex, so this never interleaves with an HTTP POST. */
static void gnss_poll_once(void)
{
    char resp[RESP_BUF_SIZE];
    if (cellular_uart_send_at("AT+CGPSINFO", resp, sizeof(resp), AT_TIMEOUT_MS) == ESP_OK) {
        gnss_reader_submit_response(resp);
    }
}

/* ── AT bring-up phase (wake / SIM / registration / GPS power) ─────────────── */
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

    /* Pre-net GPS: best-effort, time-boxed.  We stay in AT command mode for
     * the whole session now, so the monitor loop also polls — this just gives
     * a fast first fix when the receiver already has one cached. */
    if (!gnss_reader_have_fix()) {
        gnss_power_on();
        for (int i = 0; i < GNSS_PRENET_ATTEMPTS; i++) {
            gnss_poll_once();
            if (gnss_reader_have_fix()) break;
            vTaskDelay(pdMS_TO_TICKS(GNSS_PRENET_INTERVAL_MS));
        }
        if (!gnss_reader_have_fix()) {
            ESP_LOGW(TAG, "no GPS fix in pre-net window — uploading without node_position");
        }
    }

    set_state(MODEM_STATE_REGISTERED);
    return true;
}

/* ── NETOPEN (idempotent / already-open safe) ──────────────────────────────
 * AT+NETOPEN replies "OK" then later an async "+NETOPEN: 0" (0 = success).
 * If the socket service is already open it instead returns ERROR / 903 /
 * "already opened". The query AT+NETOPEN? is the unambiguous source of truth:
 * "+NETOPEN: 1" = open, "+NETOPEN: 0" = closed. We hold the AT lock across
 * the whole dance so the async URC can't be consumed by another AT user. */
static bool net_open(void)
{
    char resp[RESP_BUF_SIZE];
    bool open = false;

    cellular_uart_lock();
    for (int attempt = 0; attempt < 3 && !open; attempt++) {
        /* Already open? */
        if (cellular_uart_send_at("AT+NETOPEN?", resp, sizeof(resp), AT_TIMEOUT_MS) == ESP_OK &&
            strstr(resp, "+NETOPEN: 1")) {
            open = true;
            break;
        }

        /* Try to open.  Only wait for the async URC if the command was
         * accepted (OK); an ERROR means already-open/903 — re-query instead
         * of blocking 40 s for a URC that will never come. */
        if (cellular_uart_send_at("AT+NETOPEN", resp, sizeof(resp), AT_TIMEOUT_MS) == ESP_OK) {
            cellular_uart_collect("+NETOPEN:", resp, sizeof(resp), NETOPEN_URC_MS);
        }

        /* Source of truth: re-query. */
        if (cellular_uart_send_at("AT+NETOPEN?", resp, sizeof(resp), AT_TIMEOUT_MS) == ESP_OK &&
            strstr(resp, "+NETOPEN: 1")) {
            open = true;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(2000 * (attempt + 1)));
    }
    cellular_uart_unlock();
    return open;
}

/* ── Data-session bring-up: NETOPEN + TLS config (proven recipe) ───────────── */
static bool net_phase(void)
{
    set_state(MODEM_STATE_NET_OPENING);

    if (!net_open()) {
        ESP_LOGE(TAG, "AT+NETOPEN failed after retries");
        return false;
    }
    ESP_LOGI(TAG, "NETOPEN up — configuring TLS context 0");

    /* PROVEN recipe (LE20B04SIM7600G22). Params are case-SENSITIVE lowercase;
     * enableSNI is REQUIRED for the Render/Cloudflare backend. authmode 0 =
     * no cert verify (v1 pragmatic; CA load is a later option via "cacert"). */
    static const char *ssl_cfg[] = {
        "AT+CSSLCFG=\"sslversion\",0,4",
        "AT+CSSLCFG=\"authmode\",0,0",
        "AT+CSSLCFG=\"enableSNI\",0,1",
        "AT+CSSLCFG=\"ignorelocaltime\",0,1",
    };
    char resp[RESP_BUF_SIZE];
    for (size_t i = 0; i < sizeof(ssl_cfg) / sizeof(ssl_cfg[0]); i++) {
        if (cellular_uart_send_at(ssl_cfg[i], resp, sizeof(resp), AT_TIMEOUT_MS) != ESP_OK) {
            ESP_LOGE(TAG, "SSL config failed: %s", ssl_cfg[i]);
            return false;
        }
    }
    ESP_LOGI(TAG, "TLS context configured (sslversion=4 authmode=0 SNI=on)");
    return true;
}

/* ── Main task ────────────────────────────────────────────────────────────── */
static void modem_task(void *arg)
{
    char resp[RESP_BUF_SIZE];

    while (true) {
        s_net_up = false;

        if (!at_phase()) {
            set_state(MODEM_STATE_ERROR);
            ESP_LOGE(TAG, "AT phase failed — sending AT+CFUN=1,1 reset");
            cellular_uart_send_at("AT+CFUN=1,1", resp, sizeof(resp), AT_TIMEOUT_MS);
            // TODO: add MOSFET-based power-cycle if AT+CFUN=1,1 is insufficient
            ESP_LOGW(TAG, "waiting 60s before retry...");
            vTaskDelay(pdMS_TO_TICKS(60000));
            continue;
        }

        if (!net_phase()) {
            set_state(MODEM_STATE_ERROR);
            ESP_LOGE(TAG, "data-session bring-up failed — NETCLOSE + retry in 10s");
            cellular_uart_send_at("AT+NETCLOSE", resp, sizeof(resp), AT_TIMEOUT_MS);
            vTaskDelay(pdMS_TO_TICKS(10000));
            continue;
        }

        s_net_up = true;
        set_state(MODEM_STATE_ONLINE);
        ESP_LOGI(TAG, "cellular data session UP — native AT HTTP ready");

        /* Monitor loop: re-poll GPS on a cadence so this MOBILE node's
         * reported position tracks where it actually is (not just the boot
         * fix), and periodically verify the data session + registration are
         * still healthy.  All of these are AT commands serialized against
         * HTTP POSTs by the UART mutex, so they queue cleanly rather than
         * colliding. */
        int hc = 0;
        int gc = 0;
        while (s_net_up) {
            /* Before the first fix, poll every tick to acquire one fast.
             * After that, re-poll every GNSS_REPOLL_EVERY ticks (~15 s) so the
             * cached node_position attached to detection uploads keeps up with
             * movement, without hammering the AT channel. gnss_poll_once()
             * takes the UART mutex, so it never interleaves with an HTTP POST.
             * On a momentary no-fix the reader keeps the last-known-good fix
             * (it only overwrites on a valid parse), so a brief GPS dropout
             * doesn't blank the position. */
            if (!gnss_reader_have_fix() || ++gc >= GNSS_REPOLL_EVERY) {
                gc = 0;
                gnss_poll_once();
            }
            if (++hc >= HEALTH_CHECK_EVERY) {
                hc = 0;
                bool open = (cellular_uart_send_at("AT+NETOPEN?", resp, sizeof(resp),
                                                   AT_TIMEOUT_MS) == ESP_OK &&
                             strstr(resp, "+NETOPEN: 1"));
                if (!open || !registered_ok()) {
                    ESP_LOGW(TAG, "data session/registration lost (netopen=%d) — re-establishing",
                             open);
                    s_net_up = false;
                    break;
                }
            }
            vTaskDelay(pdMS_TO_TICKS(MONITOR_POLL_MS));
        }

        set_state(MODEM_STATE_ERROR);
        ESP_LOGW(TAG, "data session down — NETCLOSE + restarting modem sequence");
        cellular_uart_send_at("AT+NETCLOSE", resp, sizeof(resp), AT_TIMEOUT_MS);
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

esp_err_t modem_manager_start(void)
{
    esp_err_t err = cellular_uart_init();
    if (err != ESP_OK) return err;

    BaseType_t ret = xTaskCreate(modem_task, "modem_mgr", 8192, NULL,
                                 WSD_OUTPUT_TASK_PRIO + 1, NULL);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "failed to create modem_task");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "modem manager task started (native AT HTTP transport)");
    return ESP_OK;
}
