#include "modem_manager.h"
#include "cellular_uart.h"
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

static const char *TAG = "MODEM_MGR";

/* ── Tuning ───────────────────────────────────────────────────────────────── */
#define STATE_STUCK_TIMEOUT_MS   (5 * 60 * 1000)
#define AT_RETRY_DELAY_MS        2000
#define AT_TIMEOUT_MS            5000
#define PPP_CONNECT_TIMEOUT_MS   (30 * 1000)
#define GNSS_POLL_INTERVAL_MS    (5 * 1000)   /* CGPSINFO poll cadence over CMUX until first fix */
#define RESP_BUF_SIZE            256
/* CMUX stabilization Step 1: run the UART faster than 115200 so CMUX has
 * headroom to frame PPP + AT on one wire without Rx Breaks (UART starvation
 * is the suspected cause of the CMUX state-machine restarts). */
#define CMUX_UART_BAUD           460800

/* ── esp_modem UART pins (must match cellular_uart) ───────────────────────── */
/* Must match cellular_uart.c. Verified XIAO ESP32-C5 mapping from the
 * official pins_arduino.h: D4=GPIO23 (C5 TX → modem R), D5=GPIO24
 * (modem T → C5 RX). esp_modem reuses these same pins after the AT-phase
 * handoff (cellular_uart_deinit → esp_modem_new_dev). */
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
        if (cellular_uart_send_at("AT+CREG?", resp, sizeof(resp), AT_TIMEOUT_MS) == ESP_OK) {
            if (strstr(resp, ",1") || strstr(resp, ",5")) {
                ESP_LOGI(TAG, "network registered");
                break;
            }
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

    /* Power on GNSS now (single non-blocking command) so the receiver
     * warms up during PPP negotiation. SIM7600 dialect: AT+CGPS=1 starts
     * a standalone GPS session (NOT SIM800's AT+CGNSPWR). We do NOT block
     * waiting for a fix here — that kept detection→upload off the air for
     * up to 2 min. The fix is read opportunistically post-PPP over CMUX
     * (see modem_task). If GPS is already running, this returns ERROR,
     * which is harmless. */
    cellular_uart_send_at("AT+CGPS=1", resp, sizeof(resp), AT_TIMEOUT_MS);

    set_state(MODEM_STATE_REGISTERED);
    return true;
}

/* ── PPP phase via esp_modem ──────────────────────────────────────────────── */
static bool ppp_phase(void)
{
    set_state(MODEM_STATE_PPP_CONNECTING);

    /* Step 1: raise the link to CMUX_UART_BAUD before the esp_modem handoff.
     * AT+IPR=<rate> sets the SESSION baud only — it is NOT persisted to
     * NVRAM unless followed by AT&W, which we deliberately do NOT send. So a
     * power-cycle reverts the modem to its NVRAM-locked 115200, and
     * cellular_uart_wake_and_lock_baud() re-establishes 115200 each boot
     * before we raise it again here. The "OK" comes back at the old (115200)
     * rate, then the modem switches, so we send this while still at 115200,
     * give it a moment to apply, then hand the UART to esp_modem at the new
     * rate. If the modem rejects it, we fall back to 115200 for CMUX. */
    int cmux_baud = 115200;
    {
        char ipr_resp[RESP_BUF_SIZE];
        char ipr_cmd[24];
        snprintf(ipr_cmd, sizeof(ipr_cmd), "AT+IPR=%d", CMUX_UART_BAUD);
        if (cellular_uart_send_at(ipr_cmd, ipr_resp, sizeof(ipr_resp),
                                  AT_TIMEOUT_MS) == ESP_OK) {
            cmux_baud = CMUX_UART_BAUD;
            vTaskDelay(pdMS_TO_TICKS(300));  /* let the modem apply the new UART rate */
            ESP_LOGI(TAG, "UART raised to %d for CMUX headroom (session only)", cmux_baud);
        } else {
            ESP_LOGW(TAG, "AT+IPR=%d rejected — staying at 115200 for CMUX",
                     CMUX_UART_BAUD);
        }
    }

    /* Release our UART driver so esp_modem can take over */
    cellular_uart_deinit();

    char apn[64];
    read_nvs_string("cellular_apn", apn, sizeof(apn), "hologram");
    ESP_LOGI(TAG, "APN: %s", apn);

    esp_event_handler_register(IP_EVENT, IP_EVENT_PPP_GOT_IP, on_ip_event, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_PPP_LOST_IP, on_ip_event, NULL);

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
    dte_config.uart_config.baud_rate   = cmux_baud;   /* match the modem's session rate (Step 1) */

    esp_modem_dce_config_t dce_config = ESP_MODEM_DCE_DEFAULT_CONFIG(apn);
    s_dce = esp_modem_new_dev(ESP_MODEM_DCE_SIM7600, &dte_config,
                              &dce_config, s_netif);
    if (!s_dce) {
        ESP_LOGE(TAG, "esp_modem_new_dev(SIM7600) failed");
        return false;
    }

    /* CMUX (not plain DATA): multiplexes the PPP data channel and an AT
     * command channel over the one UART, so we can poll GNSS (AT+CGPSINFO)
     * via esp_modem_at() WITHOUT dropping the upload link. */
    esp_err_t err = esp_modem_set_mode(s_dce, ESP_MODEM_MODE_CMUX);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "set_mode(CMUX) failed: %s", esp_err_to_name(err));
        return false;
    }

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

        /* Connected. Opportunistically read GNSS over the CMUX command
         * channel until we get one fix (node is stationary → cache forever,
         * then stop polling). This never gates or interrupts uploads: the
         * uploader task is already POSTing over the PPP channel, and it
         * attaches node_position only once a fix exists. */
        while (s_ppp_connected) {
            if (!gnss_reader_have_fix() && s_dce) {
                char gnss_resp[RESP_BUF_SIZE];
                if (esp_modem_at(s_dce, "AT+CGPSINFO", gnss_resp,
                                 AT_TIMEOUT_MS) == ESP_OK) {
                    gnss_reader_submit_response(gnss_resp);
                }
            }
            vTaskDelay(pdMS_TO_TICKS(GNSS_POLL_INTERVAL_MS));
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
