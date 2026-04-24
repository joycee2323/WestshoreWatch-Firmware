#include "wifi_scanner.h"
#include "config.h"
#include "config_server.h"
#include "dns_server.h"
#include "nvs_config.h"
#include "odid_decoder.h"
#include "led.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_mac.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "WIFI_SCAN";

/* Fixed channel the softAP beacons on. Scanner must be pinned here
 * whenever a client is connected to the config portal, otherwise the
 * channel hopper drags the radio off-channel and the AP beacon drops. */
#define WSD_AP_CHANNEL 6

static QueueHandle_t  s_output_queue = NULL;
static bool           s_running      = false;
static bool           s_paused       = false;
static TaskHandle_t   s_hop_task     = NULL;
static SemaphoreHandle_t s_pause_mutex = NULL;

/* Number of stations currently associated with the config-portal AP.
 * Scanner auto-pauses while this is > 0. */
static int s_ap_sta_count = 0;


/* Soft-AP SSID built from MAC at startup */
static char s_ap_ssid[32] = "WestshoreWatch-0000";

/* ─────────────────────────────────────────────────────────────────────────────
 * 802.11 frame header
 * ───────────────────────────────────────────────────────────────────────────── */
#pragma pack(push, 1)
typedef struct {
    uint16_t frame_ctrl;
    uint16_t duration;
    uint8_t  addr1[6];
    uint8_t  addr2[6];
    uint8_t  addr3[6];
    uint16_t seq_ctrl;
} ieee80211_hdr_t;

typedef struct {
    uint8_t  category;
    uint8_t  oui[3];
    uint8_t  oui_subtype;
    uint8_t  action_code;
} vs_action_t;
#pragma pack(pop)

#define FC_TYPE_MASK       0x000C
#define FC_TYPE_MGMT       0x0000
#define FC_SUBTYPE_MASK    0x00F0
#define FC_SUBTYPE_BEACON  0x0080
#define FC_SUBTYPE_ACTION  0x00D0

/* ─────────────────────────────────────────────────────────────────────────────
 * Search beacon IEs for ODID vendor-specific IE
 * ───────────────────────────────────────────────────────────────────────────── */
static bool parse_beacon_ies(const uint8_t *body, uint16_t body_len,
                              uint8_t **odid_out, uint8_t *odid_len_out)
{
    if (body_len < 12) return false;
    const uint8_t *ie = body + 12;
    uint16_t remaining = body_len - 12;

    while (remaining >= 2) {
        uint8_t ie_id  = ie[0];
        uint8_t ie_len = ie[1];
        if (remaining < (uint16_t)(ie_len + 2)) break;

        if (ie_id == 0xDD && ie_len >= 5) {
            if (ie[2] == ODID_WIFI_OUI_0 &&
                ie[3] == ODID_WIFI_OUI_1 &&
                ie[4] == ODID_WIFI_OUI_2 &&
                ie[5] == ODID_WIFI_ACTION_TYPE) {
                *odid_out     = (uint8_t *)&ie[6];
                *odid_len_out = ie_len - 4;
                return true;
            }
            if (ie[2] == 0x26 && ie[3] == 0x37 && ie[4] == 0x12) {
                if (ie_len >= 6 && ie[5] == 0x0D) {
                    *odid_out     = (uint8_t *)&ie[6];
                    *odid_len_out = ie_len - 4;
                    return true;
                }
            }
        }
        ie        += ie_len + 2;
        remaining -= ie_len + 2;
    }
    return false;
}

/* ─────────────────────────────────────────────────────────────────────────────
 * Promiscuous callback
 * ───────────────────────────────────────────────────────────────────────────── */
static void promiscuous_cb(void *buf, wifi_promiscuous_pkt_type_t type)
{
    static uint32_t mgmt_count = 0;
    if (type == WIFI_PKT_MGMT) {
        mgmt_count++;
        if (mgmt_count % 50 == 0) {
            ESP_LOGD("WIFI_DBG", "Mgmt frames: %lu", (unsigned long)mgmt_count);
        }
    }

    if (type != WIFI_PKT_MGMT) return;

    const wifi_promiscuous_pkt_t *ppkt =
        (const wifi_promiscuous_pkt_t *)buf;
    const uint8_t *payload = ppkt->payload;
    uint16_t       pkt_len = ppkt->rx_ctrl.sig_len;
    int8_t         rssi    = ppkt->rx_ctrl.rssi;

    if (pkt_len < sizeof(ieee80211_hdr_t) + 1) return;
    if (ppkt->rx_ctrl.rx_state != 0) return;

    const ieee80211_hdr_t *hdr = (const ieee80211_hdr_t *)payload;
    uint16_t fc = hdr->frame_ctrl;
    if ((fc & FC_TYPE_MASK) != FC_TYPE_MGMT) return;

    uint16_t subtype  = fc & FC_SUBTYPE_MASK;
    uint8_t *odid_payload = NULL;
    uint8_t  odid_len     = 0;
    odid_source_t src;

    if (subtype == FC_SUBTYPE_BEACON) {
        const uint8_t *body     = payload + sizeof(ieee80211_hdr_t);
        uint16_t       body_len = pkt_len - sizeof(ieee80211_hdr_t);
        if (!parse_beacon_ies(body, body_len, &odid_payload, &odid_len)) return;
        src = (ppkt->rx_ctrl.rate <= 11) ? ODID_SRC_WIFI_B : ODID_SRC_WIFI_N;

    } else if (subtype == FC_SUBTYPE_ACTION) {
        const uint8_t *body     = payload + sizeof(ieee80211_hdr_t);
        uint16_t       body_len = pkt_len - sizeof(ieee80211_hdr_t);
        if (body_len < sizeof(vs_action_t) + 1) return;

        const vs_action_t *va = (const vs_action_t *)body;
        if (va->category    != ODID_WIFI_ACTION_CAT  ||
            va->oui[0]      != ODID_WIFI_OUI_0        ||
            va->oui[1]      != ODID_WIFI_OUI_1        ||
            va->oui[2]      != ODID_WIFI_OUI_2        ||
            va->oui_subtype != ODID_WIFI_ACTION_TYPE) return;

        odid_payload = (uint8_t *)body + sizeof(vs_action_t);
        odid_len     = body_len - sizeof(vs_action_t);
        src          = (ppkt->rx_ctrl.rate <= 11) ? ODID_SRC_WIFI_B : ODID_SRC_WIFI_N;
    } else {
        return;
    }

    if (!odid_payload || odid_len < 25) return;

    ESP_LOGI(TAG, "ODID frame! RSSI=%d len=%d", rssi, odid_len);
    led_flash_detection();

    odid_detection_t det;
    memset(&det, 0, sizeof(det));
    det.source = src;
    det.rssi   = rssi;
    memcpy(det.mac, hdr->addr2, 6);

    int ret = odid_parse_pack(odid_payload, odid_len, &det);
    if (ret == 0) ret = odid_parse_message(odid_payload, odid_len, &det);

    if (ret >= 0) {
        xQueueSend(s_output_queue, &det, pdMS_TO_TICKS(5));
    } else {
        static TickType_t last_dump = 0;
        TickType_t now = xTaskGetTickCount();
        if ((now - last_dump) > pdMS_TO_TICKS(5000)) {
            last_dump = now;
            ESP_LOGW(TAG, "ODID parse failed (ret=%d) RSSI=%d len=%d",
                     ret, rssi, odid_len);
        }
    }
}

/* ─────────────────────────────────────────────────────────────────────────────
 * Channel hopping task
 * ───────────────────────────────────────────────────────────────────────────── */
static void channel_hop_task(void *arg)
{
    uint8_t ch_min = g_config.ch_2g_start;
    uint8_t ch_max = g_config.ch_2g_stop;

    if (ch_min < 1)  ch_min = 1;
    if (ch_max > 13) ch_max = 13;
    if (ch_min > ch_max) ch_min = ch_max;

    uint8_t ch = ch_min;
    wifi_second_chan_t second = WIFI_SECOND_CHAN_NONE;

    while (s_running) {
        /* If paused, wait until resumed */
        if (s_paused) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        esp_wifi_set_channel(ch, second);
        vTaskDelay(pdMS_TO_TICKS(WSD_WIFI_DWELL_MS));
        if (++ch > ch_max) ch = ch_min;
    }
    vTaskDelete(NULL);
}

/* ─────────────────────────────────────────────────────────────────────────────
 * Build WestshoreWatch-XXXX SSID from MAC
 * ───────────────────────────────────────────────────────────────────────────── */
static void build_ap_ssid(char *buf, size_t len)
{
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);
    snprintf(buf, len, "WestshoreWatch-%02X%02X", mac[4], mac[5]);
}

/* ─────────────────────────────────────────────────────────────────────────────
 * SoftAP station-connect/disconnect event handler
 *
 * While any client is associated with the config AP, the radio must stay
 * pinned to WSD_AP_CHANNEL or the AP's beacons drop and the client loses
 * the portal. Pause the scanner on first connect, resume on last disconnect.
 * ───────────────────────────────────────────────────────────────────────────── */
static void ap_event_handler(void *arg, esp_event_base_t base,
                             int32_t event_id, void *event_data)
{
    if (base != WIFI_EVENT) return;

    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        s_ap_sta_count++;
        ESP_LOGI(TAG, "AP client connected (total=%d) — pausing scanner",
                 s_ap_sta_count);
        if (s_ap_sta_count == 1) {
            wifi_scanner_pause();
        }
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        if (s_ap_sta_count > 0) s_ap_sta_count--;
        ESP_LOGI(TAG, "AP client disconnected (total=%d)", s_ap_sta_count);
        if (s_ap_sta_count == 0) {
            ESP_LOGI(TAG, "No AP clients — resuming scanner");
            wifi_scanner_resume();
        }
    }
}

/* ─────────────────────────────────────────────────────────────────────────────
 * Public API
 * ───────────────────────────────────────────────────────────────────────────── */
esp_err_t wifi_scanner_start(QueueHandle_t output_queue)
{
    if (s_running) return ESP_OK;

    s_output_queue = output_queue;
    s_pause_mutex  = xSemaphoreCreateMutex();

    build_ap_ssid(s_ap_ssid, sizeof(s_ap_ssid));
    ESP_LOGI(TAG, "Config AP SSID: %s", s_ap_ssid);

    /* Network stack init */
    ESP_LOGI(TAG, "step: esp_netif_init");
    esp_netif_init();
    ESP_LOGI(TAG, "step: esp_event_loop_create_default");
    esp_event_loop_create_default();

    /* Create both netif interfaces before WiFi starts */
    ESP_LOGI(TAG, "step: create default AP+STA netifs");
    esp_netif_create_default_wifi_ap();
    esp_netif_create_default_wifi_sta();

    ESP_LOGI(TAG, "step: esp_wifi_init");
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_LOGI(TAG, "step: esp_wifi_set_storage(RAM)");
    esp_wifi_set_storage(WIFI_STORAGE_RAM);

    /* Register AP station connect/disconnect handler so the scanner
     * auto-pauses whenever a client is on the config portal. */
    ESP_LOGI(TAG, "step: register AP event handlers");
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, WIFI_EVENT_AP_STACONNECTED,
        &ap_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, WIFI_EVENT_AP_STADISCONNECTED,
        &ap_event_handler, NULL, NULL));

    /* APSTA mode: AP for config portal + STA for uplink */
    ESP_LOGI(TAG, "step: esp_wifi_set_mode(APSTA)");
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));

    /* Configure the soft-AP */
    wifi_config_t ap_cfg = {0};
    strlcpy((char *)ap_cfg.ap.ssid, s_ap_ssid, sizeof(ap_cfg.ap.ssid));
    ap_cfg.ap.ssid_len       = strlen(s_ap_ssid);
    strlcpy((char *)ap_cfg.ap.password, CFG_AP_PASS, sizeof(ap_cfg.ap.password));
    ap_cfg.ap.max_connection = 4;
    ap_cfg.ap.authmode       = (CFG_AP_PASS[0] == '\0')
                               ? WIFI_AUTH_OPEN
                               : WIFI_AUTH_WPA2_PSK;
    ap_cfg.ap.channel        = WSD_AP_CHANNEL;  /* keep AP on ch6 — best for ODID overlap */

    ESP_LOGI(TAG, "step: esp_wifi_set_config(AP)");
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_cfg));
    ESP_LOGI(TAG, "step: esp_wifi_start");
    ESP_ERROR_CHECK(esp_wifi_start());

    /* ESP32-C5 is dual-band; lock to 2.4 GHz since ODID beacons only
     * transmit on 2.4 GHz. Must be called AFTER esp_wifi_start — the API
     * returns ESP_ERR_WIFI_NOT_STARTED otherwise. */
    ESP_LOGI(TAG, "step: esp_wifi_set_band_mode(2G_ONLY)");
    esp_err_t bm_err = esp_wifi_set_band_mode(WIFI_BAND_MODE_2G_ONLY);
    if (bm_err != ESP_OK) {
        ESP_LOGW(TAG, "esp_wifi_set_band_mode failed: 0x%x (%s)",
                 bm_err, esp_err_to_name(bm_err));
    }

    /* Start the HTTP config server (non-blocking) */
    ESP_LOGI(TAG, "step: config_server_start_http");
    config_server_start_http();

    /* Start captive-portal DNS hijack so phones auto-open the config UI.
     * Must come after softAP is up and lwIP has the netif bound. */
    if (dns_server_start(CFG_AP_IP) != ESP_OK) {
        ESP_LOGW(TAG, "Captive DNS server failed to start");
    }

    /* Enable promiscuous mode */
    wifi_promiscuous_filter_t filter = {
        .filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT |
                       WIFI_PROMIS_FILTER_MASK_DATA,
    };
    esp_wifi_set_promiscuous_filter(&filter);
    esp_wifi_set_promiscuous_rx_cb(promiscuous_cb);
    esp_wifi_set_promiscuous(true);

    s_running = true;
    s_paused  = false;

    xTaskCreate(channel_hop_task, "wifi_hop",
                WSD_WIFI_SCAN_STACK, NULL,
                WSD_WIFI_SCAN_TASK_PRIO, &s_hop_task);

    ESP_LOGI(TAG, "Scanner started — AP '%s', ch %d-%d, %dms dwell",
             s_ap_ssid, g_config.ch_2g_start, g_config.ch_2g_stop,
             WSD_WIFI_DWELL_MS);
    ESP_LOGI(TAG, "Config portal: connect to '%s' → http://%s",
             s_ap_ssid, CFG_AP_IP);
    return ESP_OK;
}

void wifi_scanner_pause(void)
{
    if (!s_running || s_paused) return;
    s_paused = true;
    /* Wait for channel hop task to notice the pause */
    vTaskDelay(pdMS_TO_TICKS(WSD_WIFI_DWELL_MS + 50));
    esp_wifi_set_promiscuous(false);
    /* Pin radio to the softAP's channel so beacons keep going out while
     * the hop loop is idle. Without this the radio stays on whichever
     * channel the hopper was on last, and the AP beacon stops. */
    esp_wifi_set_channel(WSD_AP_CHANNEL, WIFI_SECOND_CHAN_NONE);
    ESP_LOGD(TAG, "Scanner paused (pinned to ch %d)", WSD_AP_CHANNEL);
}

void wifi_scanner_resume(void)
{
    if (!s_running || !s_paused) return;
    /* Re-enable promiscuous mode */
    wifi_promiscuous_filter_t filter = {
        .filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT |
                       WIFI_PROMIS_FILTER_MASK_DATA,
    };
    esp_wifi_set_promiscuous_filter(&filter);
    esp_wifi_set_promiscuous_rx_cb(promiscuous_cb);
    esp_wifi_set_promiscuous(true);
    s_paused = false;
    ESP_LOGD(TAG, "Scanner resumed");
}

void wifi_scanner_stop(void)
{
    if (!s_running) return;
    s_running = false;
    vTaskDelay(pdMS_TO_TICKS(WSD_WIFI_DWELL_MS + 50));
    s_hop_task = NULL;
    esp_wifi_set_promiscuous(false);
    esp_wifi_stop();
    esp_wifi_deinit();
    ESP_LOGI(TAG, "Scanner stopped");
}
