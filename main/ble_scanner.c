#include "ble_scanner.h"
#include "config.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "nimble/ble.h"
#include <string.h>

static const char *TAG = "BLE_SCAN";

static QueueHandle_t  s_output_queue = NULL;
static bool           s_running      = false;

/* ─────────────────────────────────────────────────────────────────────────────
 * Search AD structures for ODID service data (UUID 0xFFFA)
 * ───────────────────────────────────────────────────────────────────────────── */
static const uint8_t *find_odid_payload(const uint8_t *data, uint8_t data_len,
                                        uint8_t *payload_len_out)
{
    uint8_t i = 0;
    while (i + 1 < data_len) {
        uint8_t ad_len  = data[i];
        uint8_t ad_type = data[i + 1];
        if (ad_len == 0) break;

        if (ad_type == 0x16 && ad_len >= 3) {
            uint16_t uuid;
            memcpy(&uuid, &data[i + 2], 2);
            if (uuid == ODID_BLE_SERVICE_UUID) {
                *payload_len_out = ad_len - 3;
                return &data[i + 4];
            }
        }
        i += ad_len + 1;
    }
    return NULL;
}

/* ─────────────────────────────────────────────────────────────────────────────
 * GAP event callback — legacy BLE scan only (no extended)
 * ───────────────────────────────────────────────────────────────────────────── */
static int gap_event_cb(struct ble_gap_event *event, void *arg)
{
    if (event->type != BLE_GAP_EVENT_DISC) return 0;

    const uint8_t *adv_data = event->disc.data;
    uint8_t        adv_len  = event->disc.length_data;
    int8_t         rssi     = event->disc.rssi;
    uint8_t        mac[6];
    memcpy(mac, event->disc.addr.val, 6);

    uint8_t payload_len = 0;
    const uint8_t *payload = find_odid_payload(adv_data, adv_len, &payload_len);
    if (!payload || payload_len < 25) return 0;

    ESP_LOGI(TAG, "BLE ODID detected RSSI=%d", rssi);

    odid_detection_t det;
    memset(&det, 0, sizeof(det));
    det.source = ODID_SRC_BT_LEGACY;
    det.rssi   = rssi;
    memcpy(det.mac, mac, 6);

    uint8_t msg_type = (payload[0] >> 4) & 0x0F;
    int ret = (msg_type == ODID_MSG_PACK)
              ? odid_parse_pack(payload, payload_len, &det)
              : odid_parse_message(payload, payload_len, &det);

    if (ret >= 0) {
        xQueueSend(s_output_queue, &det, 0);
    }
    return 0;
}

/* ─────────────────────────────────────────────────────────────────────────────
 * NimBLE host task
 * ───────────────────────────────────────────────────────────────────────────── */
static void nimble_host_task(void *param)
{
    ESP_LOGI(TAG, "NimBLE host task started");
    nimble_port_run();
    nimble_port_freertos_deinit();
}

static void on_sync(void)
{
    ESP_LOGI(TAG, "BLE host synced — starting LEGACY scan");

    uint8_t own_addr_type;
    ble_hs_id_infer_auto(0, &own_addr_type);

    /* Legacy scan only — this allows simultaneous legacy advertising
     * for the relay without BLE_ERR_CMD_DISALLOWED */
    struct ble_gap_disc_params lp = {
        .itvl              = (WSD_BLE_SCAN_ITVL_MS * 1000) / 625,
        .window            = (WSD_BLE_SCAN_WIN_MS  * 1000) / 625,
        .filter_policy     = BLE_HCI_SCAN_FILT_NO_WL,
        .limited           = 0,
        .passive           = 1,
        .filter_duplicates = 0,
    };

    int rc = ble_gap_disc(own_addr_type, BLE_HS_FOREVER, &lp,
                          gap_event_cb, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_disc failed: %d", rc);
    } else {
        ESP_LOGI(TAG, "BLE legacy scan active");
    }
}

static void on_reset(int reason)
{
    ESP_LOGE(TAG, "BLE host reset (reason %d)", reason);
}

/* ─────────────────────────────────────────────────────────────────────────────
 * Public API
 * ───────────────────────────────────────────────────────────────────────────── */
esp_err_t ble_scanner_start(QueueHandle_t output_queue)
{
    if (s_running) return ESP_OK;
    s_output_queue = output_queue;

    esp_err_t err = nimble_port_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nimble_port_init failed: %d", err);
        return err;
    }

    ble_hs_cfg.sync_cb  = on_sync;
    ble_hs_cfg.reset_cb = on_reset;

    nimble_port_freertos_init(nimble_host_task);
    s_running = true;
    ESP_LOGI(TAG, "BLE scanner started (legacy mode — compatible with relay)");
    return ESP_OK;
}

void ble_scanner_stop(void)
{
    if (!s_running) return;
    ble_gap_disc_cancel();
    nimble_port_stop();
    s_running = false;
}
