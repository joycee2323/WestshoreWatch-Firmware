#include "output.h"
#include "config.h"
#include "nvs_config.h"
#include "ble_relay.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <string.h>

static const char *TAG = "OUTPUT";

static QueueHandle_t s_queue = NULL;

/* ─────────────────────────────────────────────────────────────────────────────
 * Source name strings — match ds110 firmware conventions
 * ───────────────────────────────────────────────────────────────────────────── */
static const char *source_str(odid_source_t src)
{
    switch (src) {
        case ODID_SRC_BT_LEGACY: return "BT_LEGACY";
        case ODID_SRC_BT5:       return "BT5";
        case ODID_SRC_WIFI_B:    return "WIFI_B";
        case ODID_SRC_WIFI_N:    return "WIFI_N";
        default:                 return "UNKNOWN";
    }
}

/* ─────────────────────────────────────────────────────────────────────────────
 * UART0 initialisation
 * ───────────────────────────────────────────────────────────────────────────── */
static esp_err_t uart_init(void)
{
    uart_config_t cfg = {
        .baud_rate  = WSD_UART_PRIMARY_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t err;

    ESP_LOGI(TAG, "uart_init: uart_param_config UART%d @ %d baud",
             WSD_UART_PRIMARY_NUM, WSD_UART_PRIMARY_BAUD);
    err = uart_param_config(WSD_UART_PRIMARY_NUM, &cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_param_config failed: 0x%x (%s)",
                 err, esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "uart_init: uart_param_config OK");

    ESP_LOGI(TAG, "uart_init: uart_set_pin TX=GPIO%d RX=GPIO%d",
             WSD_UART_PRIMARY_TX, WSD_UART_PRIMARY_RX);
    err = uart_set_pin(WSD_UART_PRIMARY_NUM,
                       WSD_UART_PRIMARY_TX,
                       WSD_UART_PRIMARY_RX,
                       UART_PIN_NO_CHANGE,
                       UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_set_pin failed: 0x%x (%s)",
                 err, esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "uart_init: uart_set_pin OK");

    ESP_LOGI(TAG, "uart_init: uart_driver_install (rx_buf=%d tx_buf=%d)",
             WSD_UART_BUF_SIZE * 2, WSD_UART_BUF_SIZE * 2);
    err = uart_driver_install(WSD_UART_PRIMARY_NUM,
                              WSD_UART_BUF_SIZE * 2,
                              WSD_UART_BUF_SIZE * 2,
                              0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install failed: 0x%x (%s)",
                 err, esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "uart_init: uart_driver_install OK");

    ESP_LOGI(TAG, "uart_init: OK");
    return ESP_OK;
}

/* ─────────────────────────────────────────────────────────────────────────────
 * Escape a C string for JSON output (handles " and \ and control chars)
 * Returns number of bytes written to out (not including null terminator).
 * out must have room for at least 2*in_len + 1 bytes.
 * ───────────────────────────────────────────────────────────────────────────── */
static int json_escape(const char *in, char *out, int max_out)
{
    int written = 0;
    for (int i = 0; in[i] && written < max_out - 4; i++) {
        unsigned char c = (unsigned char)in[i];
        if (c == '"' || c == '\\') {
            out[written++] = '\\';
            out[written++] = c;
        } else if (c < 0x20) {
            /* Skip control characters */
        } else {
            out[written++] = c;
        }
    }
    out[written] = '\0';
    return written;
}

/* ─────────────────────────────────────────────────────────────────────────────
 * Format one detection event as a JSON line (full schema — UART)
 * ───────────────────────────────────────────────────────────────────────────── */
static int format_json(const odid_detection_t *d, char *buf, int max_len)
{
    char esc_uas[64], esc_desc[64], esc_op[48];
    json_escape(d->basic_id.uas_id,                esc_uas,  sizeof(esc_uas));
    json_escape(d->self_id.description,            esc_desc, sizeof(esc_desc));
    json_escape(d->operator_id.operator_id,        esc_op,   sizeof(esc_op));

    return snprintf(buf, max_len,
        "{"
        "\"ts\":%lu,"
        "\"src\":\"%s\","
        "\"rssi\":%d,"
        "\"mac\":\"%02X:%02X:%02X:%02X:%02X:%02X\","
        "\"id_type\":%d,"
        "\"ua_type\":%d,"
        "\"uas_id\":\"%s\","
        "\"status\":%d,"
        "\"lat\":%.7f,"
        "\"lon\":%.7f,"
        "\"alt_baro\":%.1f,"
        "\"alt_geo\":%.1f,"
        "\"height\":%.1f,"
        "\"speed\":%.2f,"
        "\"vspeed\":%.2f,"
        "\"heading\":%d,"
        "\"description\":\"%s\","
        "\"op_lat\":%.7f,"
        "\"op_lon\":%.7f,"
        "\"op_alt\":%.1f,"
        "\"operator_id\":\"%s\""
        "}\n",
        (unsigned long)xTaskGetTickCount(),
        source_str(d->source),
        (int)d->rssi,
        d->mac[0], d->mac[1], d->mac[2],
        d->mac[3], d->mac[4], d->mac[5],
        (int)d->basic_id.id_type,
        (int)d->basic_id.ua_type,
        esc_uas,
        (int)d->location.status,
        d->location.lat,
        d->location.lon,
        d->location.alt_baro,
        d->location.alt_geo,
        d->location.height,
        d->location.speed_horiz,
        d->location.speed_vert,
        (int)d->location.heading,
        esc_desc,
        d->system.operator_lat,
        d->system.operator_lon,
        d->system.operator_alt_geo,
        esc_op
    );
}

/* ─────────────────────────────────────────────────────────────────────────────
 * Compact JSON for GATT notify — fits in a single BLE ATT MTU.
 *
 * Fields:
 *   id      (only when has_basic_id)
 *   lat, lon, alt, spd, hdg
 *   op_lat, op_lon  (only when has_system)
 *
 * No trailing newline — GATT notify is a framed transport.
 * ───────────────────────────────────────────────────────────────────────────── */
static int format_json_compact(const odid_detection_t *d, char *buf, int max_len)
{
    int n = 0;
    n += snprintf(buf + n, max_len - n, "{");

    if (d->has_basic_id) {
        char esc_id[32];
        json_escape(d->basic_id.uas_id, esc_id, sizeof(esc_id));
        n += snprintf(buf + n, max_len - n, "\"id\":\"%s\",", esc_id);
    }

    n += snprintf(buf + n, max_len - n,
        "\"lat\":%.7f,\"lon\":%.7f,\"alt\":%.1f,\"spd\":%.2f,\"hdg\":%d",
        d->location.lat, d->location.lon, d->location.alt_geo,
        d->location.speed_horiz, (int)d->location.heading);

    if (d->has_system) {
        n += snprintf(buf + n, max_len - n,
            ",\"op_lat\":%.7f,\"op_lon\":%.7f",
            d->system.operator_lat, d->system.operator_lon);
    }

    n += snprintf(buf + n, max_len - n, "}");
    return n;
}

/* ─────────────────────────────────────────────────────────────────────────────
 * Output task
 * ───────────────────────────────────────────────────────────────────────────── */
static void output_task(void *arg)
{
    ESP_LOGI(TAG, "output_task: task entered");

    static char json_buf[WSD_JSON_MAX_LEN];
    static char gatt_buf[256];
    odid_detection_t det;

    ESP_LOGI(TAG, "Output task running on UART%d (TX=GPIO%d, %d baud)",
             WSD_UART_PRIMARY_NUM,
             WSD_UART_PRIMARY_TX,
             WSD_UART_PRIMARY_BAUD);

    /* Startup banner */
    const char *banner =
        "{\"info\":\"Westshore Drone Remote ID Receiver v1.0 ready\"}\n";
    uart_write_bytes(WSD_UART_PRIMARY_NUM, banner, strlen(banner));

    while (true) {
        if (xQueueReceive(s_queue, &det, portMAX_DELAY) == pdTRUE) {
            int len = format_json(&det, json_buf, sizeof(json_buf));
            if (len > 0 && len < (int)sizeof(json_buf)) {
                uart_write_bytes(WSD_UART_PRIMARY_NUM, json_buf, len);
            }

            /* Also push a compact variant out as a BLE manufacturer-specific
             * advertisement (handle 2, company 0x08FF) for the AirAware app. */
            int glen = format_json_compact(&det, gatt_buf, sizeof(gatt_buf));
            if (glen > 0 && glen < (int)sizeof(gatt_buf)) {
                ble_detection_advertise(gatt_buf, (size_t)glen);
            }
        }
    }
}

/* ─────────────────────────────────────────────────────────────────────────────
 * Public API
 * ───────────────────────────────────────────────────────────────────────────── */
esp_err_t output_task_start(QueueHandle_t detect_queue)
{
    ESP_LOGI(TAG, "output_task_start: enter (queue=%p)", detect_queue);
    s_queue = detect_queue;

    ESP_LOGI(TAG, "output_task_start: calling uart_init");
    esp_err_t err = uart_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART init failed: %d", err);
        return err;
    }
    ESP_LOGI(TAG, "output_task_start: uart_init returned OK");

    ESP_LOGI(TAG, "output_task_start: xTaskCreate stack=%d prio=%d",
             WSD_OUTPUT_STACK, WSD_OUTPUT_TASK_PRIO);
    BaseType_t ret = xTaskCreate(output_task, "output",
                                 WSD_OUTPUT_STACK, NULL,
                                 WSD_OUTPUT_TASK_PRIO, NULL);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create output task");
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "output_task_start: task created, returning OK");

    return ESP_OK;
}
