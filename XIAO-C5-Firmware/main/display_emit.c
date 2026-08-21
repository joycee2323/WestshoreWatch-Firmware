#include "display_emit.h"
#include "config.h"

#if WSD_DISPLAY_EMIT

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "gnss_reader.h"

static const char *TAG = "DISPLAY_EMIT";

/* Shared by both line producers (the detection tee and the 1 Hz node
 * ticker) — one UART, one writer task, one drop-on-full discipline. Depth 8
 * per the detection-tee spec; the node line only adds ~1 item/sec on top,
 * so sharing the queue doesn't meaningfully compete with detection bursts. */
#define EMIT_QUEUE_DEPTH    8
#define EMIT_LINE_MAX       96      /* headroom over the worst-case formatted line */
#define EMIT_TASK_PRIO      1       /* lowest application task priority in this
                                      * firmware — below WSD_OUTPUT_TASK_PRIO (3),
                                      * BLE (4), WiFi (5), distributor (5). Can
                                      * never delay anything on the detection or
                                      * upload path by preemption. */
#define EMIT_TASK_STACK     3072
#define EMIT_NODE_PERIOD_MS 1000

typedef enum {
    EMIT_LINE_DETECTION,
    EMIT_LINE_NODE,
} emit_line_kind_t;

/* Only what's needed for a "D," line — deliberately NOT the full
 * odid_detection_t, so the queue stays small and copying it can never
 * itself become a stall risk. */
typedef struct {
    char            id[21];        /* sanitized uas_id, <=20 chars + NUL */
    odid_source_t   source;
    int8_t          rssi;
    bool            has_location;
    float           drone_lat;
    float           drone_lon;
    bool            has_system;
    double          op_lat;
    double          op_lon;
} emit_detection_t;

typedef struct {
    double      node_lat;
    double      node_lon;
    bool        gps_valid;
    uint16_t    seq;
} emit_node_t;

typedef struct {
    emit_line_kind_t kind;
    union {
        emit_detection_t det;
        emit_node_t      node;
    };
} emit_item_t;

static QueueHandle_t s_emit_queue = NULL;

static const char *src_str(odid_source_t s)
{
    switch (s) {
    case ODID_SRC_BT_LEGACY: return "BTL";
    case ODID_SRC_BT5:       return "BT5";
    case ODID_SRC_WIFI_B:    return "WIFIB";
    case ODID_SRC_WIFI_N:    return "WIFIN";
    default:                 return "";
    }
}

/* XOR checksum over every byte strictly between the leading type letter
 * ('D'/'N') and the '*' — i.e. the caller passes the body starting at the
 * comma right after the letter, ending right before '*'. Matches the
 * NMEA-style "XOR of bytes between $ and *" convention. */
static uint8_t line_checksum(const char *body, size_t len)
{
    uint8_t ck = 0;
    for (size_t i = 0; i < len; i++) {
        ck ^= (uint8_t)body[i];
    }
    return ck;
}

/* One-shot best-effort write. TX-only, no flow control, no RX wait — a
 * detached/absent display cannot make this block: uart_write_bytes here is
 * bounded by the driver's TX ring + wire time only, never by anything the
 * far end does or doesn't do. This is the only function in this file that
 * touches the UART peripheral (WSD_UART_PRIMARY_NUM / GPIO11 / D6) — never
 * UART1, never cellular_uart.c, never s_at_mutex. */
static void uart_send_line(const char *line, size_t len)
{
    uart_write_bytes(WSD_UART_PRIMARY_NUM, line, len);
}

static void format_and_send_detection(const emit_detection_t *d)
{
    char body[EMIT_LINE_MAX];
    int n = 0;

    /* bit0 = has_location (drone_lat/lon populated below). bit1 = has_system
     * ALONE (op_lat/lon populated below) — deliberately NOT OR'd with
     * has_operator_id. The display uses bit1 to decide whether
     * operator-relative distance/bearing is computable, which needs actual
     * op_lat/op_lon coordinates; an operator-ID string with no coordinates
     * can't be ranged, so it must not set this bit. */
    uint8_t flags = 0;
    if (d->has_location) flags |= 0x01;
    if (d->has_system)   flags |= 0x02;

    uint32_t t_ms = (uint32_t)(esp_timer_get_time() / 1000);

    /* body = everything after "D" and before "*" — starts with the comma
     * that follows the letter in the wire format. */
    n += snprintf(body + n, sizeof(body) - n, ",%s,%s,%d,",
                  d->id, src_str(d->source), (int)d->rssi);
    if (n < 0 || (size_t)n >= sizeof(body)) goto truncated;

    if (d->has_location) {
        n += snprintf(body + n, sizeof(body) - n, "%.6f,%.6f,",
                      (double)d->drone_lat, (double)d->drone_lon);
    } else {
        n += snprintf(body + n, sizeof(body) - n, ",,");
    }
    if (n < 0 || (size_t)n >= sizeof(body)) goto truncated;

    if (d->has_system) {
        n += snprintf(body + n, sizeof(body) - n, "%.6f,%.6f,",
                      d->op_lat, d->op_lon);
    } else {
        n += snprintf(body + n, sizeof(body) - n, ",,");
    }
    if (n < 0 || (size_t)n >= sizeof(body)) goto truncated;

    n += snprintf(body + n, sizeof(body) - n, "%02X,%lu",
                  flags, (unsigned long)t_ms);
    if (n < 0 || (size_t)n >= sizeof(body)) goto truncated;

    {
        uint8_t ck = line_checksum(body, (size_t)n);
        char line[EMIT_LINE_MAX + 8];
        int ln = snprintf(line, sizeof(line), "D%s*%02X\n", body, ck);
        if (ln < 0 || (size_t)ln >= sizeof(line)) goto truncated;
        uart_send_line(line, (size_t)ln);
    }
    return;

truncated:
    ESP_LOGW(TAG, "detection line would overflow buffer — dropped, not sent malformed");
}

static void format_and_send_node(const emit_node_t *nd)
{
    char body[EMIT_LINE_MAX];
    uint32_t t_ms = (uint32_t)(esp_timer_get_time() / 1000);
    int n;

    if (nd->gps_valid) {
        n = snprintf(body, sizeof(body), ",%.6f,%.6f,1,%u,%lu",
                     nd->node_lat, nd->node_lon,
                     (unsigned)nd->seq, (unsigned long)t_ms);
    } else {
        n = snprintf(body, sizeof(body), ",,,0,%u,%lu",
                     (unsigned)nd->seq, (unsigned long)t_ms);
    }
    if (n < 0 || (size_t)n >= sizeof(body)) {
        ESP_LOGW(TAG, "node line would overflow buffer — dropped, not sent malformed");
        return;
    }

    uint8_t ck = line_checksum(body, (size_t)n);
    char line[EMIT_LINE_MAX + 8];
    int ln = snprintf(line, sizeof(line), "N%s*%02X\n", body, ck);
    if (ln < 0 || (size_t)ln >= sizeof(line)) {
        ESP_LOGW(TAG, "node line would overflow buffer — dropped, not sent malformed");
        return;
    }
    uart_send_line(line, (size_t)ln);
}

/* Sole reader of s_emit_queue; sole writer of the UART. Lowest priority in
 * the firmware — a slow/backed-up UART write here only delays the NEXT
 * queue item, never anything upstream, since producers only ever do a
 * zero-timeout xQueueSend. */
static void drain_task(void *arg)
{
    (void)arg;
    emit_item_t item;
    while (true) {
        if (xQueueReceive(s_emit_queue, &item, portMAX_DELAY) == pdTRUE) {
            if (item.kind == EMIT_LINE_DETECTION) {
                format_and_send_detection(&item.det);
            } else {
                format_and_send_node(&item.node);
            }
        }
    }
}

/* ~1 Hz node-position ticker. Emits even with zero detections and even
 * with no GPS fix yet (gps_valid=0, lat/lon fields empty) so the display
 * can distinguish "node alive, no fix" from "link dead" purely from
 * whether N lines keep arriving at all. */
static void node_task(void *arg)
{
    (void)arg;
    uint16_t seq = 0;
    while (true) {
        gnss_position_t pos = {0};
        bool have_fix = gnss_reader_get_position(&pos);

        emit_item_t item = {
            .kind = EMIT_LINE_NODE,
            .node = {
                .node_lat  = have_fix ? pos.lat : 0.0,
                .node_lon  = have_fix ? pos.lon : 0.0,
                .gps_valid = have_fix,
                .seq       = seq++,
            },
        };
        /* Same drop-on-full discipline as the detection tee — never blocks. */
        xQueueSend(s_emit_queue, &item, 0);

        vTaskDelay(pdMS_TO_TICKS(EMIT_NODE_PERIOD_MS));
    }
}

esp_err_t display_emit_init(void)
{
    s_emit_queue = xQueueCreate(EMIT_QUEUE_DEPTH, sizeof(emit_item_t));
    if (!s_emit_queue) {
        ESP_LOGE(TAG, "queue create failed");
        return ESP_ERR_NO_MEM;
    }

    const uart_config_t cfg = {
        .baud_rate  = WSD_UART_PRIMARY_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    esp_err_t err = uart_param_config(WSD_UART_PRIMARY_NUM, &cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_param_config failed: %s", esp_err_to_name(err));
        return err;
    }

    /* TX only. RX left UART_PIN_NO_CHANGE so the GPIO matrix never routes
     * anything onto GPIO12 — that pin is the live status-LED YELLOW line
     * (gpio_set_level in status_led.c); routing UART RX there would fight
     * that driver. WSD_UART_PRIMARY_RX (also GPIO12, from config.h) is
     * deliberately NOT used here for the same reason. */
    err = uart_set_pin(WSD_UART_PRIMARY_NUM, WSD_UART_PRIMARY_TX,
                        UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
                        UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_set_pin failed: %s", esp_err_to_name(err));
        return err;
    }

    /* Small TX-only ring; nothing is ever received on this UART. */
    err = uart_driver_install(WSD_UART_PRIMARY_NUM, WSD_UART_BUF_SIZE, 0, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install failed: %s", esp_err_to_name(err));
        return err;
    }

    xTaskCreate(drain_task, "display_emit_drain", EMIT_TASK_STACK, NULL,
                EMIT_TASK_PRIO, NULL);
    xTaskCreate(node_task, "display_emit_node", EMIT_TASK_STACK, NULL,
                EMIT_TASK_PRIO, NULL);

    ESP_LOGI(TAG, "init: UART%d TX=GPIO%d (RX unused) %d baud, queue depth %d",
             WSD_UART_PRIMARY_NUM, WSD_UART_PRIMARY_TX,
             WSD_UART_PRIMARY_BAUD, EMIT_QUEUE_DEPTH);
    return ESP_OK;
}

void display_emit_submit_detection(const odid_detection_t *det)
{
    if (!s_emit_queue || !det) return;

    char id[21] = {0};
    if (det->has_basic_id) {
        size_t w = 0;
        for (size_t i = 0; det->basic_id.uas_id[i] != '\0' && w < 20; i++) {
            char c = det->basic_id.uas_id[i];
            if (c == ',' || c == '\r' || c == '\n') continue;
            id[w++] = c;
        }
        id[w] = '\0';
    }

    emit_item_t item = {
        .kind = EMIT_LINE_DETECTION,
        .det = {
            .source          = det->source,
            .rssi            = det->rssi,
            .has_location    = det->has_location,
            .drone_lat       = det->location.lat,
            .drone_lon       = det->location.lon,
            .has_system      = det->has_system,
            .op_lat          = det->system.operator_lat,
            .op_lon          = det->system.operator_lon,
        },
    };
    memcpy(item.det.id, id, sizeof(id));

    /* Never blocks the caller (distributor_task, on the detection critical
     * path). Timeout 0 = drop silently if the queue is already full. */
    xQueueSend(s_emit_queue, &item, 0);
}

#else /* !WSD_DISPLAY_EMIT — feature compiled out entirely, zero footprint */

esp_err_t display_emit_init(void) { return ESP_OK; }
void display_emit_submit_detection(const odid_detection_t *det) { (void)det; }

#endif /* WSD_DISPLAY_EMIT */
