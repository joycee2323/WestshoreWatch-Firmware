#include "cellular_uart.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "CELL_UART";

/* ── Pin assignments (XIAO ESP32-C5 header labels) ────────────────────────── */
#define CELL_UART_NUM       UART_NUM_1
#define CELL_UART_TX_GPIO   6       /* header D4 — C5 TX → modem RX */
#define CELL_UART_RX_GPIO   7       /* header D5 — modem TX → C5 RX */
#define CELL_UART_BAUD      115200

#define CELL_UART_BUF_SIZE  2048
#define AT_LINE_MAX         512

esp_err_t cellular_uart_init(void)
{
    const uart_config_t cfg = {
        .baud_rate  = CELL_UART_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t err = uart_driver_install(CELL_UART_NUM, CELL_UART_BUF_SIZE,
                                        CELL_UART_BUF_SIZE, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install failed: %s", esp_err_to_name(err));
        return err;
    }
    uart_param_config(CELL_UART_NUM, &cfg);
    uart_set_pin(CELL_UART_NUM, CELL_UART_TX_GPIO, CELL_UART_RX_GPIO,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    ESP_LOGI(TAG, "init: UART%d TX=GPIO%d RX=GPIO%d %d baud",
             CELL_UART_NUM, CELL_UART_TX_GPIO, CELL_UART_RX_GPIO,
             CELL_UART_BAUD);
    return ESP_OK;
}

void cellular_uart_deinit(void)
{
    uart_driver_delete(CELL_UART_NUM);
    ESP_LOGI(TAG, "UART%d driver released for esp_modem handoff", CELL_UART_NUM);
}

esp_err_t cellular_uart_send_at(const char *cmd, char *resp,
                                size_t resp_size, uint32_t timeout_ms)
{
    if (!cmd || !resp || resp_size < 2) return ESP_ERR_INVALID_ARG;

    /* Flush stale data from RX buffer */
    uart_flush_input(CELL_UART_NUM);

    /* Send command + CRLF */
    char at_buf[AT_LINE_MAX];
    int n = snprintf(at_buf, sizeof(at_buf), "%s\r\n", cmd);
    uart_write_bytes(CELL_UART_NUM, at_buf, n);
    ESP_LOGI(TAG, "TX: %s", cmd);

    /* Accumulate response lines until we see a final result code
     * (OK, ERROR, NO CARRIER, +CME ERROR, +CMS ERROR) or timeout. */
    resp[0] = '\0';
    size_t total = 0;
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);

    while (xTaskGetTickCount() < deadline) {
        uint8_t byte;
        int remaining_ms = (int)(deadline - xTaskGetTickCount()) *
                           portTICK_PERIOD_MS;
        if (remaining_ms <= 0) break;

        int got = uart_read_bytes(CELL_UART_NUM, &byte, 1,
                                  pdMS_TO_TICKS(remaining_ms < 100 ? remaining_ms : 100));
        if (got <= 0) continue;

        if (total < resp_size - 1) {
            resp[total++] = (char)byte;
            resp[total] = '\0';
        }

        /* Check for final result codes at end of accumulated response */
        if (byte == '\n' && total >= 2) {
            if (strstr(resp, "\r\nOK\r\n") || strstr(resp, "\r\nOK\n")) {
                ESP_LOGI(TAG, "RX: OK (%u bytes)", (unsigned)total);
                return ESP_OK;
            }
            if (strstr(resp, "\r\nERROR\r\n") || strstr(resp, "ERROR\n") ||
                strstr(resp, "NO CARRIER") ||
                strstr(resp, "+CME ERROR") || strstr(resp, "+CMS ERROR")) {
                ESP_LOGW(TAG, "RX: error response: %.*s",
                         (int)(total > 80 ? 80 : total), resp);
                return ESP_FAIL;
            }
        }
    }

    ESP_LOGW(TAG, "RX: timeout after %lums, got %u bytes: %.*s",
             (unsigned long)timeout_ms, (unsigned)total,
             (int)(total > 80 ? 80 : total), resp);
    return ESP_ERR_TIMEOUT;
}
