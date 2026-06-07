#include "cellular_uart.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "CELL_UART";

/* Recursive mutex serializing ALL access to the AT channel.  Created once in
 * cellular_uart_init() and never deleted — the native-AT-HTTP path keeps the
 * UART for the whole process lifetime.  Recursive so send helpers can take it
 * while a transaction (cellular_uart_lock) already holds it on the same task. */
static SemaphoreHandle_t s_at_mutex;

/* ── Pin assignments (XIAO ESP32-C5 header labels) ────────────────────────── */
#define CELL_UART_NUM       UART_NUM_1
/* XIAO ESP32-C5 header→GPIO map verified against the official Seeed
 * pins_arduino.h (symbol dump + physical pin-toggle test): D4=GPIO23,
 * D5=GPIO24. The earlier 6/7 values were a wrong guess (D4/D5 are not
 * sequential with the silicon GPIOs) — do NOT trust forum pinout tables.
 * GPIO23/24 are the XIAO's default I2C SDA/SCL pads, general-purpose and
 * UART-capable via the GPIO matrix; neither is a C5 strapping pin
 * (strapping = GPIO2/7/27/28). */
#define CELL_UART_TX_GPIO   23      /* header D4 — C5 TX → modem RX (modem connector "R" pin) */
#define CELL_UART_RX_GPIO   24      /* header D5 — modem TX (modem connector "T" pin) → C5 RX */
#define CELL_UART_BAUD      115200

/* RX ring must comfortably hold the largest HTTP body the modem ECHOES back in
 * AT+HTTPDATA download mode (it echoes the whole payload before its OK) plus
 * the trailing "\r\nOK\r\n" — otherwise the ring overflows mid-echo and the OK
 * is dropped. Detection bodies are capped at JSON_BUF_SIZE (4096), so 8192
 * gives 2× headroom even for a full 8-drone batch. TX holds the largest body
 * so a single write doesn't block. */
#define CELL_UART_RX_BUF_SIZE  8192
#define CELL_UART_TX_BUF_SIZE  4096
#define AT_LINE_MAX            512

esp_err_t cellular_uart_init(void)
{
    /* Create the AT mutex exactly once, before any send can race. */
    if (!s_at_mutex) {
        s_at_mutex = xSemaphoreCreateRecursiveMutex();
        if (!s_at_mutex) {
            ESP_LOGE(TAG, "failed to create AT mutex");
            return ESP_ERR_NO_MEM;
        }
    }

    /* Idempotent: if the driver is already installed (e.g. a re-init after a
     * failed bring-up), keep it — reinstalling would drop buffered RX and can
     * fail. */
    if (uart_is_driver_installed(CELL_UART_NUM)) {
        ESP_LOGI(TAG, "UART%d already installed — reusing", CELL_UART_NUM);
        return ESP_OK;
    }

    const uart_config_t cfg = {
        .baud_rate  = CELL_UART_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t err = uart_driver_install(CELL_UART_NUM, CELL_UART_RX_BUF_SIZE,
                                        CELL_UART_TX_BUF_SIZE, 0, NULL, 0);
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
    ESP_LOGI(TAG, "UART%d driver released", CELL_UART_NUM);
}

void cellular_uart_lock(void)
{
    if (s_at_mutex) xSemaphoreTakeRecursive(s_at_mutex, portMAX_DELAY);
}

void cellular_uart_unlock(void)
{
    if (s_at_mutex) xSemaphoreGiveRecursive(s_at_mutex);
}

esp_err_t cellular_uart_set_baud(uint32_t baud)
{
    esp_err_t err = uart_set_baudrate(CELL_UART_NUM, baud);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_set_baudrate(%lu) failed: %s",
                 (unsigned long)baud, esp_err_to_name(err));
        return err;
    }
    uart_flush_input(CELL_UART_NUM);
    return ESP_OK;
}

/* Format up to 16 bytes as space-separated hex for diagnostic logging. */
static void format_hex_preview(char *out, size_t out_size,
                                const char *buf, int n)
{
    int max = n > 16 ? 16 : n;
    int pos = 0;
    for (int i = 0; i < max && pos < (int)out_size - 4; i++) {
        pos += snprintf(out + pos, out_size - pos, "%02x ",
                        (unsigned char)buf[i]);
    }
    if (pos > 0) out[pos - 1] = '\0';
    else out[0] = '\0';
}

/* Send "AT\r" and accumulate any RX bytes for up to read_ms.
 * Returns total bytes received. */
static int wake_probe(char *resp, size_t resp_size, uint32_t read_ms)
{
    uart_flush_input(CELL_UART_NUM);
    uart_write_bytes(CELL_UART_NUM, "AT\r", 3);

    resp[0] = '\0';
    size_t total = 0;
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(read_ms);
    while (xTaskGetTickCount() < deadline) {
        int remaining_ms = (int)(deadline - xTaskGetTickCount()) *
                           portTICK_PERIOD_MS;
        if (remaining_ms <= 0) break;
        uint8_t byte;
        int got = uart_read_bytes(CELL_UART_NUM, &byte, 1,
                                  pdMS_TO_TICKS(remaining_ms < 50 ? remaining_ms : 50));
        if (got > 0 && total < resp_size - 1) {
            resp[total++] = (char)byte;
            resp[total] = '\0';
        }
    }
    return (int)total;
}

bool cellular_uart_wake_and_lock_baud(void)
{
    char resp[128];
    char hex[64];

    /* Phase 1: rapid wake burst at 115200 — SIM7600 autobaud needs this
     * cadence to lock onto the host rate. */
    ESP_LOGI(TAG, "wake burst at 115200 baud (15 × AT @ 200ms)");
    cellular_uart_set_baud(115200);
    for (int i = 0; i < 15; i++) {
        int n = wake_probe(resp, sizeof(resp), 300);
        if (n > 0) {
            format_hex_preview(hex, sizeof(hex), resp, n);
            ESP_LOGI(TAG, "modem responded at 115200 (%d bytes ascii='%.*s' hex=[%s])",
                     n, n > 60 ? 60 : n, resp, hex);
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    ESP_LOGW(TAG, "wake burst at 115200 got nothing — scanning other bauds");

    /* Phase 2: baud scan fallback.  If the modem was previously locked
     * to a non-default baud (e.g. by prior provisioning), find it and
     * re-lock to 115200 via AT+IPR=115200 + AT&W. */
    static const uint32_t bauds[] = { 9600, 57600, 38400, 19200, 460800 };
    for (size_t b = 0; b < sizeof(bauds) / sizeof(bauds[0]); b++) {
        uint32_t baud = bauds[b];
        ESP_LOGI(TAG, "trying %lu baud...", (unsigned long)baud);
        cellular_uart_set_baud(baud);
        for (int i = 0; i < 5; i++) {
            int n = wake_probe(resp, sizeof(resp), 300);
            if (n > 0) {
                format_hex_preview(hex, sizeof(hex), resp, n);
                ESP_LOGI(TAG, "modem responded at %lu baud (%d bytes ascii='%.*s' hex=[%s]) — locking to 115200",
                         (unsigned long)baud, n,
                         n > 60 ? 60 : n, resp, hex);
                /* Persist 115200 in modem NVRAM.  AT+IPR response is sent
                 * at the OLD baud, then the modem switches.  We give it a
                 * brief settle delay, swap UART, then AT&W at 115200. */
                uart_flush_input(CELL_UART_NUM);
                uart_write_bytes(CELL_UART_NUM, "AT+IPR=115200\r", 14);
                vTaskDelay(pdMS_TO_TICKS(500));
                cellular_uart_set_baud(115200);
                vTaskDelay(pdMS_TO_TICKS(100));
                uart_write_bytes(CELL_UART_NUM, "AT&W\r", 5);
                vTaskDelay(pdMS_TO_TICKS(500));
                uart_flush_input(CELL_UART_NUM);
                return true;
            }
            vTaskDelay(pdMS_TO_TICKS(200));
        }
    }
    ESP_LOGE(TAG, "ERROR — no response at any baud, check UART");
    cellular_uart_set_baud(115200);
    return false;
}

esp_err_t cellular_uart_send_at(const char *cmd, char *resp,
                                size_t resp_size, uint32_t timeout_ms)
{
    if (!cmd || !resp || resp_size < 2) return ESP_ERR_INVALID_ARG;

    cellular_uart_lock();

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
    esp_err_t result = ESP_ERR_TIMEOUT;
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
                result = ESP_OK;
                break;
            }
            if (strstr(resp, "\r\nERROR\r\n") || strstr(resp, "ERROR\n") ||
                strstr(resp, "NO CARRIER") ||
                strstr(resp, "+CME ERROR") || strstr(resp, "+CMS ERROR")) {
                ESP_LOGW(TAG, "RX: error response: %.*s",
                         (int)(total > 80 ? 80 : total), resp);
                result = ESP_FAIL;
                break;
            }
        }
    }

    if (result == ESP_ERR_TIMEOUT) {
        ESP_LOGW(TAG, "RX: timeout after %lums, got %u bytes: %.*s",
                 (unsigned long)timeout_ms, (unsigned)total,
                 (int)(total > 80 ? 80 : total), resp);
    }

    cellular_uart_unlock();
    return result;
}

/* Read bytes into resp (appending from *total), watching for ok/err substrings.
 * Shared core for send_expect (after a command) and collect (no command).
 * Does NOT flush or take the lock — callers handle those. */
static esp_err_t accumulate_until(const char *expect_ok, const char *expect_err,
                                  char *resp, size_t resp_size, size_t *total,
                                  uint32_t timeout_ms)
{
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
    while (xTaskGetTickCount() < deadline) {
        int remaining_ms = (int)(deadline - xTaskGetTickCount()) *
                           portTICK_PERIOD_MS;
        if (remaining_ms <= 0) break;

        uint8_t byte;
        int got = uart_read_bytes(CELL_UART_NUM, &byte, 1,
                                  pdMS_TO_TICKS(remaining_ms < 100 ? remaining_ms : 100));
        if (got <= 0) continue;

        if (*total < resp_size - 1) {
            resp[*total] = (char)byte;
            (*total)++;
            resp[*total] = '\0';
        }

        if (expect_ok && strstr(resp, expect_ok))   return ESP_OK;
        if (expect_err && strstr(resp, expect_err)) return ESP_FAIL;
    }
    return ESP_ERR_TIMEOUT;
}

esp_err_t cellular_uart_send_expect(const char *cmd,
                                    const char *expect_ok,
                                    const char *expect_err,
                                    char *resp, size_t resp_size,
                                    uint32_t timeout_ms)
{
    if (!cmd || !resp || resp_size < 2 || !expect_ok) return ESP_ERR_INVALID_ARG;

    cellular_uart_lock();

    uart_flush_input(CELL_UART_NUM);

    char at_buf[AT_LINE_MAX];
    int n = snprintf(at_buf, sizeof(at_buf), "%s\r\n", cmd);
    uart_write_bytes(CELL_UART_NUM, at_buf, n);
    ESP_LOGI(TAG, "TX: %s", cmd);

    resp[0] = '\0';
    size_t total = 0;
    esp_err_t r = accumulate_until(expect_ok, expect_err, resp, resp_size,
                                   &total, timeout_ms);
    if (r == ESP_OK)
        ESP_LOGI(TAG, "RX: '%s' (%u bytes)", expect_ok, (unsigned)total);
    else
        ESP_LOGW(TAG, "RX: expect '%s' %s (%u bytes: %.*s)", expect_ok,
                 r == ESP_FAIL ? "got error" : "timeout", (unsigned)total,
                 (int)(total > 80 ? 80 : total), resp);

    cellular_uart_unlock();
    return r;
}

int cellular_uart_write_raw(const uint8_t *data, size_t len)
{
    if (!data || len == 0) return 0;
    return uart_write_bytes(CELL_UART_NUM, data, len);
}

esp_err_t cellular_uart_collect_ex(const char *until_ok, const char *until_err,
                                   char *resp, size_t resp_size,
                                   uint32_t timeout_ms)
{
    if (!until_ok || !resp || resp_size < 2) return ESP_ERR_INVALID_ARG;

    cellular_uart_lock();
    resp[0] = '\0';
    size_t total = 0;
    /* No flush: deferred URC bytes / a fast synchronous reply may already be
     * buffered, and flushing would discard them. */
    esp_err_t r = accumulate_until(until_ok, until_err, resp, resp_size,
                                   &total, timeout_ms);
    cellular_uart_unlock();
    return r;
}

esp_err_t cellular_uart_collect(const char *until, char *resp,
                                size_t resp_size, uint32_t timeout_ms)
{
    return cellular_uart_collect_ex(until, NULL, resp, resp_size, timeout_ms);
}

/* Rolling window for drain_until. Must exceed the longest token we match
 * ("+HTTPACTION:" = 12). 64 leaves ample room and catches a token split across
 * two read chunks. */
#define DRAIN_WINDOW 64

esp_err_t cellular_uart_drain_until(const char *until_ok, const char *until_err,
                                    uint32_t timeout_ms, int *bytes_drained)
{
    if (!until_ok) return ESP_ERR_INVALID_ARG;

    cellular_uart_lock();

    char win[DRAIN_WINDOW + 1];
    int  wlen = 0;
    win[0] = '\0';

    uint8_t buf[128];
    int total = 0;
    esp_err_t r = ESP_ERR_TIMEOUT;
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);

    while (xTaskGetTickCount() < deadline) {
        int remaining_ms = (int)(deadline - xTaskGetTickCount()) * portTICK_PERIOD_MS;
        if (remaining_ms <= 0) break;

        /* Read in chunks (fast drain so the RX ring can't back up), then scan
         * byte-by-byte through the rolling window so a token is found wherever
         * it lands in the stream. */
        int got = uart_read_bytes(CELL_UART_NUM, buf, sizeof(buf),
                                  pdMS_TO_TICKS(remaining_ms < 50 ? remaining_ms : 50));
        if (got <= 0) continue;
        total += got;

        for (int i = 0; i < got; i++) {
            if (wlen == DRAIN_WINDOW) {
                memmove(win, win + 1, DRAIN_WINDOW - 1);
                wlen = DRAIN_WINDOW - 1;
            }
            win[wlen++] = (char)buf[i];
            win[wlen] = '\0';

            if (strstr(win, until_ok))                 { r = ESP_OK;   goto done; }
            if (until_err && strstr(win, until_err))   { r = ESP_FAIL; goto done; }
        }
    }

done:
    if (bytes_drained) *bytes_drained = total;
    cellular_uart_unlock();
    return r;
}
