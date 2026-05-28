#pragma once

#include "esp_err.h"
#include <stddef.h>
#include <stdint.h>

/**
 * Low-level UART driver for the SIM7600G-H modem.
 *
 * UART1 on GPIO6 (TX, header D4) / GPIO7 (RX, header D5), 115200 baud.
 * Used for AT commands during pre-PPP setup.  Call cellular_uart_deinit()
 * before handing the UART to esp_modem for PPP.
 */

/** Install UART1 driver, configure GPIO6/GPIO7. */
esp_err_t cellular_uart_init(void);

/** Uninstall UART1 driver so esp_modem can take ownership. */
void cellular_uart_deinit(void);

/**
 * Send an AT command and wait for a final response (OK / ERROR / timeout).
 *
 * @param cmd        AT command string (without trailing \\r\\n)
 * @param resp       buffer for the full response (may contain URCs)
 * @param resp_size  size of resp buffer
 * @param timeout_ms maximum wait in milliseconds
 * @return ESP_OK on "OK", ESP_ERR_TIMEOUT on timeout,
 *         ESP_FAIL on "ERROR" or "NO CARRIER"
 */
esp_err_t cellular_uart_send_at(const char *cmd, char *resp,
                                size_t resp_size, uint32_t timeout_ms);

