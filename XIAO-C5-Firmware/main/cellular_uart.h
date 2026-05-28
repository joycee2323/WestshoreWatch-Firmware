#pragma once

#include "esp_err.h"
#include <stdbool.h>
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

/** Change UART baud rate on the fly and flush stale RX data. */
esp_err_t cellular_uart_set_baud(uint32_t baud);

/**
 * Wake the SIM7600 via autobaud burst, scan common bauds if needed, and
 * leave the link at 115200.
 *
 * SIM7600 default IPR=0 (autobaud) requires several quick AT\\r cycles at
 * the host baud to lock on.  This function emits that burst, and if no
 * response is seen, scans other common bauds — when one responds, it
 * issues AT+IPR=115200 + AT&W to persist the host baud in modem NVRAM.
 *
 * @return true if the modem responded (link is now at 115200), false if
 *         no baud yielded a response.
 */
bool cellular_uart_wake_and_lock_baud(void);

