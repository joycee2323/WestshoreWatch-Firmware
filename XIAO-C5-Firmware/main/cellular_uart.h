#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/**
 * Low-level UART driver for the SIM7600G-H modem.
 *
 * UART1 on GPIO23 (TX, header D4) / GPIO24 (RX, header D5), 115200 baud.
 * The native-AT-HTTP transport keeps the modem in AT command mode for the
 * whole session (no PPP/data-mode handoff), so this UART is shared between
 * the modem manager (wake/SIM/registration/GPS/NETOPEN) and the uploader
 * (HTTP POSTs).  ALL access is serialized through a single recursive mutex:
 * every send takes it for the duration of one command, and a caller running
 * a multi-command transaction (e.g. an HTTP POST) holds it across the whole
 * sequence with cellular_uart_lock()/unlock() so a GPS poll can never
 * interleave between, say, HTTPDATA and HTTPACTION.
 */

/** Install UART1 driver, configure GPIO23/GPIO24, create the AT mutex.
 *  Idempotent — safe to call again after a failed bring-up; the driver and
 *  mutex are created once and survive for the process lifetime. */
esp_err_t cellular_uart_init(void);

/** Uninstall UART1 driver.  Unused on the native-AT-HTTP path (we never hand
 *  the UART to esp_modem); kept for source compatibility. */
void cellular_uart_deinit(void);

/**
 * Acquire / release the AT-channel transaction lock.
 *
 * The lock is a RECURSIVE mutex: cellular_uart_send_at() and the other send
 * helpers take it internally, so a transaction that holds it via
 * cellular_uart_lock() may still call those helpers (same task re-entry).
 * Always pair lock with unlock on every path.  Use this to make a sequence
 * of AT commands atomic against other AT users (GPS polling, health checks).
 */
void cellular_uart_lock(void);
void cellular_uart_unlock(void);

/**
 * Send an AT command and accumulate the response until either expect_ok or
 * expect_err appears as a substring, or timeout elapses.  Generalizes
 * cellular_uart_send_at() for non-"OK" terminators such as the "DOWNLOAD"
 * prompt emitted by AT+HTTPDATA.
 *
 * @param expect_ok   success substring (e.g. "DOWNLOAD", "OK")
 * @param expect_err  error substring, or NULL to only watch for expect_ok
 * @return ESP_OK if expect_ok seen, ESP_FAIL if expect_err seen first,
 *         ESP_ERR_TIMEOUT otherwise.  resp always holds the bytes received.
 */
esp_err_t cellular_uart_send_expect(const char *cmd,
                                    const char *expect_ok,
                                    const char *expect_err,
                                    char *resp, size_t resp_size,
                                    uint32_t timeout_ms);

/**
 * Write raw bytes to the modem with no trailing CRLF and no RX flush.
 * Used to push the HTTP body after AT+HTTPDATA returns its DOWNLOAD prompt.
 * Caller must hold cellular_uart_lock().
 * @return bytes written, or -1 on error.
 */
int cellular_uart_write_raw(const uint8_t *data, size_t len);

/**
 * Read and accumulate RX bytes (NO command sent, NO initial flush) until the
 * `until` substring appears or timeout elapses.  Used to catch deferred
 * result URCs that arrive AFTER the command's "OK", e.g. "+NETOPEN: 0" after
 * AT+NETOPEN or "+HTTPACTION: 1,<status>,<len>" after AT+HTTPACTION.  Because
 * it does not flush, any URC bytes already buffered are not lost.
 * Caller must hold cellular_uart_lock() across the preceding send and this
 * read so nothing else consumes the URC.
 * @return ESP_OK if `until` seen, ESP_ERR_TIMEOUT otherwise.
 */
esp_err_t cellular_uart_collect(const char *until, char *resp,
                                size_t resp_size, uint32_t timeout_ms);

/**
 * Like cellular_uart_collect(), but also returns early with ESP_FAIL if
 * until_err appears first.  Used for a SYNCHRONOUS reply that may be OK or
 * ERROR — e.g. the OK the modem returns after the HTTPDATA body is received.
 * resp always holds the bytes received (NUL-terminated), so the caller can
 * log the modem's actual reply when neither token is seen (timeout).
 * @param until_err  error substring, or NULL to behave like collect()
 */
esp_err_t cellular_uart_collect_ex(const char *until_ok, const char *until_err,
                                   char *resp, size_t resp_size,
                                   uint32_t timeout_ms);

/**
 * Drain an UNBOUNDED RX stream until until_ok (or until_err) appears anywhere,
 * matching against a small rolling window so the total stream length is not
 * limited by any caller buffer.  Bytes are read in chunks and discarded after
 * the window slides past them — nothing is captured for the caller.
 *
 * This is the read path for the OK that follows an AT+HTTPDATA body: some
 * SIM7600 builds ECHO the entire payload back before the OK, so a fixed
 * capture buffer freezes mid-echo and never sees the trailing OK.  A rolling
 * window finds the OK regardless of how many KB of echo precede it.
 *
 * @param until_err    error substring, or NULL to only watch for until_ok
 * @param bytes_drained  optional out: total bytes read (incl. echo); may be NULL
 * @return ESP_OK if until_ok seen, ESP_FAIL if until_err seen first,
 *         ESP_ERR_TIMEOUT otherwise.  Caller must hold the relevant
 *         transaction lock if ordering vs other AT users matters.
 */
esp_err_t cellular_uart_drain_until(const char *until_ok, const char *until_err,
                                    uint32_t timeout_ms, int *bytes_drained);

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

