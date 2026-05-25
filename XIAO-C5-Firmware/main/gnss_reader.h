#pragma once

#include "esp_err.h"
#include <stdbool.h>

/**
 * GNSS position reader.
 *
 * Fix is acquired ONCE during the pre-PPP AT command phase (before
 * the UART is handed to esp_modem).  The node is stationary so the
 * cached position is used for all subsequent uploads.  This avoids
 * switching the modem out of PPP data mode for AT commands.
 */
typedef struct {
    double  lat;
    double  lon;
    float   alt_m;          /* metres MSL */
    float   hdop;
    bool    valid;          /* true once first fix acquired */
} gnss_position_t;

/** Initialize state.  Call before gnss_reader_acquire_fix(). */
esp_err_t gnss_reader_start(void);

/**
 * Acquire a GNSS fix via AT+CGPSINF, blocking.
 *
 * Must be called while cellular_uart is still active (pre-PPP).
 * Polls every 5s up to max_attempts times.  Best-effort: if no fix
 * is acquired, uploads proceed without node_position.
 */
void gnss_reader_acquire_fix(int max_attempts);

/**
 * Copy the cached position.
 * @return true if a valid fix is available, false otherwise.
 */
bool gnss_reader_get_position(gnss_position_t *out);
