#pragma once

#include "esp_err.h"
#include <stdbool.h>

/**
 * GNSS position reader.
 *
 * Position is read during the pre-PPP AT phase, time-boxed: modem_manager
 * powers GPS (AT+CGPS=1) and makes a short bounded AT+CGPSINFO poll over
 * cellular_uart (which it still owns before the esp_modem/PPP handoff),
 * feeding each raw response to gnss_reader_submit_response().  The node is
 * stationary, so once a fix is parsed it is cached forever.
 *
 * GNSS is NEVER on the detection→upload critical path: the poll is
 * bounded to a few seconds and PPP/uploads proceed regardless; uploads
 * attach node_position only once a fix exists.
 */
typedef struct {
    double  lat;
    double  lon;
    float   alt_m;          /* metres MSL */
    float   hdop;           /* 0 = unknown (AT+CGPSINFO carries no HDOP) */
    bool    valid;          /* true once first fix acquired */
} gnss_position_t;

/** Initialize state.  Call before the modem manager starts polling. */
esp_err_t gnss_reader_start(void);

/**
 * Parse one AT+CGPSINFO response and, on a valid fix, cache it.
 *
 * SIM7600 format:
 *   +CGPSINFO: <lat>,<N/S>,<lon>,<E/W>,<date>,<utc>,<alt>,<speed>,<course>
 * (lat/lon are NMEA ddmm.mmmm / dddmm.mmmm; a no-fix line is all empty
 * fields).  Safe to call repeatedly with junk — it ignores anything
 * that isn't a valid fix.
 *
 * @return true if a valid fix was parsed and cached, false otherwise.
 */
bool gnss_reader_submit_response(const char *resp);

/** @return true once a valid fix has been cached. */
bool gnss_reader_have_fix(void);

/**
 * Copy the cached position.
 * @return true if a valid fix is available, false otherwise.
 */
bool gnss_reader_get_position(gnss_position_t *out);
