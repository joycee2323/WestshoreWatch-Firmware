#include "gnss_reader.h"
#include "esp_log.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

static const char *TAG = "GNSS";

static gnss_position_t s_position;
static volatile bool   s_valid;

bool gnss_reader_get_position(gnss_position_t *out)
{
    if (!s_valid) return false;
    *out = s_position;
    return true;
}

bool gnss_reader_have_fix(void)
{
    return s_valid;
}

/* NMEA ddmm.mmmm (or dddmm.mmmm) → signed decimal degrees.
 * deg = integer part / 100, the remainder is minutes. Works for both
 * 2-digit (lat) and 3-digit (lon) degree fields. */
static double nmea_to_decimal(double ddmm)
{
    double deg = trunc(ddmm / 100.0);
    double min = ddmm - deg * 100.0;
    return deg + min / 60.0;
}

/* Advance past the next ',' in *pp. Returns false if none. */
static bool skip_field(const char **pp)
{
    const char *c = strchr(*pp, ',');
    if (!c) return false;
    *pp = c + 1;
    return true;
}

/**
 * Parse SIM7600 AT+CGPSINFO:
 *   +CGPSINFO: <lat>,<N/S>,<lon>,<E/W>,<date>,<utc>,<alt>,<speed>,<course>
 * No-fix lines are "+CGPSINFO: ,,,,,,,," (empty fields).
 * CGPSINFO has no HDOP field, so hdop is reported as 0 (unknown).
 */
static bool parse_cgpsinfo(const char *resp)
{
    const char *p = strstr(resp, "+CGPSINFO:");
    if (!p) return false;
    p = strchr(p, ':');
    if (!p) return false;
    p++;
    while (*p == ' ') p++;

    if (*p == ',') return false;        /* empty lat → no fix */

    char *end;
    double lat_raw = strtod(p, &end);
    if (end == p || *end != ',') return false;
    p = end + 1;

    char ns = *p;
    if (ns != 'N' && ns != 'S') return false;
    if (!skip_field(&p)) return false;  /* past N/S */

    double lon_raw = strtod(p, &end);
    if (end == p || *end != ',') return false;
    p = end + 1;

    char ew = *p;
    if (ew != 'E' && ew != 'W') return false;
    if (!skip_field(&p)) return false;  /* past E/W */
    if (!skip_field(&p)) return false;  /* past date */
    if (!skip_field(&p)) return false;  /* past utc time */

    float alt = strtof(p, &end);        /* altitude (m MSL) */

    double lat = nmea_to_decimal(lat_raw);
    double lon = nmea_to_decimal(lon_raw);
    if (ns == 'S') lat = -lat;
    if (ew == 'W') lon = -lon;

    if (lat == 0.0 && lon == 0.0) return false;
    if (lat < -90 || lat > 90 || lon < -180 || lon > 180) return false;

    s_position.lat   = lat;
    s_position.lon   = lon;
    s_position.alt_m = alt;
    s_position.hdop  = 0.0f;             /* CGPSINFO carries no HDOP */
    s_position.valid = true;
    s_valid = true;

    ESP_LOGI(TAG, "fix acquired: %.6f, %.6f  alt=%.1fm", lat, lon, alt);
    return true;
}

bool gnss_reader_submit_response(const char *resp)
{
    if (!resp) return false;
    return parse_cgpsinfo(resp);
}

esp_err_t gnss_reader_start(void)
{
    memset(&s_position, 0, sizeof(s_position));
    s_valid = false;
    ESP_LOGI(TAG, "GNSS reader initialized (time-boxed pre-PPP CGPSINFO)");
    return ESP_OK;
}
