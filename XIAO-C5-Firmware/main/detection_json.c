#include "detection_json.h"

#include <math.h>
#include <stdio.h>

/* ── JSON serialization ───────────────────────────────────────────────────── */
/* Emit ONE drone object in the canonical schema the backend ingest path reads
 * (routes/nodes.js + routes/detections.js) and the Android node sends
 * (DetectionUploader.kt): {id, lat, lon, alt, spd, hdg, status, height, vspd,
 * op_lat, op_lon}.
 *
 * CRITICAL: the backend keys on `drone.id` and does `if (!uas_id) continue;`,
 * so the field MUST be "id" (not "uas_id") or the drone is silently dropped
 * (HTTP 200, stored:0). Likewise altitude/speed/heading must be "alt"/"spd"/
 * "hdg". `alt` is the GEODETIC altitude (Android maps alt = altGeo).
 *
 * status / height / vspd feed the backend's grounded-aircraft state, with the
 * same names and units as the Sentinel-Pi upload (translator.py) and the
 * phone relay: status = ODID operational status 0-4, height = metres,
 * vspd = vertical speed m/s (positive up). Invalid / unknown values are sent
 * as null (status 5-15, alt/height -1000 m, |vspd| > 62 m/s); the backend
 * fails open on null.
 *
 * Fields the backend ignores (id_type, baro alt, rssi, mac, …) are omitted to
 * keep the cellular payload small. `ts` (ODID self-clock) and `nickname` are
 * omitted — the firmware doesn't have them; the backend treats them as null
 * (same as the Sentinel path), so the coalescer/stale gate behaves identically. */

static int put_float_or_null(char *buf, size_t sz, const char *key, float v, int valid)
{
    if (valid && isfinite(v)) return snprintf(buf, sz, ",\"%s\":%.1f", key, (double)v);
    return snprintf(buf, sz, ",\"%s\":null", key);
}

/* Remaining space after n characters, never wrapping below zero. */
static size_t rem(size_t sz, int n)
{
    return (n < 0 || (size_t)n >= sz) ? 0 : sz - (size_t)n;
}

int detection_json_format(const odid_detection_t *det, char *buf, size_t sz)
{
    int n = 0;

    /* id is mandatory — without it the backend skips the drone. */
    n += snprintf(buf + n, rem(sz, n), "{\"id\":\"%s\"",
                  det->has_basic_id ? det->basic_id.uas_id : "");

    if (det->has_location) {
        const odid_location_t *loc = &det->location;
        n += snprintf(buf + n, rem(sz, n), ",\"lat\":%.7f,\"lon\":%.7f",
                      (double)loc->lat, (double)loc->lon);
        n += put_float_or_null(buf + n, rem(sz, n), "alt", loc->alt_geo,
                               loc->alt_geo > DETJSON_ALT_INVALID_MAX_M);
        /* spd / hdg: null when the drone sent "unknown" (255 m/s / 361°). */
        if (loc->speed_valid)
            n += snprintf(buf + n, rem(sz, n), ",\"spd\":%.2f", (double)loc->speed_horiz);
        else
            n += snprintf(buf + n, rem(sz, n), ",\"spd\":null");
        if (loc->heading_valid)
            n += snprintf(buf + n, rem(sz, n), ",\"hdg\":%u", (unsigned)loc->heading);
        else
            n += snprintf(buf + n, rem(sz, n), ",\"hdg\":null");
        if ((int)loc->status <= DETJSON_STATUS_MAX_VALID) {
            n += snprintf(buf + n, rem(sz, n), ",\"status\":%d", (int)loc->status);
        } else {
            n += snprintf(buf + n, rem(sz, n), ",\"status\":null");
        }
        n += put_float_or_null(buf + n, rem(sz, n), "height", loc->height,
                               loc->height > DETJSON_ALT_INVALID_MAX_M);
        n += put_float_or_null(buf + n, rem(sz, n), "vspd", loc->speed_vert,
                               fabsf(loc->speed_vert) <= DETJSON_VSPEED_MAX_VALID);
        /* The Location message as received + which decoder produced spd/hdg:
         * the backend stores loc_raw and marks speed/heading reliable for a
         * known decoder (routes/nodes.js via services/rawLocation.js). */
        if (loc->raw_valid) {
            n += snprintf(buf + n, rem(sz, n), ",\"loc_raw\":\"");
            for (size_t i = 0; i < sizeof loc->raw; i++)
                n += snprintf(buf + n, rem(sz, n), "%02x", loc->raw[i]);
            n += snprintf(buf + n, rem(sz, n), "\",\"decoder\":\"%s\"", ODID_DECODER_VERSION);
        }
    }

    if (det->has_system) {
        n += snprintf(buf + n, rem(sz, n), ",\"op_lat\":%.7f,\"op_lon\":%.7f",
                      (double)det->system.operator_lat, (double)det->system.operator_lon);
    }

    n += snprintf(buf + n, rem(sz, n), "}");
    return n;
}
