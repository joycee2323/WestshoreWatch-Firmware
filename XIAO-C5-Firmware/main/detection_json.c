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
 * ts = the drone's own ODID Location timestamp, tenths of a second into the
 * UTC hour (0..35999), omitted when the drone sends "unknown" (0xFFFF). The
 * backend's stale gate (services/odidStaleGate.js) judges frame age with it,
 * as it does for the phone app: older than 5 min → too_old, a repeated ts →
 * unchanged. It is inside odid_detection_t, so it survives the SPIFFS spool.
 * Risk: a drone with a wrong clock can make fresh frames look stale.
 *
 * Fields the backend ignores (id_type, baro alt, rssi, mac, …) are omitted to
 * keep the cellular payload small; `nickname` is not sent. The uploader adds
 * age_ms (receive → payload build) after this object: see upload_batch.c. */

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

bool detection_json_ts_valid(const odid_detection_t *det)
{
    return det->has_location && det->location.timestamp < DETJSON_TS_MAX;
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
        n += snprintf(buf + n, rem(sz, n), ",\"spd\":%.2f,\"hdg\":%u",
                      (double)loc->speed_horiz,  /* spd = horizontal speed */
                      (unsigned)loc->heading);   /* hdg */
        if ((int)loc->status <= DETJSON_STATUS_MAX_VALID) {
            n += snprintf(buf + n, rem(sz, n), ",\"status\":%d", (int)loc->status);
        } else {
            n += snprintf(buf + n, rem(sz, n), ",\"status\":null");
        }
        n += put_float_or_null(buf + n, rem(sz, n), "height", loc->height,
                               loc->height > DETJSON_ALT_INVALID_MAX_M);
        n += put_float_or_null(buf + n, rem(sz, n), "vspd", loc->speed_vert,
                               fabsf(loc->speed_vert) <= DETJSON_VSPEED_MAX_VALID);
        if (detection_json_ts_valid(det)) {
            n += snprintf(buf + n, rem(sz, n), ",\"ts\":%u", (unsigned)loc->timestamp);
        }
    }

    if (det->has_system) {
        n += snprintf(buf + n, rem(sz, n), ",\"op_lat\":%.7f,\"op_lon\":%.7f",
                      (double)det->system.operator_lat, (double)det->system.operator_lon);
    }

    n += snprintf(buf + n, rem(sz, n), "}");
    return n;
}
