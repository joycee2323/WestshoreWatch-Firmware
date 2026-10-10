#pragma once

/* One drone object of the Cellular X1 detection upload — pure (no ESP-IDF
 * dependencies) so it can be unit-tested on a host: see
 * test_host/test_detection_json.c. Used by cellular_uploader.c. */

#include <stddef.h>
#include "odid_decoder.h"

/* ODID (ASTM F3411 / opendroneid) invalid-or-unknown encodings. A field
 * carrying one is uploaded as JSON null, never as the sentinel number — the
 * same rules as the phone parsers (OdidParser.kt / odidParser.ts) and the
 * backend's grounded-aircraft checks (services/groundedState.js). */
#define DETJSON_STATUS_MAX_VALID    4          /* 4 = RID system failure (F3411-22a); 5..15 reserved */
#define DETJSON_ALT_INVALID_MAX_M   (-999.75f) /* raw 0 = -1000 m (alt, height) */
#define DETJSON_VSPEED_MAX_VALID    62.0f      /* 63 m/s = invalid           */
#define DETJSON_TS_MAX              36000u     /* ODID timestamp: 0..35999 ds into the hour; 0xFFFF = unknown */

/* Writes `{...}` for one detection into buf (size sz). Returns the number of
 * characters snprintf would have written (same contract as snprintf). */
int detection_json_format(const odid_detection_t *det, char *buf, size_t sz);

/* True when det carries a usable ODID Location timestamp (sent as "ts"). */
bool detection_json_ts_valid(const odid_detection_t *det);
