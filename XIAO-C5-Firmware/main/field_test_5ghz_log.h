#pragma once
/* ═══════════════════════════════════════════════════════════════════════
 * TEMPORARY FIELD-TEST INSTRUMENTATION — added for one test flight to find
 * out which 5 GHz channel each real detection actually lands on, now that
 * the production sweep covers all 25 US channels (see
 * 5ghz-unii-channel-expansion-investigation.md). NOT permanent.
 *
 * To strip after the test: delete this file + field_test_5ghz_log.c, then
 * remove the single call/include added to each of wifi_scanner.c,
 * config_server.c, and main.c, and drop "field_test_5ghz_log.c" +
 * "spiffs" from main/CMakeLists.txt.
 * ═══════════════════════════════════════════════════════════════════════ */
#include "esp_err.h"
#include "esp_http_server.h"
#include "odid_decoder.h"
#include <stdint.h>

/* Mounts SPIFFS on the existing "storage" partition (already provisioned
 * in partitions.esp32c5.csv, previously unused — no partition table change,
 * no erase-flash needed) and opens the append-only log file. Call once at
 * boot, before wifi_scanner_start() so no detection can race the mount. */
esp_err_t field_test_5ghz_log_init(void);

/* Appends one line for a successful 5 GHz decode:
 *   "<ts_ms>,<channel>,<uas_id-or-dash>,<loc_valid 0|1>,<lat>/<lon-or-dash>"
 * loc_valid mirrors ble_relay.c's LOC_VALID macro (relay_task, ~line 684) —
 * same has_location + range + non-zero checks that gate the Pack advertiser
 * (handle 1) — so this settles, without guessing, whether a given decode
 * would have been eligible for Pack-handle advertising. lat/lon are the raw
 * decoded degrees (not the LOC_VALID-gated value) whenever has_location is
 * set, "-" otherwise.
 * Does a direct synchronous flash write — acceptable ONLY because 5 GHz ODID
 * decodes are rare (that rarity is the whole reason for this field test);
 * do not reuse this call pattern for anything higher-frequency. No-ops once
 * the file reaches FIELD_TEST_LOG_MAX_BYTES (field_test_5ghz_log.c) — no
 * rotation, it just stops appending, per spec for a single test flight. */
void field_test_5ghz_log_record(uint8_t channel, const odid_detection_t *det);

/* GET /debug/5ghz-log HTTP handler — serves the accumulated log as
 * text/plain, streamed straight from flash in chunks. */
esp_err_t field_test_5ghz_log_serve(httpd_req_t *req);
