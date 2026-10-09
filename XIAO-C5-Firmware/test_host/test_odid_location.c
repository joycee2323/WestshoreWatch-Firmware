/* Host-side test: Cellular X1 Location decode (main/odid_decoder.c) and the
 * upload JSON (main/detection_json.c) against the REAL opendroneid-core-c
 * encoder/decoder (third_party/opendroneid, Apache-2.0).
 *
 *   cd XIAO-C5-Firmware/test_host
 *   gcc -std=c11 -Wall -Wextra -I../main -Istubs -Ithird_party/opendroneid \
 *       test_odid_location.c ../main/odid_decoder.c ../main/detection_json.c \
 *       third_party/opendroneid/opendroneid.c -lm -o test_odid_location
 *   ./test_odid_location
 *
 * Covers every byte-1 x byte-2 combination (x6 speeds) and encoded sweeps over
 * status 0-4 x height type x heading 0-359/361 x 17 speeds: our decode equals
 * the reference field by field, the raw bytes are kept, and the upload JSON
 * carries the spec spd/hdg (null when unknown), loc_raw = the drone's bytes in
 * hex, and decoder "odid-spec-1". */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "opendroneid.h"
#include "odid_decoder.h"
#include "detection_json.h"

static long checks = 0, failures = 0;
#define CHECK(cond, ...) do { checks++; if (!(cond)) { failures++; if (failures <= 20) { \
    fprintf(stderr, "FAIL %s:%d: ", __FILE__, __LINE__); fprintf(stderr, __VA_ARGS__); fputc('\n', stderr); } } } while (0)

static void check_frame(const uint8_t msg[25], const char *ctx)
{
    ODID_Location_data ref;
    odid_initLocationData(&ref);
    decodeLocationMessage(&ref, (const ODID_Location_encoded *)msg);

    odid_detection_t d;
    memset(&d, 0, sizeof d);
    CHECK(odid_parse_message(msg, 25, &d) == ODID_MSG_LOCATION && d.has_location, "%s: parse", ctx);
    const odid_location_t *o = &d.location;

    CHECK((int)o->height_type == (int)ref.HeightType, "%s: height type", ctx);
    bool hdg_unknown = msg[2] >= 180;
    bool spd_unknown = ref.SpeedHorizontal >= 254.99f && ref.SpeedHorizontal <= 255.01f;
    CHECK(o->heading_valid == !hdg_unknown, "%s: heading_valid", ctx);
    if (!hdg_unknown) CHECK(fabsf((float)o->heading - ref.Direction) < 0.01f, "%s: heading %u vs %.1f", ctx, o->heading, ref.Direction);
    CHECK(o->speed_valid == !spd_unknown, "%s: speed_valid", ctx);
    if (!spd_unknown) CHECK(fabsf(o->speed_horiz - ref.SpeedHorizontal) < 1e-3f, "%s: speed %.3f vs %.3f", ctx, o->speed_horiz, ref.SpeedHorizontal);
    CHECK(fabsf(o->speed_vert - ref.SpeedVertical) < 1e-3f, "%s: vspeed", ctx);
    CHECK(fabsf(o->height - ref.Height) < 1e-3f && fabsf(o->alt_geo - ref.AltitudeGeo) < 1e-3f, "%s: alt/height", ctx);
    CHECK(o->lat == ref.Latitude && o->lon == ref.Longitude, "%s: lat/lon", ctx);
    CHECK(o->raw_valid && memcmp(o->raw, msg, 25) == 0, "%s: raw bytes", ctx);

    /* Upload JSON. */
    d.has_basic_id = true;
    strcpy(d.basic_id.uas_id, "1668BR40FA0098ER");
    char json[512], want[96], hex[51];
    int n = detection_json_format(&d, json, sizeof json);
    CHECK(n > 0 && (size_t)n < sizeof json, "%s: json length %d", ctx, n);
    if (spd_unknown) CHECK(strstr(json, "\"spd\":null") != NULL, "%s: spd null", ctx);
    else { snprintf(want, sizeof want, "\"spd\":%.2f,", (double)ref.SpeedHorizontal); CHECK(strstr(json, want) != NULL, "%s: %s in %s", ctx, want, json); }
    if (hdg_unknown) CHECK(strstr(json, "\"hdg\":null") != NULL, "%s: hdg null", ctx);
    else { snprintf(want, sizeof want, "\"hdg\":%u,", (unsigned)lroundf(ref.Direction)); CHECK(strstr(json, want) != NULL, "%s: %s in %s", ctx, want, json); }
    for (int i = 0; i < 25; i++) snprintf(&hex[2 * i], 3, "%02x", msg[i]);
    snprintf(want, sizeof want, "\"loc_raw\":\"%s\",\"decoder\":\"odid-spec-1\"", hex);
    CHECK(strstr(json, want) != NULL, "%s: loc_raw/decoder missing in %s", ctx, json);
}

static void encode(uint8_t out[25], int status, int ht, float dir, float speed)
{
    ODID_Location_data d;
    odid_initLocationData(&d);
    d.Status = (ODID_status_t)status;
    d.HeightType = (ODID_Height_reference_t)ht;
    d.Direction = dir;
    d.SpeedHorizontal = speed;
    d.SpeedVertical = 1.5f;
    d.Latitude = 41.4611922;
    d.Longitude = -81.9237012;
    d.AltitudeBaro = 177.5f;
    d.AltitudeGeo = 179.0f;
    d.Height = 12.5f;
    d.TimeStamp = 360.0f;
    ODID_Location_encoded enc;
    CHECK(encodeLocationMessage(&enc, &d) == ODID_SUCCESS, "encode");
    memcpy(out, &enc, 25);
}

int main(void)
{
    uint8_t msg[25];
    char ctx[96];
    const float speeds[] = { 0.0f, 0.25f, 0.5f, 1.0f, 2.5f, 5.0f, 10.0f, 14.75f, 21.5f, 30.0f,
                             44.25f, 63.75f, 64.5f, 100.0f, 200.0f, 254.25f, 255.0f };
    for (int st = 0; st <= 4; st++)
        for (int ht = 0; ht <= 1; ht++)
            for (int dir = 0; dir <= 361; dir++) {
                if (dir == 360) continue;
                for (size_t s = 0; s < sizeof speeds / sizeof speeds[0]; s++) {
                    encode(msg, st, ht, (float)dir, speeds[s]);
                    snprintf(ctx, sizeof ctx, "enc st=%d ht=%d dir=%d spd=%.2f", st, ht, dir, speeds[s]);
                    check_frame(msg, ctx);
                }
            }
    const uint8_t raws[] = { 0, 1, 2, 100, 254, 255 };
    encode(msg, 2, 0, 0.0f, 0.0f);
    for (int b1 = 0; b1 <= 255; b1++)
        for (int b2 = 0; b2 <= 255; b2++)
            for (size_t r = 0; r < sizeof raws; r++) {
                msg[1] = (uint8_t)b1; msg[2] = (uint8_t)b2; msg[3] = raws[r];
                snprintf(ctx, sizeof ctx, "bits b1=0x%02x b2=%d b3=%d", b1, b2, raws[r]);
                check_frame(msg, ctx);
            }
    /* The headline case: heading 271, 10 m/s (legacy decoder: 45 deg, 30 m/s). */
    encode(msg, 2, 0, 271.0f, 10.0f);
    {
        odid_detection_t d; memset(&d, 0, sizeof d);
        odid_parse_message(msg, 25, &d);
        char json[512];
        detection_json_format(&d, json, sizeof json);
        CHECK(strstr(json, "\"spd\":10.00,\"hdg\":271,") != NULL, "271/10: %s", json);
    }

    if (failures) {
        fprintf(stderr, "%ld of %ld checks failed\n", failures, checks);
        return 1;
    }
    printf("odid_location (cellular): all %ld checks passed\n", checks);
    return 0;
}
