/* Host-side test: X1/M1 Location decode (main/odid_decoder.c) and relay
 * re-encode (main/odid_encoder.c) against the REAL opendroneid-core-c encoder
 * and decoder (tests/third_party/opendroneid, Apache-2.0).
 *
 *   make -C tests/odid_location run SRC=../../WSW-Firmware/main
 *
 * Exits non-zero if any check fails. Covers:
 *   1. realistic Location frames from encodeLocationMessage(): status 0-4,
 *      both height types, every heading 0-359 plus 361 (unknown), speeds in
 *      both multiplier ranges plus 255 (unknown);
 *   2. every byte-1 x byte-2 combination (65 536) for several byte-3 speeds,
 *      field by field against decodeLocationMessage();
 *   3. relay: the forwarded message equals the drone's bytes, and the
 *      fallback encoder (no raw bytes) decodes to the same values as the
 *      original under the reference decoder;
 *   4. 3 000 random Message Packs (Basic ID + Location + System) built with
 *      encodeMessagePack(), parsed with odid_parse_pack(), relayed, and decoded
 *      again with the reference: lat/lon, geodetic + baro altitude, height,
 *      height type, status, direction and speed must round-trip exactly
 *      (to the encoding's own resolution, i.e. identical encoded values). */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "opendroneid.h"
#include "odid_decoder.h"
#include "odid_encoder.h"

static long checks = 0, failures = 0;
#define CHECK(cond, ...) do { checks++; if (!(cond)) { failures++; if (failures <= 25) { \
    fprintf(stderr, "FAIL %s:%d: ", __FILE__, __LINE__); fprintf(stderr, __VA_ARGS__); fputc('\n', stderr); } } } while (0)

static uint32_t rng_state = 0x5EED1234u;
static uint32_t rng(void) { rng_state = rng_state * 1664525u + 1013904223u; return rng_state; }
static double urand(double lo, double hi) { return lo + (hi - lo) * (rng() / 4294967296.0); }

static odid_location_t ours(const uint8_t msg[25])
{
    odid_detection_t d;
    memset(&d, 0, sizeof d);
    int t = odid_parse_message(msg, 25, &d);
    CHECK(t == ODID_MSG_LOCATION && d.has_location, "parse_message type %d", t);
    return d.location;
}

static void ref_decode(const uint8_t msg[25], ODID_Location_data *ref)
{
    odid_initLocationData(ref);
    decodeLocationMessage(ref, (const ODID_Location_encoded *)msg);
}

/* Our decode of `msg` must match the reference decode field by field. */
static void compare_decode(const uint8_t msg[25], const char *ctx)
{
    ODID_Location_data ref;
    ref_decode(msg, &ref);
    odid_location_t o = ours(msg);

    CHECK((int)o.status == (int)((msg[1] >> 4) & 0x0F), "%s: status %d", ctx, (int)o.status);
    CHECK((int)o.height_type == (int)ref.HeightType, "%s: height_type %d vs %d", ctx, o.height_type, (int)ref.HeightType);
    if (msg[2] >= 180) {
        CHECK(!o.heading_valid, "%s: byte2 %d must be unknown heading", ctx, msg[2]);
    } else {
        CHECK(o.heading_valid && fabsf((float)o.heading - ref.Direction) < 0.01f,
              "%s: heading %u vs ref %.1f", ctx, o.heading, ref.Direction);
    }
    if (ref.SpeedHorizontal >= 254.99f && ref.SpeedHorizontal <= 255.01f) {
        CHECK(!o.speed_valid, "%s: speed 255 must be unknown", ctx);
    } else {
        CHECK(o.speed_valid && fabsf(o.speed_horiz - ref.SpeedHorizontal) < 1e-3f,
              "%s: speed %.3f vs ref %.3f", ctx, o.speed_horiz, ref.SpeedHorizontal);
    }
    CHECK(fabsf(o.speed_vert - ref.SpeedVertical) < 1e-3f, "%s: vspeed", ctx);
    CHECK(fabsf(o.height - ref.Height) < 1e-3f, "%s: height", ctx);
    CHECK(fabsf(o.alt_geo - ref.AltitudeGeo) < 1e-3f, "%s: alt_geo", ctx);
    CHECK(fabsf(o.alt_baro - ref.AltitudeBaro) < 1e-3f, "%s: alt_baro", ctx);
    CHECK(o.lat == ref.Latitude, "%s: lat %.9f vs %.9f", ctx, o.lat, ref.Latitude);
    CHECK(o.lon == ref.Longitude, "%s: lon %.9f vs %.9f", ctx, o.lon, ref.Longitude);
    CHECK(o.raw_valid && memcmp(o.raw, msg, 25) == 0, "%s: raw bytes not retained", ctx);
}

/* Two Location messages decode to the same values under the reference
 * (TSAccuracy, which the firmware does not keep, excluded). */
static void same_ref_values(const uint8_t a[25], const uint8_t b[25], const char *ctx)
{
    ODID_Location_data x, y;
    ref_decode(a, &x);
    ref_decode(b, &y);
    CHECK(x.Status == y.Status, "%s: status %d vs %d", ctx, (int)x.Status, (int)y.Status);
    CHECK(x.HeightType == y.HeightType, "%s: height type", ctx);
    CHECK(x.Direction == y.Direction, "%s: direction %.1f vs %.1f", ctx, x.Direction, y.Direction);
    CHECK(x.SpeedHorizontal == y.SpeedHorizontal, "%s: speed %.2f vs %.2f", ctx, x.SpeedHorizontal, y.SpeedHorizontal);
    CHECK(x.SpeedVertical == y.SpeedVertical, "%s: vspeed %.2f vs %.2f", ctx, x.SpeedVertical, y.SpeedVertical);
    CHECK(x.Latitude == y.Latitude, "%s: lat %.9f vs %.9f", ctx, x.Latitude, y.Latitude);
    CHECK(x.Longitude == y.Longitude, "%s: lon %.9f vs %.9f", ctx, x.Longitude, y.Longitude);
    CHECK(x.AltitudeBaro == y.AltitudeBaro, "%s: alt_baro", ctx);
    CHECK(x.AltitudeGeo == y.AltitudeGeo, "%s: alt_geo %.1f vs %.1f", ctx, x.AltitudeGeo, y.AltitudeGeo);
    CHECK(x.Height == y.Height, "%s: height %.1f vs %.1f", ctx, x.Height, y.Height);
    CHECK(x.HorizAccuracy == y.HorizAccuracy && x.VertAccuracy == y.VertAccuracy &&
          x.BaroAccuracy == y.BaroAccuracy && x.SpeedAccuracy == y.SpeedAccuracy, "%s: accuracy", ctx);
    CHECK(x.TimeStamp == y.TimeStamp, "%s: ts %.1f vs %.1f", ctx, x.TimeStamp, y.TimeStamp);
}

/* Relay paths for one received message. */
static void check_relay(const uint8_t msg[25], const char *ctx)
{
    odid_location_t o = ours(msg);
    uint8_t out[25];

    odid_encode_location(&o, out);                       /* forward path */
    CHECK(memcmp(out, msg, 25) == 0, "%s: forwarded bytes differ from the drone's", ctx);

    o.raw_valid = false;                                 /* fallback encoder */
    odid_encode_location(&o, out);
    CHECK((out[0] >> 4) == ODID_MSG_LOCATION, "%s: fallback header", ctx);
    same_ref_values(msg, out, ctx);
}

static int encode(uint8_t out[25], int status, int height_type, float dir, float speed)
{
    ODID_Location_data d;
    odid_initLocationData(&d);
    d.Status = (ODID_status_t)status;
    d.HeightType = (ODID_Height_reference_t)height_type;
    d.Direction = dir;
    d.SpeedHorizontal = speed;
    d.SpeedVertical = 1.5f;
    d.Latitude = 41.4611922;
    d.Longitude = -81.9237012;
    d.AltitudeBaro = 177.5f;
    d.AltitudeGeo = 179.0f;
    d.Height = 12.5f;
    d.HorizAccuracy = ODID_HOR_ACC_3_METER;
    d.VertAccuracy = ODID_VER_ACC_10_METER;
    d.SpeedAccuracy = ODID_SPEED_ACC_1_METERS_PER_SECOND;
    d.TimeStamp = 360.0f;
    ODID_Location_encoded enc;
    int rc = encodeLocationMessage(&enc, &d);
    memcpy(out, &enc, 25);
    return rc;
}

int main(void)
{
    uint8_t msg[25];
    char ctx[112];

    /* 1. Realistic frames. */
    const float speeds[] = { 0.0f, 0.25f, 0.5f, 1.0f, 2.5f, 5.0f, 10.0f, 14.75f, 21.5f, 30.0f,
                             44.25f, 63.75f, 64.5f, 100.0f, 200.0f, 254.25f, 255.0f };
    for (int status = 0; status <= 4; status++)
        for (int ht = 0; ht <= 1; ht++)
            for (int dir = 0; dir <= 361; dir++) {
                if (dir == 360) continue;
                for (size_t s = 0; s < sizeof speeds / sizeof speeds[0]; s++) {
                    CHECK(encode(msg, status, ht, (float)dir, speeds[s]) == ODID_SUCCESS, "encode");
                    snprintf(ctx, sizeof ctx, "enc st=%d ht=%d dir=%d spd=%.2f", status, ht, dir, speeds[s]);
                    compare_decode(msg, ctx);
                    check_relay(msg, ctx);
                }
            }

    /* 2. Every byte-1 x byte-2 combination for several byte-3 speeds. */
    const uint8_t raws[] = { 0, 1, 2, 100, 254, 255 };
    encode(msg, 2, 0, 0.0f, 0.0f);
    for (int b1 = 0; b1 <= 255; b1++)
        for (int b2 = 0; b2 <= 255; b2++)
            for (size_t r = 0; r < sizeof raws; r++) {
                msg[1] = (uint8_t)b1; msg[2] = (uint8_t)b2; msg[3] = raws[r];
                snprintf(ctx, sizeof ctx, "bits b1=0x%02x b2=%d b3=%d", b1, b2, raws[r]);
                compare_decode(msg, ctx);
                odid_location_t o = ours(msg);
                uint8_t out[25];
                odid_encode_location(&o, out);
                CHECK(memcmp(out, msg, 25) == 0, "%s: forwarded bytes differ", ctx);
            }

    /* 3. Named cases. */
    encode(msg, 2, 0, 271.0f, 10.0f);
    { odid_location_t o = ours(msg); CHECK(o.heading == 271, "271 (legacy decoder: 45)"); CHECK(fabsf(o.speed_horiz - 10.0f) < 1e-3f, "10 m/s (legacy: 30)"); }
    encode(msg, 1, 0, 361.0f, 255.0f);
    { odid_location_t o = ours(msg); CHECK(!o.heading_valid && !o.speed_valid, "unknown heading+speed");
      o.raw_valid = false; uint8_t out[25]; odid_encode_location(&o, out);
      CHECK(out[2] == 181 && (out[1] & 0x02) && (out[1] & 0x01) && out[3] == 255, "fallback writes 361/255 unknowns"); }

    /* 4. 3 000 random Message Packs through parse_pack and the relay. */
    int packs_ok = 0;
    for (int i = 0; i < 3000; i++) {
        ODID_UAS_Data uas;
        odid_initUasData(&uas);
        uas.BasicIDValid[0] = 1;
        uas.BasicID[0].UAType = ODID_UATYPE_HELICOPTER_OR_MULTIROTOR;
        uas.BasicID[0].IDType = ODID_IDTYPE_SERIAL_NUMBER;
        snprintf(uas.BasicID[0].UASID, sizeof uas.BasicID[0].UASID, "RND%013u", (unsigned)i);
        ODID_Location_data *L = &uas.Location;
        uas.LocationValid = 1;
        L->Status = (ODID_status_t)(rng() % 5);
        L->HeightType = (ODID_Height_reference_t)(rng() % 2);
        L->Direction = (rng() % 20 == 0) ? 361.0f : (float)(rng() % 360);
        int sp = rng() % 20;
        L->SpeedHorizontal = sp == 0 ? 255.0f : sp < 4 ? (float)urand(63.76, 254.25) : (float)urand(0, 63.75);
        L->SpeedVertical = (float)urand(-62, 62);
        L->Latitude = urand(-90, 90);
        L->Longitude = urand(-180, 180);
        L->AltitudeBaro = (float)urand(-1000, 31767);
        L->AltitudeGeo = (float)urand(-1000, 31767);
        L->Height = (float)urand(-1000, 31767);
        L->HorizAccuracy = (ODID_Horizontal_accuracy_t)(rng() % 13);
        L->VertAccuracy = (ODID_Vertical_accuracy_t)(rng() % 7);
        L->BaroAccuracy = (ODID_Vertical_accuracy_t)(rng() % 7);
        L->SpeedAccuracy = (ODID_Speed_accuracy_t)(rng() % 5);
        L->TimeStamp = (float)((rng() % 36000) / 10.0);
        uas.SystemValid = 1;
        uas.System.OperatorLatitude = urand(-90, 90);
        uas.System.OperatorLongitude = urand(-180, 180);
        uas.System.AreaCount = 1;
        uas.System.OperatorAltitudeGeo = (float)urand(-1000, 3000);

        ODID_MessagePack_data pd;
        odid_initMessagePackData(&pd);
        ODID_BasicID_encoded b; ODID_Location_encoded l; ODID_System_encoded s;
        CHECK(encodeBasicIDMessage(&b, &uas.BasicID[0]) == ODID_SUCCESS, "pack %d basic", i);
        CHECK(encodeLocationMessage(&l, L) == ODID_SUCCESS, "pack %d loc", i);
        CHECK(encodeSystemMessage(&s, &uas.System) == ODID_SUCCESS, "pack %d sys", i);
        memcpy(&pd.Messages[0], &b, 25);
        memcpy(&pd.Messages[1], &l, 25);
        memcpy(&pd.Messages[2], &s, 25);
        pd.MsgPackSize = 3;
        ODID_MessagePack_encoded pe;
        CHECK(encodeMessagePack(&pe, &pd) == ODID_SUCCESS, "pack %d encode", i);

        odid_detection_t d;
        memset(&d, 0, sizeof d);
        int n = odid_parse_pack((const uint8_t *)&pe, (uint8_t)(3 + 3 * 25), &d);
        CHECK(n == 3 && d.has_basic_id && d.has_location && d.has_system, "pack %d parsed %d", i, n);
        CHECK(strcmp(d.basic_id.uas_id, uas.BasicID[0].UASID) == 0, "pack %d uas id", i);

        snprintf(ctx, sizeof ctx, "pack %d", i);
        const uint8_t *orig = (const uint8_t *)&l;
        compare_decode(orig, ctx);                       /* decode == reference */
        uint8_t out[25];
        odid_encode_location(&d.location, out);          /* relay forward */
        CHECK(memcmp(out, orig, 25) == 0, "%s: relay forward differs", ctx);
        odid_location_t fb = d.location; fb.raw_valid = false;
        odid_encode_location(&fb, out);                  /* relay fallback */
        same_ref_values(orig, out, ctx);

        /* Against the original input values, at the encoding's resolution. */
        ODID_Location_data back;
        ref_decode(out, &back);
        CHECK(fabs(back.Latitude - L->Latitude) <= 0.5e-7 + 1e-12, "%s: lat %.9f vs %.9f", ctx, back.Latitude, L->Latitude);
        CHECK(fabs(back.Longitude - L->Longitude) <= 0.5e-7 + 1e-12, "%s: lon", ctx);
        CHECK(fabsf(back.AltitudeGeo - L->AltitudeGeo) <= 0.25f + 1e-3f, "%s: alt_geo", ctx);
        CHECK(fabsf(back.AltitudeBaro - L->AltitudeBaro) <= 0.25f + 1e-3f, "%s: alt_baro", ctx);
        CHECK(fabsf(back.Height - L->Height) <= 0.25f + 1e-3f, "%s: height", ctx);
        CHECK(back.Status == L->Status && back.HeightType == L->HeightType, "%s: status/height type", ctx);
        CHECK(L->Direction == 361.0f ? back.Direction == 361.0f : fabsf(back.Direction - L->Direction) < 0.01f,
              "%s: direction %.1f vs %.1f", ctx, back.Direction, L->Direction);
        float tol = L->SpeedHorizontal > 63.75f ? 0.375f : 0.125f;
        CHECK(L->SpeedHorizontal == 255.0f ? back.SpeedHorizontal == 255.0f
                                           : fabsf(back.SpeedHorizontal - L->SpeedHorizontal) <= tol + 1e-3f,
              "%s: speed %.3f vs %.3f", ctx, back.SpeedHorizontal, L->SpeedHorizontal);
        packs_ok++;
    }

    if (failures) {
        fprintf(stderr, "%ld of %ld checks failed\n", failures, checks);
        return 1;
    }
    printf("odid_location (X1/M1): all %ld checks passed (%d random packs)\n", checks, packs_ok);
    return 0;
}
