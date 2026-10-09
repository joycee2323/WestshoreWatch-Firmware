#include "odid_encoder.h"

#include <math.h>
#include <stdbool.h>
#include <string.h>

/* Spec field encodings mirror opendroneid-core-c (encodeDirection,
 * encodeSpeedHorizontal, encodeSpeedVertical, encodeLatLon, encodeAltitude);
 * tests/odid_location checks them against the reference. */

static int clampi(long v, long lo, long hi) { return (int)(v < lo ? lo : v > hi ? hi : v); }

static uint16_t enc_alt(float m)
{
    return (uint16_t)clampi(lround((m + 1000.0f) / 0.5f), 0, 65535);
}

static int32_t enc_latlon(double deg)
{
    long long v = llround(deg * 1e7);
    if (v < -1800000000LL) v = -1800000000LL;
    if (v >  1800000000LL) v =  1800000000LL;
    return (int32_t)v;
}

void odid_encode_location(const odid_location_t *loc, uint8_t buf[25])
{
    if (loc->raw_valid && ((loc->raw[0] >> 4) & 0x0F) == ODID_MSG_LOCATION) {
        memcpy(buf, loc->raw, 25);
        return;
    }

    memset(buf, 0, 25);
    buf[0] = (ODID_MSG_LOCATION << 4) | 0x02;

    uint8_t ew, dir;
    if (loc->heading_valid) {
        unsigned h = loc->heading % 360u;
        ew  = h >= 180;
        dir = (uint8_t)(ew ? h - 180 : h);
    } else {                       /* 361 "unknown" -> 181 with E/W set */
        ew  = 1;
        dir = 181;
    }

    uint8_t mult, spd;
    if (!loc->speed_valid) {       /* 255 m/s "unknown" */
        mult = 1;
        spd  = 255;
    } else if (loc->speed_horiz <= 255 * 0.25f) {
        float v = loc->speed_horiz < 0.0f ? 0.0f : loc->speed_horiz;
        mult = 0;
        spd  = (uint8_t)clampi(lround(v / 0.25f), 0, 255);
    } else {
        mult = 1;
        spd  = (uint8_t)clampi(lround((loc->speed_horiz - 255 * 0.25f) / 0.75f), 0, 254);
    }

    buf[1] = (uint8_t)(((loc->status & 0x0F) << 4) | ((loc->height_type & 1) << 2) | (ew << 1) | mult);
    buf[2] = dir;
    buf[3] = spd;
    buf[4] = (uint8_t)(int8_t)clampi(lround(loc->speed_vert / 0.5f), -128, 127);

    int32_t lat = enc_latlon(loc->lat), lon = enc_latlon(loc->lon);
    memcpy(&buf[5], &lat, 4);
    memcpy(&buf[9], &lon, 4);
    uint16_t ab = enc_alt(loc->alt_baro), ag = enc_alt(loc->alt_geo), ht = enc_alt(loc->height);
    memcpy(&buf[13], &ab, 2);
    memcpy(&buf[15], &ag, 2);
    memcpy(&buf[17], &ht, 2);
    buf[19] = (uint8_t)(((loc->horiz_acc & 0x0F) << 4) | (loc->vert_acc & 0x0F));
    buf[20] = (uint8_t)(((loc->baro_acc & 0x0F) << 4) | (loc->speed_acc & 0x0F));
    uint16_t ts = (uint16_t)loc->timestamp;
    memcpy(&buf[21], &ts, 2);
}

/* ── Pre-1.4 compatibility ───────────────────────────────────────────────
 * The two functions below are the 1.3-westshore (cddc1df) parse_location()
 * and encode_location() bodies, unchanged apart from operating on this local
 * struct. Shipped apps decode their output; do not "fix" them. */
typedef struct {
    uint8_t  status;
    float    lat, lon, alt_baro, alt_geo, height, speed_horiz, speed_vert;
    uint16_t heading;
    uint32_t timestamp;
    uint8_t  horiz_acc, vert_acc, baro_acc, speed_acc;
} legacy_loc_t;

static void legacy_decode(const uint8_t *buf, legacy_loc_t *loc)
{
    loc->status        = (uint8_t)((buf[1] >> 4) & 0x0F);
    uint8_t speed_mult = buf[2] & 0x01;
    uint8_t ew_dir_seg = buf[1] & 0x01;
    uint8_t speed_raw  = buf[3];
    int8_t  vspeed_raw = (int8_t)buf[4];
    loc->speed_horiz = speed_mult ? (speed_raw * 0.75f)
                                  : (speed_raw * 0.25f);
    uint8_t dir_raw  = buf[2] >> 1;
    loc->heading     = (uint16_t)(dir_raw + (ew_dir_seg ? 180 : 0)) % 360;
    loc->speed_vert  = vspeed_raw * 0.5f;

    int32_t lat_raw, lon_raw;
    memcpy(&lat_raw, &buf[5], 4);
    memcpy(&lon_raw, &buf[9], 4);
    loc->lat = lat_raw * 1e-7f;
    loc->lon = lon_raw * 1e-7f;

    uint16_t ab, ag, ht, ts;
    memcpy(&ab, &buf[13], 2);
    memcpy(&ag, &buf[15], 2);
    memcpy(&ht, &buf[17], 2);
    memcpy(&ts, &buf[21], 2);
    loc->alt_baro  = ab * 0.5f - 1000.0f;
    loc->alt_geo   = ag * 0.5f - 1000.0f;
    loc->height    = ht * 0.5f - 1000.0f;
    loc->timestamp = ts;

    loc->horiz_acc = (buf[19] >> 4) & 0x0F;
    loc->vert_acc  =  buf[19]       & 0x0F;
    loc->baro_acc  = (buf[20] >> 4) & 0x0F;
    loc->speed_acc =  buf[20]       & 0x0F;
}

static void legacy_encode(const legacy_loc_t *loc, uint8_t *buf)
{
    memset(buf, 0, 25);
    buf[0] = (ODID_MSG_LOCATION << 4) | 0x02;

    uint8_t ew_seg = (loc->heading >= 180) ? 1 : 0;
    buf[1] = (uint8_t)((loc->status << 4) | ew_seg);

    uint8_t dir_mod = (uint8_t)(loc->heading % 180);
    bool use_mult = (loc->speed_horiz > 63.75f);
    buf[2] = (uint8_t)((dir_mod << 1) | (use_mult ? 1 : 0));

    float spd = use_mult ? (loc->speed_horiz / 0.75f)
                         : (loc->speed_horiz / 0.25f);
    if (spd < 0.0f) spd = 0.0f;
    if (spd > 254.0f) spd = 254.0f;
    buf[3] = (uint8_t)spd;

    buf[4] = (uint8_t)((int8_t)(loc->speed_vert / 0.5f));
    int32_t lat_raw = (int32_t)(loc->lat * 1e7f);
    int32_t lon_raw = (int32_t)(loc->lon * 1e7f);
    memcpy(&buf[5],  &lat_raw, 4);
    memcpy(&buf[9],  &lon_raw, 4);
    uint16_t ab = (uint16_t)((loc->alt_baro + 1000.0f) / 0.5f);
    uint16_t ag = (uint16_t)((loc->alt_geo  + 1000.0f) / 0.5f);
    uint16_t ht = (uint16_t)((loc->height   + 1000.0f) / 0.5f);
    memcpy(&buf[13], &ab, 2);
    memcpy(&buf[15], &ag, 2);
    memcpy(&buf[17], &ht, 2);
    buf[19] = (loc->horiz_acc << 4) | loc->vert_acc;
    buf[20] = (loc->baro_acc  << 4) | loc->speed_acc;
    uint16_t ts = (uint16_t)loc->timestamp;
    memcpy(&buf[21], &ts, 2);
}

void odid_encode_location_legacy(const odid_location_t *loc, uint8_t buf[25])
{
    uint8_t spec[25];
    odid_encode_location(loc, spec);     /* the drone's bytes (or a spec encode) */
    legacy_loc_t old;
    legacy_decode(spec, &old);           /* what a 1.3 relay decoded ...       */
    legacy_encode(&old, buf);            /* ... and re-broadcast               */
}

void odid_encode_system(const odid_system_t *sys, uint8_t buf[25])
{
    memset(buf, 0, 25);
    buf[0] = (ODID_MSG_SYSTEM << 4) | 0x02;
    buf[1] = (uint8_t)(((sys->classification_type & 0x07) << 2) | (sys->operator_location_type & 0x03));
    /* Same expression as the pre-1.4 relay, so bytes 2-9 are unchanged. */
    int32_t lat_raw = (int32_t)(sys->operator_lat * 1e7f);
    int32_t lon_raw = (int32_t)(sys->operator_lon * 1e7f);
    memcpy(&buf[2], &lat_raw, 4);
    memcpy(&buf[6], &lon_raw, 4);
    uint16_t ac = (uint16_t)sys->area_count;
    memcpy(&buf[10], &ac, 2);
    buf[12] = (uint8_t)clampi((long)(sys->area_radius / 10), 0, 255);
    uint16_t ceil_raw = enc_alt(sys->area_ceiling), floor_raw = enc_alt(sys->area_floor);
    memcpy(&buf[13], &ceil_raw, 2);
    memcpy(&buf[15], &floor_raw, 2);
    buf[17] = (uint8_t)(((sys->category & 0x0F) << 4) | (sys->class_value & 0x0F));
    uint16_t op_alt = enc_alt(sys->operator_alt_geo);
    memcpy(&buf[18], &op_alt, 2);
    uint32_t ts = sys->timestamp;
    memcpy(&buf[20], &ts, 4);
}

void odid_build_relay_pack(const uint8_t basic_id[25], const odid_location_t *loc,
                           const uint8_t system[25], uint8_t out[WSD_PACK_PAYLOAD])
{
    memset(out, 0, WSD_PACK_PAYLOAD);
    out[0] = (ODID_MSG_PACK << 4) | 0x02;
    out[1] = (uint8_t)((WSD_RELAY_FORMAT << 5) | WSD_PACK_MSG_COUNT);
    memcpy(&out[2], basic_id, 25);
    odid_encode_location_legacy(loc, &out[2 + 25]);
    memcpy(&out[2 + 50], system, 25);
    uint8_t *spec = &out[2 + 75];
    odid_encode_location(loc, spec);
    spec[0] = (uint8_t)((WSD_MSG_SPEC_LOCATION << 4) | (spec[0] & 0x0F));
}
