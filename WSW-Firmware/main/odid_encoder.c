#include "odid_encoder.h"

#include <math.h>
#include <string.h>

/* Field encodings mirror opendroneid-core-c (encodeDirection,
 * encodeSpeedHorizontal, encodeSpeedVertical, encodeLatLon, encodeAltitude);
 * test_host/test_relay_location.c checks them against the reference. */

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
