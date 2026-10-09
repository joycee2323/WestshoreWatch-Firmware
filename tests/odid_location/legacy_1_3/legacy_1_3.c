/* Pre-1.4 relay reference for tests/odid_location (NOT firmware).
 * odid_decoder.c/.h in this directory are cddc1df (1.3-westshore) verbatim;
 * encode_location()/encode_system() below are copied verbatim from
 * cddc1df WSW-Firmware/main/ble_relay.c. Compiled as a separate translation
 * unit with its public symbols renamed (see Makefile). */
#include <stdbool.h>
#include <string.h>
#include "odid_decoder.h"
#include "legacy_1_3.h"

static void encode_location(const odid_detection_t *d, uint8_t *buf)
{
    memset(buf, 0, 25);
    const odid_location_t *loc = &d->location;
    buf[0] = (ODID_MSG_LOCATION << 4) | 0x02;

    /* buf[1]: status (4 bits) | ew_dir_segment (1 bit) — 0=0..179, 1=180..359 */
    uint8_t ew_seg = (loc->heading >= 180) ? 1 : 0;
    buf[1] = (uint8_t)((loc->status << 4) | ew_seg);

    /* buf[2]: direction_mod180 (7 bits) | speed_multiplier (1 bit)
     * speed_mult=0: speed = raw * 0.25 m/s  (0..63.75 m/s)
     * speed_mult=1: speed = raw * 0.75 + 63.75 m/s (63.75..254.25 m/s) */
    uint8_t dir_mod = (uint8_t)(loc->heading % 180);
    bool use_mult = (loc->speed_horiz > 63.75f);
    buf[2] = (uint8_t)((dir_mod << 1) | (use_mult ? 1 : 0));

    /* buf[3]: horizontal speed raw
     * mult=0: v = raw * 0.25 m/s (0..63.75)
     * mult=1: v = raw * 0.75 m/s (0..191.25) */
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

static void encode_system(const odid_detection_t *d, uint8_t *buf)
{
    memset(buf, 0, 25);
    buf[0] = (ODID_MSG_SYSTEM << 4) | 0x02;
    buf[1] = 0x00; /* operator location type = takeoff */
    int32_t lat_raw = (int32_t)(d->system.operator_lat * 1e7f);
    int32_t lon_raw = (int32_t)(d->system.operator_lon * 1e7f);
    memcpy(&buf[2], &lat_raw, 4);
    memcpy(&buf[6], &lon_raw, 4);
    uint16_t ac = (uint16_t)d->system.area_count;
    uint16_t ar = (uint16_t)(d->system.area_radius / 10);
    memcpy(&buf[10], &ac, 2);
    memcpy(&buf[12], &ar, 2);
    uint16_t ceil_raw  = (uint16_t)((d->system.area_ceiling  + 1000.0f) / 0.5f);
    uint16_t floor_raw = (uint16_t)((d->system.area_floor    + 1000.0f) / 0.5f);
    memcpy(&buf[14], &ceil_raw,  2);
    memcpy(&buf[16], &floor_raw, 2);
    buf[18] = (uint8_t)((d->system.category << 4) | d->system.class_value);
    uint16_t op_alt = (uint16_t)((d->system.operator_alt_geo + 1000.0f) / 0.5f);
    memcpy(&buf[19], &op_alt, 2);
}

/* What a 1.3 relay put on air for one drone: Pack (77 bytes) and the handle-0
 * Location message, from the received Basic ID, Location and System messages. */
void legacy13_relay(const uint8_t basic_msg[25], const uint8_t loc_msg[25], const uint8_t sys_msg[25],
                    uint8_t pack_out[77], uint8_t handle0_loc_out[25])
{
    odid_detection_t d;
    memset(&d, 0, sizeof d);
    odid_parse_message(basic_msg, 25, &d);
    odid_parse_message(loc_msg, 25, &d);
    odid_parse_message(sys_msg, 25, &d);
    memset(pack_out, 0, 77);
    pack_out[0] = (ODID_MSG_PACK << 4) | 0x02;
    pack_out[1] = 3;
    memcpy(&pack_out[2], basic_msg, 25);          /* encode_basic_id is unchanged in 1.4 */
    encode_location(&d, &pack_out[2 + 25]);
    encode_system  (&d, &pack_out[2 + 25 + 25]);
    encode_location(&d, handle0_loc_out);
}
