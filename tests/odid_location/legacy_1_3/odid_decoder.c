#include "odid_decoder.h"
#include <string.h>
#include "esp_log.h"

static const char *TAG = "ODID";

static void parse_basic_id(const uint8_t *buf, odid_detection_t *out)
{
    out->basic_id.id_type = (id_type_t)((buf[1] >> 4) & 0x0F);
    out->basic_id.ua_type = (ua_type_t)(buf[1] & 0x0F);
    memcpy(out->basic_id.uas_id, &buf[2], 20);
    out->basic_id.uas_id[20] = '\0';
    for (int i = 19; i >= 0; i--) {
        if (out->basic_id.uas_id[i] == '\0' || out->basic_id.uas_id[i] == ' ')
            out->basic_id.uas_id[i] = '\0';
        else break;
    }
    out->has_basic_id = true;
}

static void parse_location(const uint8_t *buf, odid_detection_t *out)
{
    odid_location_t *loc = &out->location;
    loc->status        = (op_status_t)((buf[1] >> 4) & 0x0F);
    uint8_t speed_mult = buf[2] & 0x01;
    uint8_t ew_dir_seg = buf[1] & 0x01;
    uint8_t speed_raw  = buf[3];
    int8_t  vspeed_raw = (int8_t)buf[4];
    /* ASTM F3411-22a: mult=0 => v=raw*0.25 m/s, mult=1 => v=raw*0.75 m/s */
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
    out->has_location = true;
}

static void parse_self_id(const uint8_t *buf, odid_detection_t *out)
{
    memcpy(out->self_id.description, &buf[2], 23);
    out->self_id.description[23] = '\0';
    out->has_self_id = true;
}

static void parse_system(const uint8_t *buf, odid_detection_t *out)
{
    odid_system_t *sys = &out->system;
    int32_t lat_raw, lon_raw;
    memcpy(&lat_raw, &buf[2], 4);
    memcpy(&lon_raw, &buf[6], 4);
    sys->operator_lat = lat_raw * 1e-7;
    sys->operator_lon = lon_raw * 1e-7;

    uint16_t ac, ar, ceil_raw, floor_raw, op_alt_raw;
    memcpy(&ac,        &buf[10], 2);
    memcpy(&ar,        &buf[12], 2);
    memcpy(&ceil_raw,  &buf[14], 2);
    memcpy(&floor_raw, &buf[16], 2);
    memcpy(&op_alt_raw,&buf[19], 2);
    sys->area_count       = ac;
    sys->area_radius      = ar * 10;
    sys->area_ceiling     = ceil_raw  * 0.5f - 1000.0f;
    sys->area_floor       = floor_raw * 0.5f - 1000.0f;
    sys->category         = (buf[18] >> 4) & 0x0F;
    sys->class_value      =  buf[18]       & 0x0F;
    sys->operator_alt_geo = op_alt_raw * 0.5f - 1000.0f;
    out->has_system = true;
}

static void parse_operator_id(const uint8_t *buf, odid_detection_t *out)
{
    out->operator_id.operator_id_type = buf[1];
    memcpy(out->operator_id.operator_id, &buf[2], 20);
    out->operator_id.operator_id[19] = '\0';
    out->has_operator_id = true;
}

int odid_parse_message(const uint8_t *buf, uint8_t len, odid_detection_t *out)
{
    if (!buf || len < 25) return -1;
    uint8_t msg_type = (buf[0] >> 4) & 0x0F;
    switch (msg_type) {
        case ODID_MSG_BASIC_ID:    parse_basic_id(buf, out);    return ODID_MSG_BASIC_ID;
        case ODID_MSG_LOCATION:    parse_location(buf, out);    return ODID_MSG_LOCATION;
        case ODID_MSG_SELF_ID:     parse_self_id(buf, out);     return ODID_MSG_SELF_ID;
        case ODID_MSG_SYSTEM:      parse_system(buf, out);      return ODID_MSG_SYSTEM;
        case ODID_MSG_OPERATOR_ID: parse_operator_id(buf, out); return ODID_MSG_OPERATOR_ID;
        case ODID_MSG_AUTH:        return ODID_MSG_AUTH;
        default:                   return -1;
    }
}

/* ─────────────────────────────────────────────────────────────────────────────
 * Try to find and parse an ODID Message Pack in buf.
 *
 * DJI Wi-Fi Beacon IE format (after OUI+subtype match):
 *   [counter 1B] [pack_header 1B = 0xF2] [msg_size 1B = 25] [msg_count 1B] [messages...]
 *
 * Standard ASTM format (no leading counter):
 *   [pack_header 1B = 0xF?] [msg_size 1B = 25] [msg_count 1B] [messages...]
 *
 * Strategy: look for the sequence (0xF?, 25) at offset 0 or 1.
 * ───────────────────────────────────────────────────────────────────────────── */
int odid_parse_pack(const uint8_t *buf, uint8_t len, odid_detection_t *out)
{
    if (!buf || len < 4) return 0;

    const uint8_t *p   = NULL;
    uint8_t        plen = 0;

    /* Try offset 0: standard format (no counter) */
    if (((buf[0] >> 4) & 0x0F) == ODID_MSG_PACK && buf[1] == 25) {
        p    = buf;
        plen = len;
    }
    /* Try offset 1: DJI format (leading counter byte) */
    else if (len >= 5 &&
             ((buf[1] >> 4) & 0x0F) == ODID_MSG_PACK && buf[2] == 25) {
        p    = buf + 1;
        plen = len - 1;
    }
    /* Try offset 0 with any msg_size (fallback) */
    else if (((buf[0] >> 4) & 0x0F) == ODID_MSG_PACK) {
        p    = buf;
        plen = len;
    }
    else {
        return 0;
    }

    uint8_t msg_size  = p[1];
    uint8_t msg_count = p[2];

    if (msg_size == 0 || msg_size > 25) {
        ESP_LOGW(TAG, "Pack msg_size=%d invalid, forcing 25", msg_size);
        msg_size = 25;
    }

    int parsed = 0;
    for (int i = 0; i < msg_count; i++) {
        uint16_t offset = 3 + (uint16_t)i * msg_size;
        if (offset + msg_size > plen) break;
        if (odid_parse_message(&p[offset], msg_size, out) >= 0)
            parsed++;
    }

    if (parsed > 0) {
        ESP_LOGI(TAG, "Pack parsed %d msgs — basic_id=%d loc=%d sys=%d",
                 parsed, out->has_basic_id, out->has_location, out->has_system);
    }

    return parsed;
}
