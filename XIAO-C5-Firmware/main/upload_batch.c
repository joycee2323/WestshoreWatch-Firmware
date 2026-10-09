#include "upload_batch.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

static bool has_id(const odid_detection_t *d)
{
    return d->has_basic_id && d->basic_id.uas_id[0] != '\0';
}

static int find_drone(const upload_frame_t *out, int count, const char *uas_id)
{
    for (int i = 0; i < count; i++) {
        if (strcmp(out[i].det.basic_id.uas_id, uas_id) == 0) return i;
    }
    return -1;
}

int upload_batch_collapse(const upload_frame_t *in, int n,
                          upload_frame_t *out, int cap,
                          upload_batch_stats_t *st)
{
    int count = 0;
    /* Walk newest → oldest: the first frame seen for a drone is its newest. */
    for (int i = n - 1; i >= 0; i--) {
        const upload_frame_t *f = &in[i];
        if (!has_id(&f->det)) {
            if (st) st->no_id++;
            continue;
        }
        int slot = find_drone(out, count, f->det.basic_id.uas_id);
        if (slot < 0) {
            if (count < cap) {
                out[count++] = *f;
            } else if (st) {
                st->over_cap++;
            }
            continue;
        }
        /* Older frame of a drone already in the batch: fill gaps, then drop. */
        upload_frame_t *o = &out[slot];
        if (!o->det.has_location && f->det.has_location) {
            o->det.has_location = true;
            o->det.location     = f->det.location;
            o->rx_ms            = f->rx_ms;   /* the position is this old */
            o->live             = f->live;
        }
        if (!o->det.has_system && f->det.has_system) {
            o->det.has_system = true;
            o->det.system     = f->det.system;
        }
        if (st) st->collapsed++;
    }
    return count;
}

int upload_batch_add_spool(upload_frame_t *out, int count, int cap,
                           const odid_detection_t *spooled,
                           upload_batch_stats_t *st)
{
    if (count >= cap) return count;
    if (!has_id(spooled)) {
        if (st) st->no_id++;
        return count;
    }
    if (find_drone(out, count, spooled->basic_id.uas_id) >= 0) {
        if (st) st->spool_skipped++;
        return count;
    }
    out[count].det   = *spooled;
    out[count].rx_ms = 0;
    out[count].live  = false;
    return count + 1;
}

bool upload_batch_ts_valid(const odid_detection_t *det)
{
    return det->has_location && det->location.timestamp < UPLOAD_ODID_TS_MAX;
}

/* snprintf that never lets the running offset run past the buffer. */
static int put(char *buf, size_t sz, int n, const char *fmt, ...)
    __attribute__((format(printf, 4, 5)));
static int put(char *buf, size_t sz, int n, const char *fmt, ...)
{
    if (n < 0) n = 0;
    if ((size_t)n >= sz) return n;
    va_list ap;
    va_start(ap, fmt);
    int w = vsnprintf(buf + n, sz - (size_t)n, fmt, ap);
    va_end(ap);
    if (w < 0) return n;
    if ((size_t)(n + w) >= sz) return (int)sz - 1;   /* truncated: clamp */
    return n + w;
}

int upload_batch_format_drone(const upload_frame_t *f, uint32_t now_ms,
                              char *buf, size_t sz)
{
    const odid_detection_t *det = &f->det;
    int n = 0;

    /* Canonical schema read by routes/nodes.js (same as 1.2.1-westshore):
     * id is mandatory; alt = geodetic altitude; spd/hdg as decoded. */
    n = put(buf, sz, n, "{\"id\":\"%s\"", det->has_basic_id ? det->basic_id.uas_id : "");

    if (det->has_location) {
        n = put(buf, sz, n, ",\"lat\":%.7f,\"lon\":%.7f,\"alt\":%.1f,\"spd\":%.1f,\"hdg\":%u",
                (double)det->location.lat, (double)det->location.lon,
                (double)det->location.alt_geo, (double)det->location.speed_horiz,
                (unsigned)det->location.heading);
    }
    if (det->has_system) {
        n = put(buf, sz, n, ",\"op_lat\":%.7f,\"op_lon\":%.7f",
                det->system.operator_lat, det->system.operator_lon);
    }
    if (upload_batch_ts_valid(det)) {
        n = put(buf, sz, n, ",\"ts\":%u", (unsigned)det->location.timestamp);
    }
    if (f->live) {
        n = put(buf, sz, n, ",\"age_ms\":%lu", (unsigned long)(uint32_t)(now_ms - f->rx_ms));
    }
    n = put(buf, sz, n, "}");
    return n;
}
