#pragma once

/**
 * Upload batch shaping for the Cellular X1 uploader — pure C, no ESP-IDF
 * dependency, so test_host/test_upload_batch.c can build it with gcc.
 *
 * Each upload cycle the uploader drains its whole RAM queue (FIFO, oldest
 * first) and collapses it to the NEWEST frame per drone (Basic ID uas_id).
 * The backend's coalescer keeps only the first frame per drone in a POST, so
 * sending older frames of the same drone only adds bytes and, worse, used to
 * put the OLDEST position on the live map. Collapsing here means the frame the
 * backend keeps is always the newest one the node has.
 */

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "odid_decoder.h"

/* One frame as the uploader handles it. `rx_ms` is the uptime (ms, 32-bit,
 * wraps after ~49.7 days; only differences are used) at which the distributor
 * took the frame off the scanner queue. Only meaningful when `live` is true:
 * frames replayed from the SPIFFS spool carry no receive time (the spool
 * stores a bare odid_detection_t and its format is fixed). */
typedef struct {
    odid_detection_t det;
    uint32_t         rx_ms;
    bool             live;
} upload_frame_t;

typedef struct {
    int collapsed;      /* older frames of a drone already in the batch (dropped) */
    int no_id;          /* frames without a Basic ID (backend would skip them)    */
    int over_cap;       /* frames of drones beyond the batch cap (dropped)        */
    int spool_skipped;  /* spool frames of a drone already in the batch (dropped) */
} upload_batch_stats_t;

/**
 * Collapse `n` frames (FIFO order: in[0] oldest, in[n-1] newest) to at most
 * `cap` frames, one per drone, written to `out` newest drone first.
 *
 * Per drone the newest frame wins. If that frame has no Location (e.g. a
 * Basic-ID-only Pack) the newest earlier Location of the same drone is merged
 * in, together with that frame's rx_ms, so the position's age stays truthful.
 * A missing System message is likewise filled from the newest earlier one.
 * Frames without a Basic ID are dropped (counted in no_id). When more than
 * `cap` drones are present the most recently heard `cap` are kept.
 *
 * Returns the number of frames written to `out`. `st` may be NULL.
 */
int upload_batch_collapse(const upload_frame_t *in, int n,
                          upload_frame_t *out, int cap,
                          upload_batch_stats_t *st);

/**
 * Offer one spool frame (already popped, newest spool frame first) to the
 * batch. Appended as a non-live frame unless its drone is already in the
 * batch (counted in spool_skipped) or the batch is full (returns count
 * unchanged, nothing counted — the caller should stop popping first).
 * Returns the new count.
 */
int upload_batch_add_spool(upload_frame_t *out, int count, int cap,
                           const odid_detection_t *spooled,
                           upload_batch_stats_t *st);

/** True when `det` carries a usable ODID Location timestamp (detection_json). */
bool upload_batch_ts_valid(const odid_detection_t *det);

/**
 * Serialize one drone object for POST /api/nodes/:device_id/detections: the
 * object detection_json_format() builds (id, lat, lon, alt, spd, hdg, status,
 * height, vspd, ts, op_lat, op_lon — invalid ODID values as null, ts omitted
 * when unknown) plus
 *   "age_ms" — ms between receive and `now_ms` (payload build time), live
 *              frames only; omitted for spool replays.
 * Returns the number of characters written, clamped to sz - 1 so the caller
 * can keep appending safely even when the buffer was too small.
 */
int upload_batch_format_drone(const upload_frame_t *f, uint32_t now_ms,
                              char *buf, size_t sz);
