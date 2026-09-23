#pragma once
/* Relay-slot eviction, frozen-content cap, reject list and Pack-radio
 * watchdog — as pure logic with no FreeRTOS / NimBLE / ESP-IDF dependencies,
 * so it compiles and unit-tests on a host (see tests/relay_policy/). ble_relay.c owns
 * the slots and the radio; it asks this module what to do.
 *
 * Ticks are opaque uint32 counters (FreeRTOS ticks on target). All
 * comparisons use unsigned subtraction, so they are wrap-safe; "is this
 * timestamp set" is an explicit flag, never tick != 0 (a frame landing on
 * tick 0 used to make a slot unevictable).
 *
 * Incident 2026-09-22: a node kept radiating one cached Pack (ODID ts frozen)
 * for 30+ min after its slot evicted, because on this NimBLE build
 * ble_gap_ext_adv_stop doesn't stop the controller — only overwriting the
 * payload does — and after a BLE host reset the overwrite silently no-op'd.
 * The pieces here:
 *   - rp_check_evict: land / silent / no-frames (unchanged semantics) plus
 *     FROZEN: slot's ODID location timestamp hasn't advanced for frozen_ticks
 *     while frames keep arriving. A real drone's GPS-stamped ts always
 *     advances; a stuck one doesn't.
 *   - rp_reject_*: small TTL list of (uas_id, ts) evicted as frozen, so the
 *     same frozen frames can't immediately re-arm a slot.
 *   - rp_radio_*: Pack advertiser health. Reconfigure when the host lost the
 *     instance or payload writes have been failing for a while; restart the
 *     chip if that keeps failing (a power-cycle is the one recovery known to clear a
 *     wedged controller buffer). */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef uint32_t rp_tick_t;

#define RP_ODID_TS_MAX 36000u  /* ODID location timestamp range 0..36000 */

#define RP_UAS_ID_MAX 20   /* ODID Basic ID UAS_ID is at most 20 chars */

typedef struct {
    bool      has_any_frame;
    rp_tick_t last_any_frame;
    bool      has_airborne;
    rp_tick_t last_valid_airborne;
    bool      has_grounded;
    rp_tick_t first_grounded_after_flight;
    bool      has_loc_ts;
    uint32_t  loc_ts;              /* last ODID location timestamp seen */
    rp_tick_t loc_ts_changed_at;   /* tick when loc_ts last changed */
} rp_slot_timing_t;

typedef enum {
    RP_KEEP = 0,
    RP_EVICT_LANDED,
    RP_EVICT_SILENT,
    RP_EVICT_NO_FRAMES,
    RP_EVICT_FROZEN,
} rp_evict_t;

typedef struct {
    rp_tick_t land_ticks;    /* grounded-after-flight grace (land_timeout_s) */
    rp_tick_t silent_ticks;  /* no airborne / no frames (silent_timeout_s) */
    rp_tick_t frozen_ticks;  /* ODID ts not advancing */
} rp_limits_t;

/* Any frame merged into the slot. */
void rp_note_frame(rp_slot_timing_t *t, rp_tick_t now);
/* Slot's merged location status after the frame. Mirrors the old inline
 * airborne / first-grounded bookkeeping in relay_task. */
void rp_note_airborne_state(rp_slot_timing_t *t, bool airborne, bool airborne_ever, rp_tick_t now);
/* A frame carrying a valid location with ODID timestamp ts (tenths of a second
 * since the UTC hour, 0..36000). An out-of-range / unknown ts (ODID 0xFFFF)
 * turns frozen tracking off for the slot rather than looking frozen. */
void rp_note_location_ts(rp_slot_timing_t *t, uint32_t ts, rp_tick_t now);

rp_evict_t  rp_check_evict(const rp_slot_timing_t *t, rp_tick_t now, const rp_limits_t *lim);
const char *rp_evict_name(rp_evict_t r);

/* ── Reject list ─────────────────────────────────────────────────────────── */
#define RP_REJECT_SLOTS 8

typedef struct {
    bool      used;
    char      uas_id[RP_UAS_ID_MAX + 1];
    uint32_t  ts;
    rp_tick_t added_at;
} rp_reject_entry_t;

typedef struct {
    rp_reject_entry_t e[RP_REJECT_SLOTS];
    rp_tick_t         ttl_ticks;
} rp_reject_list_t;

void rp_reject_init(rp_reject_list_t *l, rp_tick_t ttl_ticks);
/* Adds (or refreshes) an entry; when full, replaces the oldest. */
void rp_reject_add(rp_reject_list_t *l, const char *uas_id, uint32_t ts, rp_tick_t now);
/* True while an unexpired entry matches exactly. */
bool rp_reject_contains(const rp_reject_list_t *l, const char *uas_id, uint32_t ts, rp_tick_t now);

/* ── Pack advertiser watchdog ───────────────────────────────────────────── */
typedef enum {
    RP_RADIO_OK = 0,
    RP_RADIO_RECONFIGURE,
    RP_RADIO_RESTART,
} rp_radio_action_t;

typedef struct {
    bool      failing;            /* a write attempt failed since the last success */
    rp_tick_t failing_since;      /* first failed attempt of the current streak */
    bool      unhealthy;
    rp_tick_t unhealthy_since;
    bool      has_reconfig;
    rp_tick_t last_reconfig_at;
} rp_radio_health_t;

typedef struct {
    rp_tick_t stale_write_ticks;    /* write attempts failing this long => reconfigure */
    rp_tick_t restart_ticks;        /* unhealthy this long => restart */
    rp_tick_t reconfig_retry_ticks; /* min spacing between reconfigure attempts */
} rp_radio_limits_t;

void rp_radio_init(rp_radio_health_t *h);
/* Result of one set_data attempt on the Pack handle. No attempts at all (live
 * slots without a valid location emit no Pack) is healthy, not stale. */
void rp_radio_note_write(rp_radio_health_t *h, bool ok, rp_tick_t now);
/* configured = the host still believes the Pack instance is configured. */
rp_radio_action_t rp_radio_check(rp_radio_health_t *h, bool configured, rp_tick_t now,
                                 const rp_radio_limits_t *lim);
/* Call after acting on RP_RADIO_RECONFIGURE. */
void rp_radio_note_reconfigure(rp_radio_health_t *h, rp_tick_t now);

#ifdef __cplusplus
}
#endif
