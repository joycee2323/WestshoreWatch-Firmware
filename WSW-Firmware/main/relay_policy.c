#include "relay_policy.h"

#include <string.h>

/* elapsed(a→now) with unsigned wrap. */
static rp_tick_t since(rp_tick_t then, rp_tick_t now)
{
    return (rp_tick_t)(now - then);
}

void rp_note_frame(rp_slot_timing_t *t, rp_tick_t now)
{
    t->has_any_frame  = true;
    t->last_any_frame = now;
}

void rp_note_airborne_state(rp_slot_timing_t *t, bool airborne, bool airborne_ever, rp_tick_t now)
{
    if (airborne) {
        t->has_airborne        = true;
        t->last_valid_airborne = now;
        t->has_grounded        = false;
    } else if (airborne_ever && !t->has_grounded) {
        t->has_grounded                = true;
        t->first_grounded_after_flight = now;
    }
}

void rp_note_location_ts(rp_slot_timing_t *t, uint32_t ts, rp_tick_t now)
{
    if (ts > RP_ODID_TS_MAX) {
        t->has_loc_ts = false;
        return;
    }
    if (!t->has_loc_ts || t->loc_ts != ts) {
        t->has_loc_ts        = true;
        t->loc_ts            = ts;
        t->loc_ts_changed_at = now;
    }
    t->loc_ts_last_seen_at = now;
}

rp_evict_t rp_check_evict(const rp_slot_timing_t *t, rp_tick_t now, const rp_limits_t *lim)
{
    if (t->has_grounded && since(t->first_grounded_after_flight, now) > lim->land_ticks)
        return RP_EVICT_LANDED;
    if (t->has_airborne && since(t->last_valid_airborne, now) > lim->silent_ticks)
        return RP_EVICT_SILENT;
    if (t->has_any_frame && since(t->last_any_frame, now) > lim->silent_ticks)
        return RP_EVICT_NO_FRAMES;
    /* Frozen = the same valid ts is STILL arriving frozen_ticks after it first
     * appeared: measured between first and latest sighting of that ts, so a
     * stretch of frames without Location (or with an unknown ts) can't make
     * the last good ts look frozen. */
    if (t->has_loc_ts &&
        since(t->loc_ts_changed_at, t->loc_ts_last_seen_at) > lim->frozen_ticks)
        return RP_EVICT_FROZEN;
    return RP_KEEP;
}

const char *rp_evict_name(rp_evict_t r)
{
    switch (r) {
    case RP_EVICT_LANDED:    return "landed";
    case RP_EVICT_SILENT:    return "silent";
    case RP_EVICT_NO_FRAMES: return "no frames";
    case RP_EVICT_FROZEN:    return "frozen ts";
    default:                 return "keep";
    }
}

/* ── Reject list ─────────────────────────────────────────────────────────── */

void rp_reject_init(rp_reject_list_t *l, rp_tick_t ttl_ticks)
{
    memset(l, 0, sizeof(*l));
    l->ttl_ticks = ttl_ticks;
}

static bool entry_live(const rp_reject_list_t *l, const rp_reject_entry_t *e, rp_tick_t now)
{
    return e->used && since(e->added_at, now) <= l->ttl_ticks;
}

void rp_reject_add(rp_reject_list_t *l, const char *uas_id, uint32_t ts, rp_tick_t now)
{
    if (!uas_id || !uas_id[0] || ts > RP_ODID_TS_MAX) return;
    rp_reject_entry_t *slot = NULL;
    for (int i = 0; i < RP_REJECT_SLOTS; i++) {
        rp_reject_entry_t *e = &l->e[i];
        if (e->used && e->ts == ts && strncmp(e->uas_id, uas_id, RP_UAS_ID_MAX) == 0) {
            slot = e;   /* refresh */
            break;
        }
    }
    if (!slot) {
        for (int i = 0; i < RP_REJECT_SLOTS; i++) {
            if (!entry_live(l, &l->e[i], now)) { slot = &l->e[i]; break; }
        }
    }
    if (!slot) {
        /* Full of live entries: replace the oldest. */
        slot = &l->e[0];
        for (int i = 1; i < RP_REJECT_SLOTS; i++) {
            if (since(l->e[i].added_at, now) > since(slot->added_at, now)) slot = &l->e[i];
        }
    }
    slot->used = true;
    strncpy(slot->uas_id, uas_id, RP_UAS_ID_MAX);
    slot->uas_id[RP_UAS_ID_MAX] = '\0';
    slot->ts       = ts;
    slot->added_at = now;
}

bool rp_reject_contains(const rp_reject_list_t *l, const char *uas_id, uint32_t ts, rp_tick_t now)
{
    if (!uas_id || !uas_id[0] || ts > RP_ODID_TS_MAX) return false;
    for (int i = 0; i < RP_REJECT_SLOTS; i++) {
        const rp_reject_entry_t *e = &l->e[i];
        if (entry_live(l, e, now) && e->ts == ts &&
            strncmp(e->uas_id, uas_id, RP_UAS_ID_MAX) == 0)
            return true;
    }
    return false;
}

/* ── Pack advertiser watchdog ───────────────────────────────────────────── */

void rp_radio_init(rp_radio_health_t *h)
{
    memset(h, 0, sizeof(*h));
}

void rp_radio_note_write(rp_radio_health_t *h, bool ok, rp_tick_t now)
{
    if (ok) {
        h->failing = false;
    } else if (!h->failing) {
        h->failing       = true;
        h->failing_since = now;
    }
}

rp_radio_action_t rp_radio_check(rp_radio_health_t *h, bool configured, rp_tick_t now,
                                 const rp_radio_limits_t *lim)
{
    bool writes_stuck = h->failing && since(h->failing_since, now) > lim->stale_write_ticks;
    if (configured && !writes_stuck) {
        h->unhealthy = false;
        return RP_RADIO_OK;
    }
    if (!h->unhealthy) {
        h->unhealthy       = true;
        h->unhealthy_since = now;
    }
    if (since(h->unhealthy_since, now) > lim->restart_ticks)
        return RP_RADIO_RESTART;
    if (h->has_reconfig && since(h->last_reconfig_at, now) < lim->reconfig_retry_ticks)
        return RP_RADIO_OK;   /* recently tried; give it a moment */
    return RP_RADIO_RECONFIGURE;
}

void rp_radio_note_reconfigure(rp_radio_health_t *h, rp_tick_t now)
{
    h->has_reconfig     = true;
    h->last_reconfig_at = now;
}
