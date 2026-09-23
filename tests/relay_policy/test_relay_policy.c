/* Host unit tests for main/relay_policy.c (Unity).
 *
 *   make -C tests/relay_policy SRC=../../WSW-Firmware/main UNITY=/path/to/Unity/src
 *
 * CI (.github/workflows/host-tests.yml) runs this against every project copy
 * of relay_policy.c on the branch. Ticks are 1 ms (CONFIG_FREERTOS_HZ=1000). */

#include "unity.h"
#include "relay_policy.h"

#include <stdio.h>
#include <string.h>

#define S(x) ((rp_tick_t)((x) * 1000u))

static const rp_limits_t LIM = {
    .land_ticks   = S(5),
    .silent_ticks = S(15),
    .frozen_ticks = S(30),
};

static const rp_radio_limits_t RLIM = {
    .stale_write_ticks    = S(30),
    .restart_ticks        = S(60),
    .reconfig_retry_ticks = S(5),
};

static rp_slot_timing_t slot;

void setUp(void)    { memset(&slot, 0, sizeof(slot)); }
void tearDown(void) {}

/* A slot fed a frame every second with the given ts stepping. */
static void feed(rp_tick_t from, rp_tick_t to, int ts_step, uint32_t ts0, bool airborne)
{
    uint32_t ts = ts0;
    for (rp_tick_t t = from; t <= to; t += S(1)) {
        rp_note_frame(&slot, t);
        rp_note_airborne_state(&slot, airborne, true, t);
        rp_note_location_ts(&slot, ts, t);
        ts = (uint32_t)((int)ts + ts_step) % (RP_ODID_TS_MAX);
    }
}

/* ── eviction ───────────────────────────────────────────────────────────── */

static void test_live_drone_with_advancing_ts_is_kept_for_20_minutes(void)
{
    feed(S(0), S(1200), 10, 24000, true);
    TEST_ASSERT_EQUAL(RP_KEEP, rp_check_evict(&slot, S(1200), &LIM));
}

static void test_frozen_ts_with_frames_still_arriving_evicts_after_30s(void)
{
    feed(S(0), S(10), 10, 24000, true);          /* genuine */
    feed(S(11), S(41), 0, 24721, true);           /* stuck: same ts every frame */
    TEST_ASSERT_EQUAL(RP_KEEP, rp_check_evict(&slot, S(41), &LIM));   /* 30 s since change */
    feed(S(42), S(42), 0, 24721, true);
    TEST_ASSERT_EQUAL(RP_EVICT_FROZEN, rp_check_evict(&slot, S(42), &LIM));
}

static void test_no_frames_evicts_after_silent_timeout(void)
{
    feed(S(0), S(10), 10, 100, false);
    TEST_ASSERT_EQUAL(RP_KEEP, rp_check_evict(&slot, S(25), &LIM));
    TEST_ASSERT_EQUAL(RP_EVICT_NO_FRAMES, rp_check_evict(&slot, S(26), &LIM));
}

static void test_landed_evicts_after_land_timeout(void)
{
    feed(S(0), S(10), 10, 100, true);
    feed(S(11), S(13), 10, 200, false);            /* grounded from t=11 */
    TEST_ASSERT_EQUAL(RP_KEEP, rp_check_evict(&slot, S(16), &LIM));
    TEST_ASSERT_EQUAL(RP_EVICT_LANDED, rp_check_evict(&slot, S(16) + 1, &LIM));
}

static void test_airborne_silence_evicts(void)
{
    feed(S(0), S(5), 10, 100, true);
    /* Airborne frames stop at t=5. At t=21 both "silent" and "no frames"
     * apply; silent is reported first. */
    TEST_ASSERT_EQUAL(RP_EVICT_SILENT, rp_check_evict(&slot, S(21), &LIM));
}

static void test_frame_on_tick_zero_is_still_evictable(void)
{
    /* Old code used tick==0 as "unset": a frame landing on tick 0 made the
     * slot unevictable. */
    rp_note_frame(&slot, 0);
    TEST_ASSERT_EQUAL(RP_EVICT_NO_FRAMES, rp_check_evict(&slot, S(16), &LIM));
}

static void test_eviction_is_wrap_safe(void)
{
    rp_tick_t base = (rp_tick_t)(0xFFFFFFFFu - S(5));
    feed(base, base + S(4), 10, 100, true);         /* crosses no wrap yet */
    rp_tick_t after_wrap = base + S(20);            /* wrapped past 0 */
    TEST_ASSERT_TRUE(after_wrap < base);
    TEST_ASSERT_NOT_EQUAL(RP_KEEP, rp_check_evict(&slot, after_wrap, &LIM));
    TEST_ASSERT_EQUAL(RP_KEEP, rp_check_evict(&slot, base + S(10), &LIM));
}

static void test_hour_wrap_of_odid_ts_counts_as_advancing(void)
{
    feed(S(0), S(3), 3, 35990, true);               /* 35990 → 35999 */
    rp_note_frame(&slot, S(4));
    rp_note_location_ts(&slot, 2, S(4));            /* wrapped past the hour */
    TEST_ASSERT_EQUAL_UINT32(S(4), slot.loc_ts_changed_at);
    TEST_ASSERT_EQUAL(RP_KEEP, rp_check_evict(&slot, S(18), &LIM));
}

static void test_unknown_odid_ts_does_not_look_frozen(void)
{
    /* ODID 0xFFFF = timestamp unknown. A real drone without GPS time must
     * not be evicted every 30 s as "frozen". */
    for (rp_tick_t now = S(0); now <= S(120); now += S(1)) {
        rp_note_frame(&slot, now);
        rp_note_airborne_state(&slot, true, true, now);
        rp_note_location_ts(&slot, 0xFFFF, now);
    }
    TEST_ASSERT_EQUAL(RP_KEEP, rp_check_evict(&slot, S(120), &LIM));
}

/* ── reject list ───────────────────────────────────────────────────────── */

static void test_reject_list_matches_exact_pair_until_ttl(void)
{
    rp_reject_list_t l;
    rp_reject_init(&l, S(600));
    rp_reject_add(&l, "1581F8HHX258900A03N0", 24721, S(100));
    TEST_ASSERT_TRUE(rp_reject_contains(&l, "1581F8HHX258900A03N0", 24721, S(100)));
    TEST_ASSERT_TRUE(rp_reject_contains(&l, "1581F8HHX258900A03N0", 24721, S(700)));
    TEST_ASSERT_FALSE(rp_reject_contains(&l, "1581F8HHX258900A03N0", 24731, S(200)));
    TEST_ASSERT_FALSE(rp_reject_contains(&l, "OTHER", 24721, S(200)));
    TEST_ASSERT_FALSE(rp_reject_contains(&l, "1581F8HHX258900A03N0", 24721, S(701)));
}

static void test_reject_list_full_replaces_oldest(void)
{
    rp_reject_list_t l;
    rp_reject_init(&l, S(600));
    char id[8];
    for (int i = 0; i < RP_REJECT_SLOTS; i++) {
        snprintf(id, sizeof(id), "D%d", i);
        rp_reject_add(&l, id, (uint32_t)i, S(10 + i));
    }
    rp_reject_add(&l, "NEW", 99, S(50));
    TEST_ASSERT_TRUE(rp_reject_contains(&l, "NEW", 99, S(50)));
    TEST_ASSERT_FALSE(rp_reject_contains(&l, "D0", 0, S(50)));      /* oldest gone */
    TEST_ASSERT_TRUE(rp_reject_contains(&l, "D7", 7, S(50)));
}

static void test_reject_list_ignores_empty_id(void)
{
    rp_reject_list_t l;
    rp_reject_init(&l, S(600));
    rp_reject_add(&l, "", 1, S(1));
    TEST_ASSERT_FALSE(rp_reject_contains(&l, "", 1, S(1)));
}

/* ── radio watchdog ────────────────────────────────────────────────────── */

static void test_radio_healthy_while_writes_succeed(void)
{
    rp_radio_health_t h;
    rp_radio_init(&h);
    for (rp_tick_t now = S(0); now < S(300); now += S(1)) {
        rp_radio_note_write(&h, true, now);
        TEST_ASSERT_EQUAL(RP_RADIO_OK, rp_radio_check(&h, true, now, &RLIM));
    }
}

static void test_radio_no_write_attempts_is_healthy(void)
{
    /* Live slots without a valid location never write a Pack — that must
     * not look like a wedged radio (it would reconfigure, then reboot). */
    rp_radio_health_t h;
    rp_radio_init(&h);
    TEST_ASSERT_EQUAL(RP_RADIO_OK, rp_radio_check(&h, true, S(600), &RLIM));
}

static void test_radio_unconfigured_reconfigures_then_restarts_after_60s(void)
{
    rp_radio_health_t h;
    rp_radio_init(&h);
    /* Host reset: instance no longer configured, placeholder writes no-op. */
    TEST_ASSERT_EQUAL(RP_RADIO_RECONFIGURE, rp_radio_check(&h, false, S(10), &RLIM));
    rp_radio_note_reconfigure(&h, S(10));
    TEST_ASSERT_EQUAL(RP_RADIO_OK, rp_radio_check(&h, false, S(12), &RLIM));   /* retry spacing */
    TEST_ASSERT_EQUAL(RP_RADIO_RECONFIGURE, rp_radio_check(&h, false, S(16), &RLIM));
    rp_radio_note_reconfigure(&h, S(16));
    TEST_ASSERT_EQUAL(RP_RADIO_RECONFIGURE, rp_radio_check(&h, false, S(70), &RLIM));
    rp_radio_note_reconfigure(&h, S(70));
    TEST_ASSERT_EQUAL(RP_RADIO_RESTART, rp_radio_check(&h, false, S(71), &RLIM));
}

static void test_radio_recovers_when_reconfigure_works(void)
{
    rp_radio_health_t h;
    rp_radio_init(&h);
    TEST_ASSERT_EQUAL(RP_RADIO_RECONFIGURE, rp_radio_check(&h, false, S(10), &RLIM));
    rp_radio_note_reconfigure(&h, S(10));
    rp_radio_note_write(&h, true, S(11));
    TEST_ASSERT_EQUAL(RP_RADIO_OK, rp_radio_check(&h, true, S(11), &RLIM));
    /* Much later, still fine — the old unhealthy window must not trigger a restart. */
    rp_radio_note_write(&h, true, S(200));
    TEST_ASSERT_EQUAL(RP_RADIO_OK, rp_radio_check(&h, true, S(200), &RLIM));
}

static void test_radio_failed_writes_for_30s_reconfigure_then_restart(void)
{
    rp_radio_health_t h;
    rp_radio_init(&h);
    rp_radio_note_write(&h, true, S(1));
    for (rp_tick_t now = S(2); now <= S(32); now += S(1)) {
        rp_radio_note_write(&h, false, now);        /* streak starts at t=2 */
        TEST_ASSERT_EQUAL(RP_RADIO_OK, rp_radio_check(&h, true, now, &RLIM));
    }
    rp_radio_note_write(&h, false, S(33));
    TEST_ASSERT_EQUAL(RP_RADIO_RECONFIGURE, rp_radio_check(&h, true, S(33), &RLIM));
    rp_radio_note_reconfigure(&h, S(33));
    TEST_ASSERT_EQUAL(RP_RADIO_OK, rp_radio_check(&h, true, S(35), &RLIM));
    TEST_ASSERT_EQUAL(RP_RADIO_RESTART, rp_radio_check(&h, true, S(94), &RLIM));
}

static void test_radio_single_failure_then_success_is_fine(void)
{
    rp_radio_health_t h;
    rp_radio_init(&h);
    rp_radio_note_write(&h, false, S(1));
    rp_radio_note_write(&h, true, S(2));
    TEST_ASSERT_EQUAL(RP_RADIO_OK, rp_radio_check(&h, true, S(100), &RLIM));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_live_drone_with_advancing_ts_is_kept_for_20_minutes);
    RUN_TEST(test_frozen_ts_with_frames_still_arriving_evicts_after_30s);
    RUN_TEST(test_no_frames_evicts_after_silent_timeout);
    RUN_TEST(test_landed_evicts_after_land_timeout);
    RUN_TEST(test_airborne_silence_evicts);
    RUN_TEST(test_frame_on_tick_zero_is_still_evictable);
    RUN_TEST(test_eviction_is_wrap_safe);
    RUN_TEST(test_hour_wrap_of_odid_ts_counts_as_advancing);
    RUN_TEST(test_unknown_odid_ts_does_not_look_frozen);
    RUN_TEST(test_reject_list_matches_exact_pair_until_ttl);
    RUN_TEST(test_reject_list_full_replaces_oldest);
    RUN_TEST(test_reject_list_ignores_empty_id);
    RUN_TEST(test_radio_healthy_while_writes_succeed);
    RUN_TEST(test_radio_no_write_attempts_is_healthy);
    RUN_TEST(test_radio_unconfigured_reconfigures_then_restarts_after_60s);
    RUN_TEST(test_radio_recovers_when_reconfigure_works);
    RUN_TEST(test_radio_failed_writes_for_30s_reconfigure_then_restart);
    RUN_TEST(test_radio_single_failure_then_success_is_fine);
    return UNITY_END();
}
