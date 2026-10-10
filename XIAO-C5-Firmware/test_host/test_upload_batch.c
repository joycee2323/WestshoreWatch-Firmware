/* Host test for main/upload_batch.c — newest-frame-per-drone batching and the
 * per-drone upload JSON (ts / age_ms).
 *
 *   cd XIAO-C5-Firmware/test_host
 *   gcc -std=c11 -Wall -Wextra -I../main test_upload_batch.c ../main/upload_batch.c ../main/detection_json.c -lm -o test_upload_batch
 *   ./test_upload_batch
 */
#include <stdio.h>
#include <string.h>
#include "upload_batch.h"

static int fails, checks;
#define CHECK(c) do { checks++; if (!(c)) { fails++; printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); } } while (0)

static upload_frame_t frame(const char *id, float lat, uint32_t ts, uint32_t rx_ms)
{
    upload_frame_t f;
    memset(&f, 0, sizeof f);
    if (id) {
        f.det.has_basic_id = true;
        snprintf(f.det.basic_id.uas_id, sizeof f.det.basic_id.uas_id, "%s", id);
    }
    f.det.has_location = true;
    f.det.location.lat = lat;
    f.det.location.lon = -81.9f;
    f.det.location.alt_geo = 180.0f;
    f.det.location.timestamp = ts;
    f.rx_ms = rx_ms;
    f.live = true;
    return f;
}

static void test_newest_per_drone(void)
{
    /* FIFO: oldest first. A1 A2 B1 A3 B2 → A3, B2 (newest first: A3 is last). */
    upload_frame_t in[5] = {
        frame("A", 41.1f, 100, 1000), frame("A", 41.2f, 110, 2000), frame("B", 42.1f, 105, 2500),
        frame("A", 41.3f, 120, 3000), frame("B", 42.2f, 115, 3500),
    };
    upload_frame_t out[8];
    upload_batch_stats_t st = {0};
    int n = upload_batch_collapse(in, 5, out, 8, &st);
    CHECK(n == 2);
    CHECK(strcmp(out[0].det.basic_id.uas_id, "B") == 0 && out[0].det.location.timestamp == 115);
    CHECK(strcmp(out[1].det.basic_id.uas_id, "A") == 0 && out[1].det.location.timestamp == 120);
    CHECK(out[1].rx_ms == 3000);
    CHECK(st.collapsed == 3 && st.no_id == 0 && st.over_cap == 0);
}

static void test_location_merge(void)
{
    /* Newest A frame has no Location (Basic-ID-only Pack): keep it but take the
     * newest earlier Location AND that frame's rx_ms. */
    upload_frame_t in[3] = { frame("A", 41.1f, 100, 1000), frame("A", 41.2f, 110, 2000), frame("A", 0, 0, 3000) };
    in[2].det.has_location = false;
    in[0].det.has_system = true; in[0].det.system.operator_lat = 41.5;
    upload_frame_t out[8];
    upload_batch_stats_t st = {0};
    int n = upload_batch_collapse(in, 3, out, 8, &st);
    CHECK(n == 1);
    CHECK(out[0].det.has_location && out[0].det.location.timestamp == 110);
    CHECK(out[0].rx_ms == 2000);
    CHECK(out[0].det.has_system && out[0].det.system.operator_lat == 41.5);
    CHECK(st.collapsed == 2);
}

static void test_no_id_and_cap(void)
{
    upload_frame_t in[4] = { frame("A", 1, 1, 1), frame(NULL, 2, 2, 2), frame("B", 3, 3, 3), frame("C", 4, 4, 4) };
    upload_frame_t out[2];
    upload_batch_stats_t st = {0};
    int n = upload_batch_collapse(in, 4, out, 2, &st);
    CHECK(n == 2);                                      /* C and B: the most recently heard */
    CHECK(strcmp(out[0].det.basic_id.uas_id, "C") == 0);
    CHECK(strcmp(out[1].det.basic_id.uas_id, "B") == 0);
    CHECK(st.no_id == 1 && st.over_cap == 1);
    CHECK(upload_batch_collapse(in, 0, out, 2, NULL) == 0);
}

static void test_spool_fill(void)
{
    upload_frame_t out[3];
    upload_frame_t a = frame("A", 1, 10, 100);
    out[0] = a;
    int n = 1;
    upload_batch_stats_t st = {0};
    upload_frame_t sa = frame("A", 2, 5, 0), sb = frame("B", 3, 6, 0), sc = frame("C", 4, 7, 0);
    n = upload_batch_add_spool(out, n, 3, &sa.det, &st);   /* A already live → skipped */
    CHECK(n == 1 && st.spool_skipped == 1);
    n = upload_batch_add_spool(out, n, 3, &sb.det, &st);
    CHECK(n == 2 && !out[1].live && strcmp(out[1].det.basic_id.uas_id, "B") == 0);
    n = upload_batch_add_spool(out, n, 3, &sc.det, &st);
    CHECK(n == 3);
    n = upload_batch_add_spool(out, n, 3, &sc.det, &st);   /* full → unchanged, not counted */
    CHECK(n == 3 && st.spool_skipped == 1);
}

static void test_json(void)
{
    char buf[512];
    upload_frame_t f = frame("1581F5FKD229400TEST", 41.4611922f, 18345, 1000);
    f.det.location.speed_horiz = 6.5f; f.det.location.heading = 90;
    upload_batch_format_drone(&f, 1850, buf, sizeof buf);
    CHECK(strstr(buf, "\"id\":\"1581F5FKD229400TEST\"") != NULL);
    CHECK(strstr(buf, "\"lat\":41.46119") != NULL);
    CHECK(strstr(buf, "\"alt\":180.0") != NULL);
    CHECK(strstr(buf, "\"ts\":18345") != NULL);
    CHECK(strstr(buf, "\"age_ms\":850") != NULL);
    CHECK(buf[0] == '{' && buf[strlen(buf) - 1] == '}');

    /* Unknown ODID timestamp (0xFFFF) → no ts; spool frame → no age_ms. */
    f.det.location.timestamp = 0xFFFF; f.live = false;
    upload_batch_format_drone(&f, 1850, buf, sizeof buf);
    CHECK(strstr(buf, "\"ts\"") == NULL);
    CHECK(strstr(buf, "\"age_ms\"") == NULL);
    f.det.location.timestamp = 35999;
    CHECK(upload_batch_ts_valid(&f.det));
    f.det.location.timestamp = 36000;
    CHECK(!upload_batch_ts_valid(&f.det));
    f.det.has_location = false; f.det.location.timestamp = 100;
    CHECK(!upload_batch_ts_valid(&f.det));

    /* age_ms across the 32-bit uptime wrap. */
    f = frame("A", 1, 1, 0xFFFFFF00u);
    upload_batch_format_drone(&f, 0x00000100u, buf, sizeof buf);
    CHECK(strstr(buf, "\"age_ms\":512") != NULL);

    /* 1.2.2 fields and ts/age_ms go out together, in one well-formed object. */
    f = frame("1581F5FKD229400TEST", 41.4611922f, 18345, 1000);
    f.det.location.status = (op_status_t)2;
    f.det.location.height = 30.0f;
    f.det.location.speed_vert = -1.5f;
    f.det.location.speed_horiz = 6.5f;
    upload_batch_format_drone(&f, 1850, buf, sizeof buf);
    CHECK(strstr(buf, "\"spd\":6.50,\"hdg\":0") != NULL);          /* two-decimal spd */
    CHECK(strstr(buf, ",\"status\":2") != NULL);
    CHECK(strstr(buf, ",\"height\":30.0") != NULL);
    CHECK(strstr(buf, ",\"vspd\":-1.5") != NULL);
    CHECK(strstr(buf, ",\"ts\":18345") != NULL);
    CHECK(strstr(buf, ",\"age_ms\":850}") != NULL);                 /* last field, object closed */
    CHECK(buf[0] == '{' && strchr(buf, '}') == buf + strlen(buf) - 1); /* exactly one closing brace */

    /* Invalid values → null, alongside ts/age_ms. */
    f.det.location.status = (op_status_t)7;
    f.det.location.height = -1000.0f;
    f.det.location.alt_geo = -1000.0f;
    f.det.location.speed_vert = 63.0f;
    upload_batch_format_drone(&f, 1850, buf, sizeof buf);
    CHECK(strstr(buf, ",\"status\":null") != NULL);
    CHECK(strstr(buf, ",\"height\":null") != NULL);
    CHECK(strstr(buf, "\"alt\":null") != NULL);
    CHECK(strstr(buf, ",\"vspd\":null") != NULL);
    CHECK(strstr(buf, ",\"ts\":18345") != NULL);
    CHECK(strstr(buf, ",\"age_ms\":850}") != NULL);

    /* Status 4 (RID system failure) is valid. */
    f.det.location.status = (op_status_t)4;
    upload_batch_format_drone(&f, 1850, buf, sizeof buf);
    CHECK(strstr(buf, ",\"status\":4") != NULL);

    /* Spool frame: same fields, no age_ms, object still closed. */
    f.live = false;
    upload_batch_format_drone(&f, 1850, buf, sizeof buf);
    CHECK(strstr(buf, "age_ms") == NULL && buf[strlen(buf) - 1] == '}');

    /* Truncation never runs past the buffer. */
    char small[24];
    int w = upload_batch_format_drone(&f, 0, small, sizeof small);
    CHECK(w <= (int)sizeof small - 1 && strlen(small) < sizeof small);
}

int main(void)
{
    test_newest_per_drone();
    test_location_merge();
    test_no_id_and_cap();
    test_spool_fill();
    test_json();
    printf("%d checks, %d failed\n", checks, fails);
    return fails ? 1 : 0;
}
