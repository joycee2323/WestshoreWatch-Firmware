/* Host-side test for main/detection_json.c (no ESP-IDF needed).
 *
 *   cd XIAO-C5-Firmware/test_host
 *   gcc -std=c11 -Wall -Wextra -I../main test_detection_json.c ../main/detection_json.c -lm -o test_detection_json
 *   ./test_detection_json
 *
 * Exits non-zero on the first failure. Cases mirror the phone parser tests
 * (test/odidParser.test.mjs in the app repo): the decoded docked-Skydio frame
 * plus every ODID invalid encoding. */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "detection_json.h"

static int failures = 0;

static void expect_contains(const char *json, const char *needle, const char *what)
{
    if (!strstr(json, needle)) {
        fprintf(stderr, "FAIL %s: expected %s in %s\n", what, needle, json);
        failures++;
    }
}

static void expect_absent(const char *json, const char *needle, const char *what)
{
    if (strstr(json, needle)) {
        fprintf(stderr, "FAIL %s: did not expect %s in %s\n", what, needle, json);
        failures++;
    }
}

/* Decoded docked Skydio X10E: status 1, speed 0, height -1.0, alt 179.0. */
static odid_detection_t skydio(void)
{
    odid_detection_t d;
    memset(&d, 0, sizeof d);
    d.has_basic_id = true;
    strcpy(d.basic_id.uas_id, "1668BR40FA0098ER");
    d.has_location = true;
    d.location.status = (op_status_t)1;
    d.location.lat = 41.4611922f;
    d.location.lon = -81.9237012f;
    d.location.alt_geo = 179.0f;
    d.location.height = -1.0f;
    d.location.speed_horiz = 0.0f;
    d.location.speed_vert = 0.0f;
    d.location.heading = 0;
    d.has_system = true;
    d.system.operator_lat = 41.4559348f;
    d.system.operator_lon = -81.9238019f;
    return d;
}

static void fmt(const odid_detection_t *d, char *buf, size_t sz)
{
    int n = detection_json_format(d, buf, sz);
    if (n <= 0 || (size_t)n >= sz) {
        fprintf(stderr, "FAIL format returned %d for buffer %zu\n", n, sz);
        failures++;
    }
}

int main(void)
{
    char buf[512];
    odid_detection_t d;

    /* 1. Docked Skydio: the Sentinel field names and units. */
    d = skydio();
    fmt(&d, buf, sizeof buf);
    expect_contains(buf, "{\"id\":\"1668BR40FA0098ER\"", "skydio id");
    expect_contains(buf, "\"alt\":179.0,", "skydio alt");
    expect_contains(buf, "\"spd\":0.00,\"hdg\":0", "skydio spd/hdg unchanged");
    expect_contains(buf, ",\"status\":1", "skydio status");
    expect_contains(buf, ",\"height\":-1.0", "skydio height -1.0 is valid");
    expect_contains(buf, ",\"vspd\":0.0", "skydio vspd");
    expect_contains(buf, ",\"op_lat\":", "skydio operator kept");

    /* 2. Invalid encodings → null. */
    d = skydio(); d.location.height = -1000.0f;
    fmt(&d, buf, sizeof buf);
    expect_contains(buf, ",\"height\":null", "height -1000 invalid");

    d = skydio(); d.location.alt_geo = -1000.0f;
    fmt(&d, buf, sizeof buf);
    expect_contains(buf, ",\"alt\":null", "alt -1000 invalid");

    d = skydio(); d.location.speed_vert = 63.0f;
    fmt(&d, buf, sizeof buf);
    expect_contains(buf, ",\"vspd\":null", "vspd 63 invalid");

    d = skydio(); d.location.speed_vert = -63.0f;
    fmt(&d, buf, sizeof buf);
    expect_contains(buf, ",\"vspd\":null", "vspd -63 invalid");

    d = skydio(); d.location.speed_vert = 62.0f;
    fmt(&d, buf, sizeof buf);
    expect_contains(buf, ",\"vspd\":62.0", "vspd 62 valid");

    d = skydio(); d.location.status = (op_status_t)5;
    fmt(&d, buf, sizeof buf);
    expect_contains(buf, ",\"status\":null", "status 5 reserved");

    d = skydio(); d.location.status = (op_status_t)15;
    fmt(&d, buf, sizeof buf);
    expect_contains(buf, ",\"status\":null", "status 15 reserved");

    d = skydio(); d.location.status = (op_status_t)2; d.location.height = 30.0f; d.location.speed_vert = -1.5f;
    fmt(&d, buf, sizeof buf);
    expect_contains(buf, ",\"status\":2", "status 2");
    expect_contains(buf, ",\"height\":30.0", "height 30");
    expect_contains(buf, ",\"vspd\":-1.5", "vspd -1.5");

    /* 3. No Location message: no location fields at all (unchanged behaviour). */
    d = skydio(); d.has_location = false;
    fmt(&d, buf, sizeof buf);
    expect_absent(buf, "\"status\"", "no location → no status");
    expect_absent(buf, "\"lat\"", "no location → no lat");

    /* 4. Small buffer: never writes past sz and stays NUL-terminated. */
    {
        char small[24];
        memset(small, 'X', sizeof small);
        d = skydio();
        int n = detection_json_format(&d, small, sizeof small);
        if (n < (int)sizeof small || small[sizeof small - 1] != '\0') {
            fprintf(stderr, "FAIL truncation: n=%d last=%d\n", n, small[sizeof small - 1]);
            failures++;
        }
    }

    if (failures) {
        fprintf(stderr, "%d failure(s)\n", failures);
        return 1;
    }
    printf("detection_json: all tests passed\n");
    return 0;
}
