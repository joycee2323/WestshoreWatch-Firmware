#include "ble_relay.h"
#include "config.h"
#include "nvs_config.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_random.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led.h"
#include <string.h>

static const char *TAG = "BLE_RELAY";

static QueueHandle_t s_queue        = NULL;
static bool          s_running      = false;
static bool          s_inited       = false;
static bool          s_task_created = false;
static uint8_t       s_counter      = 0;

/* Handle 0: ODID relay broadcast (extended adv, legacy PDU, non-connectable)
 * Handle 2: Westshore Watch detection advertiser (extended PDU, non-connectable,
 *           manufacturer-specific data, company ID 0x08FF)
 * Handle 3: Node identity advertiser (extended PDU, non-connectable,
 *           manufacturer-specific data, company ID 0x08FE — MAC + key prefix) */
#define ADV_HANDLE      0
#define DET_ADV_HANDLE  2
#define ID_ADV_HANDLE   3

/* Max JSON payload that fits in a single extended-adv AD structure.
 * AD header: 1 len + 1 type(0xFF) + 2 company id = 4 bytes.
 * Extended adv can carry more, but keep the detection payload small
 * enough that the Android app can parse it from a single adv report. */
#define DET_JSON_MAX    200

/* Identity advertiser payload — manufacturer-specific AD inside an extended
 * advertisement on handle 3:
 *   [0]     length = 1 (type) + 2 (company) + 6 (mac) + key_len
 *   [1]     0xFF
 *   [2]     company LSB (0xFE of 0x08FE)
 *   [3]     company MSB (0x08 of 0x08FE)
 *   [4..9]  node MAC (6 bytes)
 *   [10..]  api_key bytes (up to ID_API_KEY_MAX, unterminated)
 *
 * Extended PDUs support up to ~1650 bytes, so the full 36-char UUID api_key
 * fits comfortably. 64 gives headroom for any future key format. */
#define ID_API_KEY_MAX  64

static bool s_det_adv_configured = false;
static bool s_id_adv_configured  = false;

/* Forward decls */
static void relay_task(void *arg);

/* ─────────────────────────────────────────────────────────────────────────────
 * Encode helpers
 * ───────────────────────────────────────────────────────────────────────────── */
static void encode_basic_id(const odid_detection_t *d, uint8_t *buf)
{
    memset(buf, 0, 25);
    buf[0] = (ODID_MSG_BASIC_ID << 4) | 0x02;
    buf[1] = ((uint8_t)d->basic_id.id_type << 4) | (uint8_t)d->basic_id.ua_type;
    size_t id_len = strlen(d->basic_id.uas_id);
    if (id_len > 20) id_len = 20;
    memcpy(&buf[2], d->basic_id.uas_id, id_len);
}

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

static void encode_self_id_signal(const odid_detection_t *d, uint8_t *buf,
                                  int8_t rssi, odid_source_t src)
{
    (void)d; (void)rssi; (void)src;
    memset(buf, 0, 25);
    buf[0] = (ODID_MSG_SELF_ID << 4) | 0x02;
    buf[1] = 0x00; /* description type: General text */
    /* Override Self-ID with bridge name — tells DroneScout app this signal
     * is being relayed by a bridge, which triggers the green bridge icon. */
    const char *name = "DroneScout Bridge";
    memcpy(&buf[2], name, strlen(name));
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


/* Advertise one 25-byte ODID message using the extended advertising API.
 * Builds a 31-byte AD structure:
 *   [0]    len=30
 *   [1]    type=0x16 (Service Data 16-bit UUID)
 *   [2-3]  UUID=0xFFFA (LE)
 *   [4]    app code 0x0D
 *   [5]    rolling counter
 *   [6-30] 25-byte ODID message */
static void advertise_odid(const uint8_t *odid_msg_25)
{
    uint8_t adv_raw[31];
    adv_raw[0] = 30;
    adv_raw[1] = 0x16;
    adv_raw[2] = 0xFA;
    adv_raw[3] = 0xFF;
    adv_raw[4] = 0x0D;
    adv_raw[5] = s_counter++;
    memcpy(&adv_raw[6], odid_msg_25, 25);

    /* Build os_mbuf for the advertising data */
    struct os_mbuf *data = os_msys_get_pkthdr(sizeof(adv_raw), 0);
    if (!data) {
        ESP_LOGW(TAG, "os_msys_get_pkthdr failed");
        return;
    }
    if (os_mbuf_append(data, adv_raw, sizeof(adv_raw)) != 0) {
        ESP_LOGW(TAG, "os_mbuf_append failed");
        os_mbuf_free_chain(data);
        return;
    }

    /* ble_gap_ext_adv_set_data requires the advertiser to be stopped first.
     * Stop only if currently active, update data, then restart immediately
     * to minimise the gap in advertising. */
    bool was_active = ble_gap_ext_adv_active(ADV_HANDLE);
    if (was_active) ble_gap_ext_adv_stop(ADV_HANDLE);

    int rc = ble_gap_ext_adv_set_data(ADV_HANDLE, data);
    if (rc != 0) {
        ESP_LOGW(TAG, "ext_adv_set_data failed: %d", rc);
        return;
    }

    rc = ble_gap_ext_adv_start(ADV_HANDLE, 0, 0);
    if (rc != 0 && rc != BLE_HS_EALREADY) {
        ESP_LOGW(TAG, "ext_adv_start failed: %d", rc);
    }
}

/* ─────────────────────────────────────────────────────────────────────────────
 * Bridge identity beacon — exactly matches the real DroneScout Bridge hardware.
 *
 * Captured from real bridge via nRF Connect:
 *   AD type 0x16 (Service Data), UUID 0xFFFA
 *   app_code=0x0D (single msg), Basic ID msg, version 2
 *   id_type=1 (Serial Number), ua_type=15 (Other)
 *   UAS ID = "DroneScout Bridge" padded to 20 bytes
 *
 * The app sees ua_type=Other + UAS ID="DroneScout Bridge" and shows green icon.
 * Broadcast continuously (not a periodic ping) at fast BLE interval.
 * ───────────────────────────────────────────────────────────────────────────── */
static void encode_bridge_beacon(uint8_t *buf)
{
    memset(buf, 0, 25);
    buf[0] = (ODID_MSG_BASIC_ID << 4) | 0x02;  /* Basic ID, version 2 */
    buf[1] = (1 << 4) | 15;  /* id_type=1 (Serial Number), ua_type=15 (Other) */
    const char *name = "DroneScout Bridge";
    memcpy(&buf[2], name, strlen(name));  /* padded with zeros to 20 bytes */
}


static void relay_task(void *arg)
{
    odid_detection_t det;

    ESP_LOGI(TAG, "Relay task running");

    /* Persistent accumulated state — survives across relay cycles so that
     * basic_id from one frame is not lost when the next frame only has location.
     * Reset after 30s of no valid relay to avoid stale data persisting. */
    odid_detection_t acc;
    memset(&acc, 0, sizeof(acc));
    TickType_t last_valid_airborne = 0;
    TickType_t first_grounded_after_flight = 0;
    TickType_t last_any_frame = 0;  /* tracks any valid frame regardless of status */
    /* sys_saved preserves operator/pilot location across signal loss.
     * It is only populated once and never cleared. */
    odid_system_t   sys_saved     = {0};
    bool            sys_saved_ok  = false;

    /* ── Relay strategy ────────────────────────────────────────────────────────
     * Location is the only message that changes frequently — broadcast it on
     * every cycle (100ms dwell = 1 adv event at 100ms interval).
     * Basic ID and Self-ID are static; broadcast them every 5th cycle (~500ms).
     * This gives ~10 location updates/second to the app vs ~1/second previously.
     * ────────────────────────────────────────────────────────────────────────── */
    uint8_t basic_buf[25] = {0};
    uint8_t loc_buf[25]   = {0};
    uint8_t self_buf[25]  = {0};
    uint8_t sys_buf[25]   = {0};
    uint8_t cycle = 0;

    /* Bridge beacon — Basic ID with UAS ID = "DroneScout Bridge", ua_type=Other.
     * Broadcast every cycle (continuously) matching real bridge behavior. */
    uint8_t bridge_buf[25] = {0};
    encode_bridge_beacon(bridge_buf);

    #define LOC_VALID(d) ((d).has_location && \
                          (d).location.lat >= -90.0f && (d).location.lat <= 90.0f && \
                          (d).location.lon >= -180.0f && (d).location.lon <= 180.0f && \
                          ((d).location.lat != 0.0f || (d).location.lon != 0.0f))
    #define MERGE(src) do { \
        acc.rssi = (src).rssi; acc.source = (src).source; \
        if ((src).has_basic_id) { acc.basic_id  = (src).basic_id;  acc.has_basic_id  = true; } \
        if (LOC_VALID(src))     { acc.location  = (src).location;  acc.has_location  = true; } \
        if ((src).has_system)   { acc.system    = (src).system;    acc.has_system    = true; } \
        if ((src).has_self_id)  { acc.self_id   = (src).self_id;   acc.has_self_id   = true; } \
    } while(0)

    while (s_running) {

        /* Get one detection — short timeout so we stay responsive */
        bool got_frame = (xQueueReceive(s_queue, &det, pdMS_TO_TICKS(50)) == pdTRUE);

        if (got_frame) {
            /* Merge this frame — do NOT drain the whole queue so each position
             * gets its own broadcast cycle rather than being collapsed into one. */
            MERGE(det);
            last_any_frame = xTaskGetTickCount();  /* track last frame regardless of status */

            /* Persist operator location.
             * Rules:
             *   1. Require Basic ID first — DJI sends bogus coords before startup.
             *   2. Coords must be valid WGS84 and non-zero.
             *   3. Once a good fix is saved it never gets overwritten.
             *   4. Fallback: if RC sends 0,0 indefinitely (Mavic 3T / M4T / RC Pro),
             *      use the drone's first airborne GPS position as pilot proxy.
             *   5. CRITICAL: once saved, any subsequent bad coords from RC must be
             *      replaced with the saved location before broadcasting. */
            if (acc.has_system && acc.has_basic_id) {
                float op_lat = acc.system.operator_lat;
                float op_lon = acc.system.operator_lon;

                bool op_coords_valid =
                    (op_lat >= -90.0f && op_lat <= 90.0f &&
                     op_lon >= -180.0f && op_lon <= 180.0f &&
                     (op_lat != 0.0f || op_lon != 0.0f));

                if (!sys_saved_ok) {
                    if (op_coords_valid) {
                        sys_saved    = acc.system;
                        sys_saved_ok = true;
                        ESP_LOGI(TAG, "Operator location saved: lat=%.6f lon=%.6f",
                                 (double)op_lat, (double)op_lon);
                    } else {
                        /* Invalid coords, discard until we get a good one */
                        ESP_LOGD(TAG, "Skipping invalid operator coords: lat=%.6f lon=%.6f",
                                 (double)op_lat, (double)op_lon);
                        acc.has_system = false;
                    }
                } else {
                    /* Already have a good saved location — if RC is still sending
                     * bad coords (0,0), override with saved good location so the
                     * app always receives the correct pilot position. */
                    if (!op_coords_valid) {
                        acc.system = sys_saved;
                    }
                }
            } else if (acc.has_system && !acc.has_basic_id) {
                ESP_LOGD(TAG, "Deferring operator location — waiting for Basic ID");
                acc.has_system = false;
            }

            /* Fallback: drone airborne but RC never sent valid operator coords */
            if (!sys_saved_ok && acc.has_basic_id && LOC_VALID(acc) &&
                acc.has_location && acc.location.status == OP_STATUS_AIRBORNE) {
                sys_saved = acc.system;
                sys_saved.operator_lat = acc.location.lat;
                sys_saved.operator_lon = acc.location.lon;
                sys_saved_ok = true;
                ESP_LOGI(TAG, "Operator location fallback (drone pos): lat=%.6f lon=%.6f",
                         (double)acc.location.lat, (double)acc.location.lon);
            }

            if (sys_saved_ok && !acc.has_system) {
                acc.system    = sys_saved;
                acc.has_system = true;
            }
        }

        /* Silence/landing check — runs even when no frame received */
        bool landed_chk = (first_grounded_after_flight != 0 &&
                           (xTaskGetTickCount() - first_grounded_after_flight) > pdMS_TO_TICKS(5000));
        bool silent_chk = (last_valid_airborne != 0 &&
                           (xTaskGetTickCount() - last_valid_airborne) > pdMS_TO_TICKS(15000));
        if (landed_chk || silent_chk) {
            ESP_LOGI(TAG, "Drone %s — stopping relay", landed_chk ? "landed" : "silent");
            ble_gap_ext_adv_stop(ADV_HANDLE);
            led_set_detecting(false);
            memset(&acc, 0, sizeof(acc));
            if (sys_saved_ok) { acc.system = sys_saved; acc.has_system = true; }
            last_valid_airborne = 0;
            first_grounded_after_flight = 0;
            cycle = 0;
            continue;
        }

        if (!acc.has_basic_id) {
            /* No drone — continuously broadcast bridge beacon */
            advertise_odid(bridge_buf);
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        det = acc;

        bool loc_valid = LOC_VALID(det);
        bool airborne  = det.has_location && det.location.status == OP_STATUS_AIRBORNE;

        if (airborne) {
            last_valid_airborne = xTaskGetTickCount();
            first_grounded_after_flight = 0;  /* reset grounded timer while flying */
        } else if (last_valid_airborne != 0) {
            /* Drone was airborne, now grounded — start grounded timer */
            if (first_grounded_after_flight == 0)
                first_grounded_after_flight = xTaskGetTickCount();
        }

        /* Check landing timeout — stop relay if:
         *   (a) 15s silence with no airborne frames, OR
         *   (b) drone has been grounded for 5s after previously being airborne */
        bool landed = (first_grounded_after_flight != 0 &&
                       (xTaskGetTickCount() - first_grounded_after_flight) > pdMS_TO_TICKS(5000));
        bool silent = (last_valid_airborne != 0 &&
                       (xTaskGetTickCount() - last_valid_airborne) > pdMS_TO_TICKS(15000));
        /* Catch drones that never broadcast airborne status (e.g. Mavic 3T) —
         * stop relay if no frames at all for 15s after we had at least one frame. */
        bool any_silent = (last_any_frame != 0 &&
                           (xTaskGetTickCount() - last_any_frame) > pdMS_TO_TICKS(15000));

        if (landed || silent || any_silent) {
            ESP_LOGI(TAG, "Drone %s — stopping relay",
                     landed ? "landed" : (silent ? "silent" : "no frames"));
            ble_gap_ext_adv_stop(ADV_HANDLE);
            led_set_detecting(false);
            memset(&acc, 0, sizeof(acc));
            if (sys_saved_ok) { acc.system = sys_saved; acc.has_system = true; }
            last_valid_airborne = 0;
            first_grounded_after_flight = 0;
            last_any_frame = 0;
            cycle = 0;
            continue;
        }

        /* Bridge beacon broadcast is intentionally omitted during active relay.
         * The idle path below handles it when no drone is being tracked.
         * This gives maximum BLE airtime to drone position updates. */

        /* Encode fresh buffers */
        encode_basic_id(&det, basic_buf);
        if (loc_valid) encode_location(&det, loc_buf);
        if (det.has_system) encode_system(&det, sys_buf);
        encode_self_id_signal(&det, self_buf, det.rssi, det.source);

        /* Broadcast cycle:
         * - Every cycle:   location (freshest data) + system (operator location)
         * - Every 5th:     also basic_id + self_id
         * System message is broadcast every cycle so apps joining mid-flight
         * immediately receive the correct pilot location. */
        if (cycle == 0) {
            advertise_odid(basic_buf);
            vTaskDelay(pdMS_TO_TICKS(50));
            advertise_odid(self_buf);
            vTaskDelay(pdMS_TO_TICKS(50));
        }

        if (loc_valid) {
            advertise_odid(loc_buf);
            vTaskDelay(pdMS_TO_TICKS(50));
        }

        if (det.has_system) {
            advertise_odid(sys_buf);
            vTaskDelay(pdMS_TO_TICKS(50));
        } else {
            vTaskDelay(pdMS_TO_TICKS(50));
        }

        cycle = (cycle + 1) % 5;

        /* Log only on cycle 0 to avoid spamming */
        if (cycle == 0) {
            ESP_LOGI(TAG, "Relay: airborne=%d lat=%.6f lon=%.6f op_lat=%.6f op_lon=%.6f",
                     airborne,
                     loc_valid ? (double)det.location.lat : 0.0,
                     loc_valid ? (double)det.location.lon : 0.0,
                     det.has_system ? (double)det.system.operator_lat : 0.0,
                     det.has_system ? (double)det.system.operator_lon : 0.0);
        }
    }

    #undef MERGE
    #undef LOC_VALID

    ble_gap_ext_adv_stop(ADV_HANDLE);
    s_task_created = false;
    vTaskDelete(NULL);
}

/* Helper: spawn the relay task once. Safe to call from both on_sync and
 * ble_relay_start — guarded by s_task_created. */
static void spawn_relay_task_if_ready(void)
{
    if (!s_running || s_task_created) return;
    BaseType_t ret = xTaskCreate(relay_task, "ble_relay",
                                 4096, NULL,
                                 WSD_OUTPUT_TASK_PRIO + 1, NULL);
    if (ret == pdPASS) {
        s_task_created = true;
    } else {
        ESP_LOGE(TAG, "relay task create failed");
    }
}

/* ─────────────────────────────────────────────────────────────────────────────
 * Detection advertiser (handle 2) — configuration
 * ───────────────────────────────────────────────────────────────────────────── */
static int configure_detection_advertiser(void)
{
    /* Non-connectable, non-scannable extended PDU. Payload capacity is large
     * enough for our ~150-byte compact JSON plus the 4-byte AD header. */
    struct ble_gap_ext_adv_params params;
    memset(&params, 0, sizeof(params));
    params.legacy_pdu    = 0;
    params.connectable   = 0;
    params.scannable     = 0;
    params.own_addr_type = BLE_OWN_ADDR_PUBLIC;
    params.primary_phy   = BLE_HCI_LE_PHY_1M;
    params.secondary_phy = BLE_HCI_LE_PHY_1M;
    params.itvl_min      = BLE_GAP_ADV_ITVL_MS(100);
    params.itvl_max      = BLE_GAP_ADV_ITVL_MS(150);
    params.sid           = 2;
    params.tx_power      = 127;  /* 127 = host has no preference */

    int rc = ble_gap_ext_adv_configure(DET_ADV_HANDLE, &params,
                                       NULL, NULL, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "det ext_adv_configure (handle %d) failed: %d",
                 DET_ADV_HANDLE, rc);
        return rc;
    }

    s_det_adv_configured = true;
    ESP_LOGI(TAG, "Detection advertiser configured on handle %d", DET_ADV_HANDLE);
    return 0;
}

/* ─────────────────────────────────────────────────────────────────────────────
 * Identity advertiser (handle 3) — extended advertising
 *
 * Mirrors the detection advertiser setup (non-connectable, non-scannable
 * extended PDU) but at 500ms interval and with a static manufacturer-specific
 * payload: company 0x08FE + node MAC(6) + api_key prefix (up to 8 bytes).
 * ───────────────────────────────────────────────────────────────────────────── */
static int configure_id_advertiser(void)
{
    struct ble_gap_ext_adv_params params;
    memset(&params, 0, sizeof(params));
    params.legacy_pdu    = 0;
    params.connectable   = 0;
    params.scannable     = 0;
    params.own_addr_type = BLE_OWN_ADDR_PUBLIC;
    params.primary_phy   = BLE_HCI_LE_PHY_1M;
    params.secondary_phy = BLE_HCI_LE_PHY_1M;
    params.itvl_min      = BLE_GAP_ADV_ITVL_MS(500);
    params.itvl_max      = BLE_GAP_ADV_ITVL_MS(500);
    params.sid           = 3;
    params.tx_power      = 127;

    int rc = ble_gap_ext_adv_configure(ID_ADV_HANDLE, &params,
                                       NULL, NULL, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "id ext_adv_configure (handle %d) failed: %d",
                 ID_ADV_HANDLE, rc);
        return rc;
    }

    /* Use ESP_MAC_BT so the payload MAC matches the on-air public address
     * the controller advertises under (set via r_esp_ble_ll_set_public_addr
     * during BLE init). The WiFi STA MAC differs by 2 bytes, which caused
     * the Android scanner's observed device_id to mismatch the registered
     * backend identity. */
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_BT);

    size_t key_len = strlen(g_config.api_key);
    if (key_len > ID_API_KEY_MAX) key_len = ID_API_KEY_MAX;

    /* Manufacturer-specific AD structure:
     *   [0] length = 1 (type) + 2 (company) + 6 (mac) + key_len
     *   [1] 0xFF
     *   [2] company LSB (0xFE of 0x08FE)
     *   [3] company MSB (0x08 of 0x08FE)
     *   [4..9]  MAC
     *   [10..]  api_key prefix */
    uint8_t buf[1 + 1 + 2 + 6 + ID_API_KEY_MAX];
    size_t payload_len = 1 + 2 + 6 + key_len;  /* type + company + mac + key */
    buf[0] = (uint8_t)payload_len;
    buf[1] = 0xFF;
    buf[2] = 0xFE;
    buf[3] = 0x08;
    memcpy(&buf[4],  mac, 6);
    memcpy(&buf[10], g_config.api_key, key_len);
    size_t total = 1 + payload_len;

    struct os_mbuf *data = os_msys_get_pkthdr(total, 0);
    if (!data) {
        ESP_LOGW(TAG, "id msys_get_pkthdr failed");
        return -1;
    }
    if (os_mbuf_append(data, buf, total) != 0) {
        ESP_LOGW(TAG, "id mbuf_append failed");
        os_mbuf_free_chain(data);
        return -1;
    }

    rc = ble_gap_ext_adv_set_data(ID_ADV_HANDLE, data);
    if (rc != 0) {
        ESP_LOGE(TAG, "id ext_adv_set_data failed: %d", rc);
        return rc;
    }

    rc = ble_gap_ext_adv_start(ID_ADV_HANDLE, 0, 0);
    if (rc != 0 && rc != BLE_HS_EALREADY) {
        ESP_LOGE(TAG, "id ext_adv_start failed: %d", rc);
        return rc;
    }

    s_id_adv_configured = true;
    ESP_LOGI(TAG,
             "Identity advertiser started on handle %d mac=%02X:%02X:%02X:%02X:%02X:%02X key_len=%u",
             ID_ADV_HANDLE,
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
             (unsigned)key_len);
    return 0;
}

/* ─────────────────────────────────────────────────────────────────────────────
 * NimBLE host callbacks
 * ───────────────────────────────────────────────────────────────────────────── */
static void on_sync(void)
{
    ESP_LOGI(TAG, "BLE host synced — configuring extended advertisers");

    /* Handle 0 — non-connectable legacy PDU for ODID relay. */
    struct ble_gap_ext_adv_params params;
    memset(&params, 0, sizeof(params));
    params.legacy_pdu      = 1;
    params.connectable     = 0;
    params.scannable       = 0;
    params.own_addr_type   = BLE_OWN_ADDR_PUBLIC;
    params.primary_phy     = BLE_HCI_LE_PHY_1M;
    params.secondary_phy   = BLE_HCI_LE_PHY_1M;
    params.itvl_min        = BLE_GAP_ADV_ITVL_MS(100);
    params.itvl_max        = BLE_GAP_ADV_ITVL_MS(100);
    params.sid             = 0;

    int rc = ble_gap_ext_adv_configure(ADV_HANDLE, &params, NULL, NULL, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "ext_adv_configure failed: %d — relay will not work", rc);
    } else {
        ESP_LOGI(TAG, "Relay advertiser configured OK");
    }

    /* Legacy GAP advertiser — identity beacon (static MAC + API key). */
    configure_id_advertiser();

    /* Handle 2 — extended PDU for Westshore Watch detection advertising. */
    configure_detection_advertiser();

    /* If ble_relay_start() was called before the host synced, start the task
     * now. If relay mode is disabled, this is a no-op. */
    spawn_relay_task_if_ready();
}

static void on_reset(int reason)
{
    ESP_LOGW(TAG, "BLE host reset: %d", reason);
    s_det_adv_configured = false;
    s_id_adv_configured  = false;
}

static void nimble_host_task(void *arg)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

/* ─────────────────────────────────────────────────────────────────────────────
 * Public API
 * ───────────────────────────────────────────────────────────────────────────── */
esp_err_t ble_relay_init(void)
{
    if (s_inited) return ESP_OK;

    esp_err_t err = nimble_port_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nimble_port_init failed: %d", err);
        return err;
    }

    ble_hs_cfg.sync_cb  = on_sync;
    ble_hs_cfg.reset_cb = on_reset;

    nimble_port_freertos_init(nimble_host_task);

    s_inited = true;
    ESP_LOGI(TAG, "NimBLE host initialized");
    return ESP_OK;
}

esp_err_t ble_relay_start(QueueHandle_t detect_queue)
{
    if (s_running) return ESP_OK;
    if (!s_inited) {
        ESP_LOGE(TAG, "ble_relay_start called before ble_relay_init");
        return ESP_ERR_INVALID_STATE;
    }

    s_queue   = detect_queue;
    s_running = true;

    /* If the host has already synced, spawn the task immediately.
     * Otherwise on_sync will do it when sync fires. */
    if (ble_hs_synced()) {
        spawn_relay_task_if_ready();
    }

    ESP_LOGI(TAG, "BLE relay started");
    return ESP_OK;
}

void ble_detection_advertise(const char *json, size_t len)
{
    if (!s_det_adv_configured) return;
    if (!json || len == 0) return;
    if (len > DET_JSON_MAX) len = DET_JSON_MAX;

    /* Build manufacturer-specific AD structure:
     *   [0] length = 1 (type) + 2 (company id) + json_len
     *   [1] 0xFF  (AD type: Manufacturer Specific Data)
     *   [2] company id LSB (0xFF of 0x08FF)
     *   [3] company id MSB (0x08 of 0x08FF)
     *   [4..] json bytes */
    uint8_t buf[4 + DET_JSON_MAX];
    buf[0] = (uint8_t)(3 + len);
    buf[1] = 0xFF;
    buf[2] = 0xFF;   /* company 0x08FF, little-endian */
    buf[3] = 0x08;
    memcpy(&buf[4], json, len);
    size_t total = 4 + len;

    struct os_mbuf *data = os_msys_get_pkthdr(total, 0);
    if (!data) {
        ESP_LOGW(TAG, "det msys_get_pkthdr failed");
        return;
    }
    if (os_mbuf_append(data, buf, total) != 0) {
        ESP_LOGW(TAG, "det mbuf_append failed");
        os_mbuf_free_chain(data);
        return;
    }

    /* Stop, update payload, restart. ble_gap_ext_adv_set_data requires the
     * advertiser to be stopped if it is currently active. */
    if (ble_gap_ext_adv_active(DET_ADV_HANDLE)) {
        ble_gap_ext_adv_stop(DET_ADV_HANDLE);
    }

    int rc = ble_gap_ext_adv_set_data(DET_ADV_HANDLE, data);
    if (rc != 0) {
        ESP_LOGW(TAG, "det ext_adv_set_data failed: %d", rc);
        return;
    }

    rc = ble_gap_ext_adv_start(DET_ADV_HANDLE, 0, 0);
    if (rc != 0 && rc != BLE_HS_EALREADY) {
        ESP_LOGW(TAG, "det ext_adv_start failed: %d", rc);
    }
}

void ble_relay_stop(void)
{
    s_running = false;
}
