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

/* 32 chosen as a pragmatic ceiling that covers realistic multi-drone
 * operations, stress scenarios, and adversarial cases without hitting
 * dynamic allocation complexity. At ~400 bytes per slot this is ~13 KB
 * static RAM, negligible on ESP32-C6 (~441 KB usable SRAM). The eviction
 * ESP_LOGI line will flag if this ceiling is ever exceeded in the field,
 * at which point bumping higher is a one-line change. */
#define MAX_CONCURRENT_DRONES 32

static const char *TAG = "BLE_RELAY";

static QueueHandle_t s_queue        = NULL;
static bool          s_running      = false;
static bool          s_inited       = false;
static bool          s_task_created = false;
static uint8_t       s_counter      = 0;

/* Handle 0: ODID relay broadcast (extended adv, legacy PDU, non-connectable).
 *           DroneScout/DJI-app compatible per-message emission stays on this
 *           handle so third-party receivers keep working.
 * Handle 1: ODID Message Pack advertiser (extended PDU, non-connectable).
 *           Bundles basic_id + location + system into a single self-identifying
 *           advertisement for the Westshore Watch phone app, eliminating the
 *           one-uasId-per-sourceMac attribution heuristic that breaks under
 *           multi-drone relay.
 * Handle 2: Westshore Watch detection advertiser (extended PDU, non-connectable,
 *           manufacturer-specific data, company ID 0x08FF)
 * Handle 3: Node identity advertiser (extended PDU, non-connectable,
 *           manufacturer-specific data, company ID 0x08FE — MAC + key prefix) */
#define ADV_HANDLE       0
#define PACK_ADV_HANDLE  1
#define DET_ADV_HANDLE   2
#define ID_ADV_HANDLE    3

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

static bool s_det_adv_configured  = false;
static bool s_id_adv_configured   = false;
static bool s_pack_adv_configured = false;

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
 * ODID Message Pack (msgType 0xF) — self-identifying multi-message advertisement
 *
 * Each pack bundles 3 × 25-byte sub-messages (basic_id + location + system)
 * with a 2-byte pack header:
 *   [0]      msgType<<4 | version = 0xF2
 *   [1]      msg_count = 3
 *   [2..26]  basic_id sub-message (25B)
 *   [27..51] location sub-message (25B)
 *   [52..76] system sub-message (25B)
 *
 * The pack is self-identifying: every packet carries its own basic_id alongside
 * the location/system fields, so the phone doesn't need the fragile "most-recent
 * basic_id on this sourceMac" heuristic. Emitted on PACK_ADV_HANDLE (handle 1),
 * extended PDU, so it exceeds the 31-byte legacy cap without breaking handle 0's
 * DroneScout compatibility.
 * ───────────────────────────────────────────────────────────────────────────── */
#define ODID_PACK_MSG_COUNT  3
#define ODID_PACK_PAYLOAD    (2 + 25 * ODID_PACK_MSG_COUNT)   /* 77 */

static void encode_pack(const odid_detection_t *d, uint8_t *buf)
{
    memset(buf, 0, ODID_PACK_PAYLOAD);
    buf[0] = (ODID_MSG_PACK << 4) | 0x02;
    buf[1] = ODID_PACK_MSG_COUNT;
    encode_basic_id(d, &buf[2]);
    encode_location(d, &buf[2 + 25]);
    encode_system  (d, &buf[2 + 25 + 25]);
}

/* Advertise one ODID Message Pack on PACK_ADV_HANDLE.
 *
 * AD structure (extended PDU):
 *   [0]       length = 1 (type) + 2 (UUID) + 2 (app header) + 77 (pack) = 82
 *   [1]       0x16 (Service Data 16-bit UUID)
 *   [2-3]     UUID = 0xFFFA (LE)
 *   [4]       app code 0x0D
 *   [5]       rolling counter (shared s_counter with handle 0)
 *   [6..82]   77-byte pack payload */
static void advertise_pack(const uint8_t *pack_payload)
{
    if (!s_pack_adv_configured) return;

    const size_t ad_len = 1 + 1 + 2 + 2 + ODID_PACK_PAYLOAD;  /* 83 */
    uint8_t adv_raw[83];
    adv_raw[0] = (uint8_t)(ad_len - 1);  /* length field excludes itself */
    adv_raw[1] = 0x16;
    adv_raw[2] = 0xFA;
    adv_raw[3] = 0xFF;
    adv_raw[4] = 0x0D;
    adv_raw[5] = s_counter++;
    memcpy(&adv_raw[6], pack_payload, ODID_PACK_PAYLOAD);

    struct os_mbuf *data = os_msys_get_pkthdr(ad_len, 0);
    if (!data) {
        ESP_LOGW(TAG, "pack msys_get_pkthdr failed");
        return;
    }
    if (os_mbuf_append(data, adv_raw, ad_len) != 0) {
        ESP_LOGW(TAG, "pack mbuf_append failed");
        os_mbuf_free_chain(data);
        return;
    }

    if (ble_gap_ext_adv_active(PACK_ADV_HANDLE)) {
        ble_gap_ext_adv_stop(PACK_ADV_HANDLE);
    }

    int rc = ble_gap_ext_adv_set_data(PACK_ADV_HANDLE, data);
    if (rc != 0) {
        ESP_LOGW(TAG, "pack ext_adv_set_data failed: %d", rc);
        return;
    }

    rc = ble_gap_ext_adv_start(PACK_ADV_HANDLE, 0, 0);
    if (rc != 0 && rc != BLE_HS_EALREADY) {
        ESP_LOGW(TAG, "pack ext_adv_start failed: %d", rc);
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

/* Overwrite handle-1 (Pack) payload with a benign placeholder so the
 * controller stops re-emitting the last live drone's data after slot
 * eviction. We can't rely on ble_gap_ext_adv_stop alone — on this NimBLE
 * build the host marks the instance inactive but the controller keeps
 * radiating the cached data buffer (see STALE_UAS_AUDIT_ROUND_3 for
 * details). Set + start with a Pack carrying a Basic ID with
 * UAS_ID="DroneScout Bridge" and msg_count=1; the WSW phone filter at
 * maybeEnqueueForUpload (BLEScannerService.kt:357) drops anything with
 * that UAS string, and DroneScout-compatible apps treat it as a bridge
 * marker (same semantics as handle 0). */
static void advertise_pack_idle_placeholder(void)
{
    if (!s_pack_adv_configured) return;
    uint8_t pack_buf[ODID_PACK_PAYLOAD];
    memset(pack_buf, 0, sizeof(pack_buf));
    pack_buf[0] = (ODID_MSG_PACK << 4) | 0x02;
    pack_buf[1] = 1;                          /* msg_count = 1 (just basic_id) */
    encode_bridge_beacon(&pack_buf[2]);       /* basic_id = DroneScout Bridge */
    advertise_pack(pack_buf);
}


/* Per-drone slot — replaces the single-accumulator design that independently
 * overwrote basic_id / location / system across frames from different drones.
 *
 * Attribution: BasicId-bearing frames key the slot by uas_id; bare
 * Location/System frames (no uas_id of their own) fall back to matching by
 * the drone's original BLE MAC — safe here because odid_detection_t.mac is
 * the drone's own address, not the relay's (downstream consumers on the
 * phone side see only the relay MAC and can't use this signal).
 *
 * sys_saved moved per-slot so one drone's RC state can't leak into
 * another's operator-location fallback. */
typedef struct {
    char              uas_id[ODID_STR_LEN + 1];  /* '\0' first byte = free slot */
    uint8_t           src_mac[6];
    bool              mac_known;
    odid_detection_t  acc;
    odid_system_t     sys_saved;
    bool              sys_saved_ok;
    TickType_t        last_any_frame;
    TickType_t        last_valid_airborne;
    TickType_t        first_grounded_after_flight;
    bool              airborne_ever;  /* latched once we observe airborne */
} drone_slot_t;

static drone_slot_t s_slots[MAX_CONCURRENT_DRONES];

#define LOC_VALID(d) ((d).has_location && \
                      (d).location.lat >= -90.0f && (d).location.lat <= 90.0f && \
                      (d).location.lon >= -180.0f && (d).location.lon <= 180.0f && \
                      ((d).location.lat != 0.0f || (d).location.lon != 0.0f))

/* [diag] TEMPORARY — revert after triage. Scan all live slots and log if any
 * two share the same src_mac, which would make MAC-fallback attribution
 * ambiguous and is the suspected residual cross-attribution path. */
static void diag_check_mac_collision(void)
{
    for (int i = 0; i < MAX_CONCURRENT_DRONES; i++) {
        if (s_slots[i].uas_id[0] == '\0' || !s_slots[i].mac_known) continue;
        for (int j = i + 1; j < MAX_CONCURRENT_DRONES; j++) {
            if (s_slots[j].uas_id[0] == '\0' || !s_slots[j].mac_known) continue;
            if (memcmp(s_slots[i].src_mac, s_slots[j].src_mac, 6) == 0) {
                ESP_LOGW(TAG,
                    "[diag] MAC COLLISION slot[%d](%s) and slot[%d](%s) "
                    "both src_mac=%02X:%02X:%02X:%02X:%02X:%02X",
                    i, s_slots[i].uas_id, j, s_slots[j].uas_id,
                    s_slots[i].src_mac[0], s_slots[i].src_mac[1],
                    s_slots[i].src_mac[2], s_slots[i].src_mac[3],
                    s_slots[i].src_mac[4], s_slots[i].src_mac[5]);
            }
        }
    }
}

/* Return the slot owning this frame, claiming a free or stalest slot if the
 * frame carries a new uas_id. Returns NULL when a bare Location/System frame
 * arrives from a MAC we have no prior BasicId context for (drop). */
static drone_slot_t *resolve_slot(const odid_detection_t *det, TickType_t now)
{
    if (det->has_basic_id && det->basic_id.uas_id[0]) {
        for (int i = 0; i < MAX_CONCURRENT_DRONES; i++) {
            if (s_slots[i].uas_id[0] &&
                strncmp(s_slots[i].uas_id, det->basic_id.uas_id,
                        ODID_STR_LEN) == 0) {
                /* [diag] TEMPORARY — log MAC overwrites on existing slots. */
                if (s_slots[i].mac_known &&
                    memcmp(s_slots[i].src_mac, det->mac, 6) != 0) {
                    ESP_LOGI(TAG,
                        "[diag] basicid uas=%s mac=%02X:%02X:%02X:%02X:%02X:%02X "
                        "(overwriting prior mac=%02X:%02X:%02X:%02X:%02X:%02X)",
                        det->basic_id.uas_id,
                        det->mac[0], det->mac[1], det->mac[2],
                        det->mac[3], det->mac[4], det->mac[5],
                        s_slots[i].src_mac[0], s_slots[i].src_mac[1],
                        s_slots[i].src_mac[2], s_slots[i].src_mac[3],
                        s_slots[i].src_mac[4], s_slots[i].src_mac[5]);
                }
                memcpy(s_slots[i].src_mac, det->mac, 6);
                s_slots[i].mac_known = true;
                diag_check_mac_collision();
                return &s_slots[i];
            }
        }
        drone_slot_t *claim = NULL;
        for (int i = 0; i < MAX_CONCURRENT_DRONES; i++) {
            if (s_slots[i].uas_id[0] == '\0') { claim = &s_slots[i]; break; }
        }
        if (!claim) {
            TickType_t oldest = now;
            for (int i = 0; i < MAX_CONCURRENT_DRONES; i++) {
                if (s_slots[i].last_any_frame <= oldest) {
                    oldest = s_slots[i].last_any_frame;
                    claim  = &s_slots[i];
                }
            }
            ESP_LOGI(TAG, "All %d slots full — evicting stalest for new uas_id=%s",
                     MAX_CONCURRENT_DRONES, det->basic_id.uas_id);
        }
        memset(claim, 0, sizeof(*claim));
        strlcpy(claim->uas_id, det->basic_id.uas_id, sizeof(claim->uas_id));
        memcpy(claim->src_mac, det->mac, 6);
        claim->mac_known = true;
        diag_check_mac_collision();
        return claim;
    }

    /* [diag] TEMPORARY — count MAC matches and log fallback outcome so we can
     * see whether ambiguous matches are happening on bare Location frames. */
    drone_slot_t *first_match = NULL;
    int match_count = 0;
    for (int i = 0; i < MAX_CONCURRENT_DRONES; i++) {
        if (s_slots[i].uas_id[0] == '\0' || !s_slots[i].mac_known) continue;
        if (memcmp(s_slots[i].src_mac, det->mac, 6) == 0) {
            if (!first_match) first_match = &s_slots[i];
            match_count++;
        }
    }
    if (match_count == 0) {
        ESP_LOGD(TAG,
            "[diag] mac-fallback miss for incoming mac=%02X:%02X:%02X:%02X:%02X:%02X "
            "— frame dropped",
            det->mac[0], det->mac[1], det->mac[2],
            det->mac[3], det->mac[4], det->mac[5]);
        return NULL;
    }
    if (match_count > 1) {
        ESP_LOGW(TAG,
            "[diag] mac-fallback AMBIGUOUS match_count=%d incoming mac=%02X:%02X:%02X:%02X:%02X:%02X "
            "first_uas=%s — returning first",
            match_count,
            det->mac[0], det->mac[1], det->mac[2],
            det->mac[3], det->mac[4], det->mac[5],
            first_match->uas_id);
    } else {
        ESP_LOGD(TAG,
            "[diag] mac-fallback hit uas=%s for incoming mac=%02X:%02X:%02X:%02X:%02X:%02X",
            first_match->uas_id,
            det->mac[0], det->mac[1], det->mac[2],
            det->mac[3], det->mac[4], det->mac[5]);
    }
    return first_match;
}

/* Merge a frame's populated fields into its slot's accumulator. Each frame
 * updates at most one slot, so basic_id / location / system never mix across
 * drones the way the old single-acc MERGE did. */
static void merge_into_slot(drone_slot_t *s, const odid_detection_t *det,
                            TickType_t now)
{
    s->acc.rssi   = det->rssi;
    s->acc.source = det->source;
    if (det->has_basic_id) { s->acc.basic_id = det->basic_id; s->acc.has_basic_id = true; }
    if (LOC_VALID(*det))   { s->acc.location = det->location; s->acc.has_location = true; }
    if (det->has_system)   { s->acc.system   = det->system;   s->acc.has_system   = true; }
    if (det->has_self_id)  { s->acc.self_id  = det->self_id;  s->acc.has_self_id  = true; }
    s->last_any_frame = now;

    /* sys_saved rules — unchanged semantics, scoped per-slot. */
    if (s->acc.has_system && s->acc.has_basic_id) {
        float op_lat = s->acc.system.operator_lat;
        float op_lon = s->acc.system.operator_lon;
        bool op_coords_valid =
            (op_lat >= -90.0f && op_lat <= 90.0f &&
             op_lon >= -180.0f && op_lon <= 180.0f &&
             (op_lat != 0.0f || op_lon != 0.0f));

        if (!s->sys_saved_ok) {
            if (op_coords_valid) {
                s->sys_saved    = s->acc.system;
                s->sys_saved_ok = true;
                ESP_LOGI(TAG, "Operator location saved (uas_id=%s): lat=%.6f lon=%.6f",
                         s->uas_id, (double)op_lat, (double)op_lon);
            } else {
                ESP_LOGD(TAG, "Skipping invalid operator coords (uas_id=%s): lat=%.6f lon=%.6f",
                         s->uas_id, (double)op_lat, (double)op_lon);
                s->acc.has_system = false;
            }
        } else if (!op_coords_valid) {
            s->acc.system = s->sys_saved;
        }
    } else if (s->acc.has_system && !s->acc.has_basic_id) {
        ESP_LOGD(TAG, "Deferring operator location — waiting for Basic ID");
        s->acc.has_system = false;
    }

    if (!s->sys_saved_ok && s->acc.has_basic_id && LOC_VALID(s->acc) &&
        s->acc.has_location && s->acc.location.status == OP_STATUS_AIRBORNE) {
        s->sys_saved = s->acc.system;
        s->sys_saved.operator_lat = s->acc.location.lat;
        s->sys_saved.operator_lon = s->acc.location.lon;
        s->sys_saved_ok = true;
        ESP_LOGI(TAG, "Operator location fallback — drone pos (uas_id=%s): lat=%.6f lon=%.6f",
                 s->uas_id, (double)s->acc.location.lat, (double)s->acc.location.lon);
    }

    if (s->sys_saved_ok && !s->acc.has_system) {
        s->acc.system     = s->sys_saved;
        s->acc.has_system = true;
    }
}

/* Per-slot silence/landing eviction. Returns true if the slot was freed. */
static bool maybe_evict_slot(drone_slot_t *s, TickType_t now)
{
    if (s->uas_id[0] == '\0') return false;

    /* Eviction thresholds come from g_config (captive-portal editable, NVS
     * persisted). Defaults: land=5s, silent=15s. Previously hardcoded;
     * the portal fields existed but were never read at runtime. */
    uint32_t land_ms   = (uint32_t)g_config.land_timeout_s   * 1000U;
    uint32_t silent_ms = (uint32_t)g_config.silent_timeout_s * 1000U;

    bool landed = (s->first_grounded_after_flight != 0 &&
                   (now - s->first_grounded_after_flight) > pdMS_TO_TICKS(land_ms));
    bool silent = (s->last_valid_airborne != 0 &&
                   (now - s->last_valid_airborne) > pdMS_TO_TICKS(silent_ms));
    bool any_silent = (s->last_any_frame != 0 &&
                       (now - s->last_any_frame) > pdMS_TO_TICKS(silent_ms));

    if (landed || silent || any_silent) {
        ESP_LOGI(TAG, "Drone %s %s — evicting slot",
                 s->uas_id,
                 landed ? "landed" : (silent ? "silent" : "no frames"));
        memset(s, 0, sizeof(*s));
        return true;
    }
    return false;
}

static int count_live_slots(void)
{
    int n = 0;
    for (int i = 0; i < MAX_CONCURRENT_DRONES; i++) {
        if (s_slots[i].uas_id[0] && s_slots[i].acc.has_basic_id) n++;
    }
    return n;
}

static void relay_task(void *arg)
{
    odid_detection_t det;

    ESP_LOGI(TAG, "Relay task running");

    memset(s_slots, 0, sizeof(s_slots));

    /* ── Relay strategy ────────────────────────────────────────────────────────
     * Two parallel emission paths:
     *
     * 1. Handle 0 (legacy PDU, DroneScout-compatible): per-slot burst with
     *    basic + location + system every cycle (option A), self_id every 5th.
     *    Emitting basic every cycle guarantees every bare Location on the wire
     *    has an immediately-preceding basic_id within ~50ms intra-burst gap,
     *    collapsing the phone's attribution inheritance window.
     *
     * 2. Handle 1 (extended PDU, self-identifying): per-slot ODID Message Pack
     *    every cycle (option C). Each pack bundles {basic_id, location, system}
     *    for one drone in one advertisement. The Westshore Watch phone app
     *    bypasses sourceMac-based attribution entirely when parsing pack
     *    msgType 0xF.
     *
     * Handle 0 and handle 1 run concurrently. DroneScout/DJI apps see legacy
     * emission; Westshore Watch prefers packs.
     * ────────────────────────────────────────────────────────────────────────── */
    uint8_t basic_buf[25] = {0};
    uint8_t loc_buf[25]   = {0};
    uint8_t self_buf[25]  = {0};
    uint8_t sys_buf[25]   = {0};
    uint8_t pack_buf[ODID_PACK_PAYLOAD] = {0};
    uint8_t cycle = 0;

    /* Bridge beacon — Basic ID with UAS ID = "DroneScout Bridge", ua_type=Other.
     * Broadcast when no slot is live, matching real bridge behavior. */
    uint8_t bridge_buf[25] = {0};
    encode_bridge_beacon(bridge_buf);

    while (s_running) {

        bool got_frame = (xQueueReceive(s_queue, &det, pdMS_TO_TICKS(50)) == pdTRUE);
        TickType_t now = xTaskGetTickCount();

        if (got_frame) {
            drone_slot_t *s = resolve_slot(&det, now);
            if (s) {
                merge_into_slot(s, &det, now);

                bool airborne = s->acc.has_location &&
                                s->acc.location.status == OP_STATUS_AIRBORNE;
                if (airborne) {
                    s->last_valid_airborne = now;
                    s->first_grounded_after_flight = 0;
                    s->airborne_ever = true;
                } else if (s->airborne_ever) {
                    if (s->first_grounded_after_flight == 0)
                        s->first_grounded_after_flight = now;
                }
            }
        }

        /* Per-slot eviction (runs every tick, covers silent slots even when
         * no frame is received). */
        for (int i = 0; i < MAX_CONCURRENT_DRONES; i++) {
            maybe_evict_slot(&s_slots[i], now);
        }

        int live = count_live_slots();
        if (live == 0) {
            ble_gap_ext_adv_stop(ADV_HANDLE);             /* keep — pre-step for advertise_odid */
            advertise_pack_idle_placeholder();            /* overwrite handle 1 (was: stop) */
            ble_detection_advertise_stop();               /* now sets handle 2 to placeholder */
            /* LED: no per-packet flash here — the XIAO LED indicates
             * live RID packets, not relay cycle state. Flashes are
             * fired from the BLE/WiFi packet callbacks directly. */
            advertise_odid(bridge_buf);
            vTaskDelay(pdMS_TO_TICKS(50));
            cycle = 0;
            continue;
        }

        /* Broadcast each live slot's full cycle back-to-back. Each burst is
         * self-consistent (basic_id + location + system all from one drone),
         * so the phone's attributionBySource window collapses from ~500ms
         * (old global cycle) to ~50ms (slot-to-slot gap). */
        for (int i = 0; i < MAX_CONCURRENT_DRONES; i++) {
            drone_slot_t *s = &s_slots[i];
            if (s->uas_id[0] == '\0' || !s->acc.has_basic_id) continue;

            odid_detection_t out = s->acc;
            bool loc_valid = LOC_VALID(out);

            encode_basic_id(&out, basic_buf);
            if (loc_valid) encode_location(&out, loc_buf);
            if (out.has_system) encode_system(&out, sys_buf);
            encode_self_id_signal(&out, self_buf, out.rssi, out.source);

            /* Option C: emit a self-identifying pack on handle 1 every cycle
             * for every live slot. The Westshore Watch phone app consumes
             * these; attribution is resolved in-packet via the embedded
             * basic_id. Handle 0 below continues to emit legacy per-message
             * ODID for DroneScout compatibility.
             *
             * Packs require location to be valid (the pack carries a
             * location sub-message; an empty one would mislead receivers).
             * If location is missing this cycle we skip the pack and let
             * legacy emission on handle 0 carry what we have. */
            if (loc_valid) {
                encode_pack(&out, pack_buf);
                advertise_pack(pack_buf);
            }

            /* Option A: basic_id now emits every cycle (not just cycle 0).
             * Locations arriving at the phone on handle 0 will always have
             * an immediately-preceding basic_id from the same slot within
             * the same burst, shrinking the attribution inheritance window
             * from one every-5th-cycle gap to a tight ~50ms intra-burst gap.
             * self_id stays gated to cycle 0 — its frequency is irrelevant
             * to attribution (Self-Id is the "DroneScout Bridge" tag). */
            advertise_odid(basic_buf);
            vTaskDelay(pdMS_TO_TICKS(50));
            if (cycle == 0) {
                advertise_odid(self_buf);
                vTaskDelay(pdMS_TO_TICKS(50));
            }
            if (loc_valid) {
                advertise_odid(loc_buf);
                vTaskDelay(pdMS_TO_TICKS(50));
            }
            if (out.has_system) {
                advertise_odid(sys_buf);
                vTaskDelay(pdMS_TO_TICKS(50));
            } else {
                vTaskDelay(pdMS_TO_TICKS(50));
            }

            /* [diag] TEMPORARY — log every cycle, not every 5th, so we can
             * verify per-slot ground truth on what the firmware is actually
             * broadcasting. Revert after triage. */
            ESP_LOGI(TAG, "[diag] broadcast slot[%d] uas=%s lat=%.6f lon=%.6f",
                     i, s->uas_id,
                     loc_valid ? (double)out.location.lat : 0.0,
                     loc_valid ? (double)out.location.lon : 0.0);
            if (cycle == 0) {
                ESP_LOGI(TAG, "Relay[%s]: airborne=%d op_lat=%.6f op_lon=%.6f",
                         s->uas_id,
                         out.has_location && out.location.status == OP_STATUS_AIRBORNE,
                         out.has_system ? (double)out.system.operator_lat : 0.0,
                         out.has_system ? (double)out.system.operator_lon : 0.0);
            }
        }

        cycle = (cycle + 1) % 5;
    }

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
 * Pack advertiser (handle 1) — extended PDU ODID Message Pack
 * ───────────────────────────────────────────────────────────────────────────── */
static int configure_pack_advertiser(void)
{
    struct ble_gap_ext_adv_params params;
    memset(&params, 0, sizeof(params));
    params.legacy_pdu    = 0;        /* extended PDU — 83-byte AD exceeds legacy 31B cap */
    params.connectable   = 0;
    params.scannable     = 0;
    params.own_addr_type = BLE_OWN_ADDR_PUBLIC;
    params.primary_phy   = BLE_HCI_LE_PHY_1M;
    params.secondary_phy = BLE_HCI_LE_PHY_1M;
    params.itvl_min      = BLE_GAP_ADV_ITVL_MS(100);
    params.itvl_max      = BLE_GAP_ADV_ITVL_MS(150);
    params.sid           = 1;
    params.tx_power      = 127;

    int rc = ble_gap_ext_adv_configure(PACK_ADV_HANDLE, &params,
                                       NULL, NULL, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "pack ext_adv_configure (handle %d) failed: %d",
                 PACK_ADV_HANDLE, rc);
        return rc;
    }

    s_pack_adv_configured = true;
    ESP_LOGI(TAG, "Pack advertiser configured on handle %d", PACK_ADV_HANDLE);
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

    /* Handle 1 — extended PDU for self-identifying ODID Message Pack. */
    configure_pack_advertiser();

    /* Handle 2 — extended PDU for Westshore Watch detection advertising. */
    configure_detection_advertiser();

    /* If ble_relay_start() was called before the host synced, start the task
     * now. If relay mode is disabled, this is a no-op. */
    spawn_relay_task_if_ready();
}

static void on_reset(int reason)
{
    ESP_LOGW(TAG, "BLE host reset: %d", reason);
    s_det_adv_configured  = false;
    s_id_adv_configured   = false;
    s_pack_adv_configured = false;
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

void ble_detection_advertise_stop(void)
{
    if (!s_det_adv_configured) return;
    /* Same NimBLE caveat as the Pack handle: a bare ext_adv_stop on this
     * build doesn't actually halt the radio. Overwrite the AD payload with
     * a tiny placeholder JSON instead. The Android app's ScanFilter at
     * BLEScannerService.kt:230 doesn't ingest manufacturer 0x08FF anyway,
     * so this is purely about silencing the controller's broadcast loop. */
    static const char kPlaceholder[] = "{\"id\":\"\"}";
    ble_detection_advertise(kPlaceholder, sizeof(kPlaceholder) - 1);
}

void ble_relay_stop(void)
{
    s_running = false;
}
