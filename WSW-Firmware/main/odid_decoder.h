#pragma once
#include <stdint.h>
#include <stdbool.h>

// ── ODID Message Types ────────────────────────────────────────────────────────
#define ODID_MSG_BASIC_ID       0x0
#define ODID_MSG_LOCATION       0x1
#define ODID_MSG_AUTH           0x2
#define ODID_MSG_SELF_ID        0x3
#define ODID_MSG_SYSTEM         0x4
#define ODID_MSG_OPERATOR_ID    0x5
#define ODID_MSG_PACK           0xF

// ── BLE / Wi-Fi identifiers ───────────────────────────────────────────────────
#define ODID_BLE_SERVICE_UUID   0xFFFA      // 16-bit UUID in service data AD
#define ODID_WIFI_OUI_0         0xFA
#define ODID_WIFI_OUI_1         0x0B
#define ODID_WIFI_OUI_2         0xBC
#define ODID_WIFI_ACTION_CAT    0x04        // Vendor-Specific action frame category
#define ODID_WIFI_ACTION_TYPE   0x0D        // OUI sub-type for Remote ID

// ── UA Types ──────────────────────────────────────────────────────────────────
typedef enum {
    UA_TYPE_NONE            = 0,
    UA_TYPE_AEROPLANE       = 1,
    UA_TYPE_ROTORCRAFT      = 2,
    UA_TYPE_GYROPLANE       = 3,
    UA_TYPE_HYBRID_LIFT     = 4,
    UA_TYPE_ORNITHOPTER     = 5,
    UA_TYPE_GLIDER          = 6,
    UA_TYPE_KITE            = 7,
    UA_TYPE_FREE_BALLOON    = 8,
    UA_TYPE_CAPTIVE_BALLOON = 9,
    UA_TYPE_AIRSHIP         = 10,
    UA_TYPE_FREE_FALL       = 11,
    UA_TYPE_ROCKET          = 12,
    UA_TYPE_TETHERED_POWERED= 13,
    UA_TYPE_GROUND_OBSTACLE = 14,
    UA_TYPE_OTHER           = 15,
} ua_type_t;

typedef enum {
    ID_TYPE_NONE            = 0,
    ID_TYPE_SERIAL          = 1,   // ANSI/CTA-2063-A Serial Number
    ID_TYPE_CAA_REG         = 2,   // CAA Registration ID
    ID_TYPE_UTM_ASSIGNED    = 3,   // UTM (USS) Assigned ID
    ID_TYPE_SPECIFIC_SESSION= 4,
} id_type_t;

typedef enum {
    OP_STATUS_UNDECLARED    = 0,
    OP_STATUS_GROUND        = 1,
    OP_STATUS_AIRBORNE      = 2,
    OP_STATUS_EMERGENCY     = 3,
    OP_STATUS_REMOTE_ID_SYS_FAILURE = 4,
} op_status_t;

// ── Source of detection ───────────────────────────────────────────────────────
typedef enum {
    ODID_SRC_BT_LEGACY  = 0,   // Bluetooth 4.x / legacy advertising
    ODID_SRC_BT5        = 1,   // Bluetooth 5 extended advertising
    ODID_SRC_WIFI_B     = 2,   // Wi-Fi 802.11b
    ODID_SRC_WIFI_N     = 3,   // Wi-Fi 802.11n
} odid_source_t;

// ── Decoded message structs ───────────────────────────────────────────────────
#define ODID_STR_LEN    24

typedef struct {
    id_type_t   id_type;
    ua_type_t   ua_type;
    char        uas_id[ODID_STR_LEN + 1];  // null-terminated
} odid_basic_id_t;

typedef struct {
    op_status_t status;
    float       lat;            // degrees
    float       lon;            // degrees
    float       alt_baro;       // metres MSL
    float       alt_geo;        // metres WGS84
    float       height;         // metres AGL
    float       speed_horiz;    // m/s
    float       speed_vert;     // m/s  (+ = up)
    uint16_t    heading;        // degrees true (0..359)
    uint32_t    timestamp;      // tenths of second since hour
    uint8_t     horiz_acc;      // encoded horizontal accuracy
    uint8_t     vert_acc;       // encoded vertical accuracy
    uint8_t     baro_acc;       // encoded baro accuracy
    uint8_t     speed_acc;      // encoded speed accuracy
} odid_location_t;

typedef struct {
    char        description[24];
} odid_self_id_t;

typedef struct {
    double      operator_lat;
    double      operator_lon;
    float       operator_alt_geo;
    uint32_t    area_count;
    uint32_t    area_radius;
    float       area_ceiling;
    float       area_floor;
    uint8_t     category;
    uint8_t     class_value;
} odid_system_t;

typedef struct {
    char        operator_id[20];
    uint8_t     operator_id_type;
} odid_operator_id_t;

// ── Detection event (posted to output queue) ─────────────────────────────────
typedef struct {
    odid_source_t   source;
    int8_t          rssi;
    uint8_t         mac[6];
    bool            has_basic_id;
    bool            has_location;
    bool            has_self_id;
    bool            has_system;
    bool            has_operator_id;
    odid_basic_id_t     basic_id;
    odid_location_t     location;
    odid_self_id_t      self_id;
    odid_system_t       system;
    odid_operator_id_t  operator_id;
} odid_detection_t;

// ── Parser API ────────────────────────────────────────────────────────────────

/**
 * Parse a single 25-byte ODID message frame.
 * Returns the message type (ODID_MSG_*) or -1 on failure.
 */
int odid_parse_message(const uint8_t *buf, uint8_t len,
                       odid_detection_t *out);

/**
 * Parse an ODID Message Pack (type 0xF) that bundles multiple messages.
 * Calls odid_parse_message() for each bundled message.
 * Returns number of messages parsed.
 */
int odid_parse_pack(const uint8_t *buf, uint8_t len,
                    odid_detection_t *out);
