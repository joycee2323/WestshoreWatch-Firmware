#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

/* ─────────────────────────────────────────────────────────────────────────────
 * Westshore Remote ID — persistent configuration
 * Stored in NVS namespace "wsd_cfg". Loaded at boot, editable via web UI.
 * ───────────────────────────────────────────────────────────────────────────── */

#define WSD_BLACKLIST_MAX    10
#define WSD_SERIAL_MAX_LEN   32
#define WSD_LABEL_MAX_LEN    32

/* AirAware upload config field sizes */
#define WSD_WIFI_SSID_MAX    64
#define WSD_WIFI_PASS_MAX    64
#define WSD_SERVER_URL_MAX   128
#define WSD_API_KEY_MAX      72    /* 64 hex chars + margin */
#define WSD_NODE_NAME_MAX    64

typedef enum {
    WSD_MODE_RELAY = 0,   /* Wi-Fi receive → BLE re-broadcast */
    WSD_MODE_UART  = 1,   /* Wi-Fi receive → UART JSON only   */
} wsd_mode_t;

typedef enum {
    WSD_WIFI_STRAT_BALANCED = 0,  /* hop all channels, 70% time on tracked drone */
    WSD_WIFI_STRAT_FOLLOW   = 1,  /* dedicate all radio time to tracked drone    */
    WSD_WIFI_STRAT_FIXED    = 2,  /* fixed channel range, no dynamic allocation  */
} wsd_wifi_strategy_t;

typedef struct {
    /* General */
    wsd_mode_t          mode;
    bool                relay_ping_en;               /* broadcast bridge beacon  */
    char                relay_ping_label[WSD_LABEL_MAX_LEN]; /* "DroneScout Bridge" */
    bool                self_id_override;            /* inject Self-ID into relay */
    bool                ble_legacy_en;               /* scan BLE legacy adverts   */
    bool                flash_led_en;                /* LED flashes on detection  */

    /* Reception */
    wsd_wifi_strategy_t wifi_strategy;
    uint8_t             ch_2g_start;                 /* 2.4GHz scan range 1-13   */
    uint8_t             ch_2g_stop;

    /* UART */
    uint32_t            uart_baud;                   /* 9600-115200              */

    /* Blacklist — serial numbers to suppress from relay */
    uint8_t             blacklist_count;
    char                blacklist[WSD_BLACKLIST_MAX][WSD_SERIAL_MAX_LEN];

    /* Timeouts */
    uint16_t            land_timeout_s;              /* seconds grounded → stop  */
    uint16_t            silent_timeout_s;            /* seconds no frames → stop */

    /* ── AirAware cloud upload ──────────────────────────────────────────────── */
    char                wifi_ssid[WSD_WIFI_SSID_MAX];   /* uplink network SSID   */
    char                wifi_pass[WSD_WIFI_PASS_MAX];   /* uplink network pass   */
    char                server_url[WSD_SERVER_URL_MAX]; /* AirAware backend URL  */
    char                api_key[WSD_API_KEY_MAX];       /* node API key          */
    char                node_name[WSD_NODE_NAME_MAX];   /* display name          */
    bool                upload_en;                      /* enable cloud upload   */
    double              node_lat;                       /* node GPS latitude     */
    double              node_lon;                       /* node GPS longitude    */
} wsd_config_t;

/* Global config instance — loaded at boot, used by all modules */
extern wsd_config_t g_config;

/* Populate cfg with factory defaults */
void      wsd_config_defaults(wsd_config_t *cfg);

/* Load from NVS into cfg. Falls back to defaults if no config saved yet. */
esp_err_t wsd_config_load(wsd_config_t *cfg);

/* Persist cfg to NVS */
esp_err_t wsd_config_save(const wsd_config_t *cfg);

/* Returns true if serial is in cfg's blacklist */
bool      wsd_config_is_blacklisted(const wsd_config_t *cfg, const char *serial);
