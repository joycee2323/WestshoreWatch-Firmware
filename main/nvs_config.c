#include "nvs_config.h"
#include "nvs.h"
#include "esp_log.h"
#include <string.h>
#include <stdio.h>

static const char *TAG     = "NVS_CFG";
static const char *NVS_NS  = "wsd_cfg";

/* Global config instance */
wsd_config_t g_config;

/* ─────────────────────────────────────────────────────────────────────────────
 * Factory defaults
 * ───────────────────────────────────────────────────────────────────────────── */
void wsd_config_defaults(wsd_config_t *cfg)
{
    memset(cfg, 0, sizeof(*cfg));
    cfg->mode              = WSD_MODE_RELAY;
    cfg->relay_ping_en     = true;
    strlcpy(cfg->relay_ping_label, "DroneScout Bridge",
            sizeof(cfg->relay_ping_label));
    cfg->self_id_override  = true;
    cfg->ble_legacy_en     = false;
    cfg->flash_led_en      = true;
    cfg->wifi_strategy     = WSD_WIFI_STRAT_BALANCED;
    cfg->ch_2g_start       = 1;
    cfg->ch_2g_stop        = 13;
    cfg->uart_baud         = 115200;
    cfg->blacklist_count   = 0;
    cfg->land_timeout_s    = 5;
    cfg->silent_timeout_s  = 15;

    /* AirAware defaults — empty until configured via captive portal */
    cfg->wifi_ssid[0]  = '\0';
    cfg->wifi_pass[0]  = '\0';
    cfg->server_url[0] = '\0';
    cfg->api_key[0]    = '\0';
    cfg->node_name[0]  = '\0';
    cfg->upload_en     = false;
    cfg->node_lat      = 0.0;
    cfg->node_lon      = 0.0;
}

/* ─────────────────────────────────────────────────────────────────────────────
 * Load from NVS
 * ───────────────────────────────────────────────────────────────────────────── */
esp_err_t wsd_config_load(wsd_config_t *cfg)
{
    wsd_config_defaults(cfg);

    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS, NVS_READONLY, &h);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "No saved config — using factory defaults");
        return ESP_OK;
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "NVS open failed: %d — using defaults", err);
        return ESP_OK;
    }

    uint8_t  u8;
    uint32_t u32;
    size_t   len;

#define LOAD_U8(key, field)  do { if (nvs_get_u8 (h, key, &u8)  == ESP_OK) cfg->field = u8;  } while(0)
#define LOAD_U32(key, field) do { if (nvs_get_u32(h, key, &u32) == ESP_OK) cfg->field = u32; } while(0)
#define LOAD_STR(key, field) do { len = sizeof(cfg->field); nvs_get_str(h, key, cfg->field, &len); } while(0)

    LOAD_U8 ("mode",        mode);
    LOAD_U8 ("ping_en",     relay_ping_en);
    LOAD_STR("ping_label",  relay_ping_label);
    LOAD_U8 ("self_id_ovr", self_id_override);
    LOAD_U8 ("ble_en",      ble_legacy_en);
    LOAD_U8 ("led_en",      flash_led_en);
    LOAD_U8 ("wifi_strat",  wifi_strategy);
    LOAD_U8 ("ch_start",    ch_2g_start);
    LOAD_U8 ("ch_stop",     ch_2g_stop);
    LOAD_U32("uart_baud",   uart_baud);
    LOAD_U8 ("land_tout",   land_timeout_s);
    LOAD_U8 ("silent_tout", silent_timeout_s);

    LOAD_U8("bl_count", blacklist_count);
    if (cfg->blacklist_count > WSD_BLACKLIST_MAX)
        cfg->blacklist_count = WSD_BLACKLIST_MAX;
    for (int i = 0; i < cfg->blacklist_count; i++) {
        char key[8];
        snprintf(key, sizeof(key), "bl_%d", i);
        len = WSD_SERIAL_MAX_LEN;
        nvs_get_str(h, key, cfg->blacklist[i], &len);
    }

    /* AirAware upload fields */
    LOAD_STR("aw_ssid",    wifi_ssid);
    LOAD_STR("aw_pass",    wifi_pass);
    LOAD_STR("aw_url",     server_url);
    LOAD_STR("aw_key",     api_key);
    LOAD_STR("aw_name",    node_name);
    LOAD_U8 ("aw_en",      upload_en);

    /* lat/lon stored as fixed-point integers (degrees * 1e6) */
    int32_t lat6 = 0, lon6 = 0;
    nvs_get_i32(h, "aw_lat6", &lat6);
    nvs_get_i32(h, "aw_lon6", &lon6);
    cfg->node_lat = lat6 / 1000000.0;
    cfg->node_lon = lon6 / 1000000.0;

#undef LOAD_U8
#undef LOAD_U32
#undef LOAD_STR

    nvs_close(h);
    ESP_LOGI(TAG, "Config loaded (mode=%d ch=%d-%d bl=%d upload=%s ssid='%s')",
             cfg->mode, cfg->ch_2g_start, cfg->ch_2g_stop,
             cfg->blacklist_count,
             cfg->upload_en ? "on" : "off",
             cfg->wifi_ssid[0] ? cfg->wifi_ssid : "(none)");
    return ESP_OK;
}

/* ─────────────────────────────────────────────────────────────────────────────
 * Save to NVS
 * ───────────────────────────────────────────────────────────────────────────── */
esp_err_t wsd_config_save(const wsd_config_t *cfg)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS open for write failed: %d", err);
        return err;
    }

    nvs_set_u8 (h, "mode",        cfg->mode);
    nvs_set_u8 (h, "ping_en",     cfg->relay_ping_en);
    nvs_set_str(h, "ping_label",  cfg->relay_ping_label);
    nvs_set_u8 (h, "self_id_ovr", cfg->self_id_override);
    nvs_set_u8 (h, "ble_en",      cfg->ble_legacy_en);
    nvs_set_u8 (h, "led_en",      cfg->flash_led_en);
    nvs_set_u8 (h, "wifi_strat",  cfg->wifi_strategy);
    nvs_set_u8 (h, "ch_start",    cfg->ch_2g_start);
    nvs_set_u8 (h, "ch_stop",     cfg->ch_2g_stop);
    nvs_set_u32(h, "uart_baud",   cfg->uart_baud);
    nvs_set_u8 (h, "land_tout",   (uint8_t)cfg->land_timeout_s);
    nvs_set_u8 (h, "silent_tout", (uint8_t)cfg->silent_timeout_s);
    nvs_set_u8 (h, "bl_count",    cfg->blacklist_count);

    for (int i = 0; i < cfg->blacklist_count && i < WSD_BLACKLIST_MAX; i++) {
        char key[8];
        snprintf(key, sizeof(key), "bl_%d", i);
        nvs_set_str(h, key, cfg->blacklist[i]);
    }
    for (int i = cfg->blacklist_count; i < WSD_BLACKLIST_MAX; i++) {
        char key[8];
        snprintf(key, sizeof(key), "bl_%d", i);
        nvs_erase_key(h, key);
    }

    /* AirAware upload fields */
    nvs_set_str(h, "aw_ssid", cfg->wifi_ssid);
    nvs_set_str(h, "aw_pass", cfg->wifi_pass);
    nvs_set_str(h, "aw_url",  cfg->server_url);
    nvs_set_str(h, "aw_key",  cfg->api_key);
    nvs_set_str(h, "aw_name", cfg->node_name);
    nvs_set_u8 (h, "aw_en",   cfg->upload_en);
    nvs_set_i32(h, "aw_lat6", (int32_t)(cfg->node_lat * 1000000.0));
    nvs_set_i32(h, "aw_lon6", (int32_t)(cfg->node_lon * 1000000.0));

    err = nvs_commit(h);
    nvs_close(h);
    if (err == ESP_OK)
        ESP_LOGI(TAG, "Config saved to NVS");
    else
        ESP_LOGE(TAG, "NVS commit failed: %d", err);
    return err;
}

/* ─────────────────────────────────────────────────────────────────────────────
 * Blacklist check
 * ───────────────────────────────────────────────────────────────────────────── */
bool wsd_config_is_blacklisted(const wsd_config_t *cfg, const char *serial)
{
    if (!serial || !serial[0]) return false;
    for (int i = 0; i < cfg->blacklist_count; i++) {
        if (strncmp(cfg->blacklist[i], serial, WSD_SERIAL_MAX_LEN) == 0)
            return true;
    }
    return false;
}
