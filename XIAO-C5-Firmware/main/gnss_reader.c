#include "gnss_reader.h"
#include "cellular_uart.h"
#include "esp_log.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "GNSS";

#define AT_TIMEOUT_MS   5000
#define RESP_BUF_SIZE   256

static gnss_position_t s_position;
static volatile bool   s_valid;

bool gnss_reader_get_position(gnss_position_t *out)
{
    if (!s_valid) return false;
    *out = s_position;
    return true;
}

/**
 * Parse AT+CGPSINF=0 response from SIM7600.
 * Format: +CGPSINF: mode,lat,lon,alt,utc,ttff,num,speed,course,...
 */
static bool parse_cgpsinf(const char *resp)
{
    const char *p = strstr(resp, "+CGPSINF:");
    if (!p) return false;
    p = strchr(p, ':');
    if (!p) return false;
    p++;

    while (*p == ' ') p++;

    char *end;
    strtol(p, &end, 10);
    if (*end != ',') return false;
    p = end + 1;

    double lat = strtod(p, &end);
    if (*end != ',') return false;
    p = end + 1;

    double lon = strtod(p, &end);
    if (*end != ',') return false;
    p = end + 1;

    float alt = strtof(p, &end);
    if (*end != ',') return false;
    p = end + 1;

    for (int i = 0; i < 3; i++) {
        p = strchr(p, ',');
        if (!p) break;
        p++;
    }

    float hdop = 99.0f;
    if (p) {
        p = strchr(p, ',');
        if (p) { p++; p = strchr(p, ','); }
        if (p) {
            p++;
            float h = strtof(p, &end);
            if (end != p && h > 0 && h < 100) hdop = h;
        }
    }

    if (lat == 0.0 && lon == 0.0) return false;
    if (lat < -90 || lat > 90 || lon < -180 || lon > 180) return false;

    s_position.lat   = lat;
    s_position.lon   = lon;
    s_position.alt_m = alt;
    s_position.hdop  = hdop;
    s_position.valid = true;
    s_valid = true;

    ESP_LOGI(TAG, "fix acquired: %.6f, %.6f  alt=%.1fm  hdop=%.1f",
             lat, lon, alt, hdop);
    return true;
}

esp_err_t gnss_reader_start(void)
{
    memset(&s_position, 0, sizeof(s_position));
    s_valid = false;
    ESP_LOGI(TAG, "GNSS reader initialized (fix acquired via modem_manager)");
    return ESP_OK;
}

/**
 * Acquire GNSS fix using cellular_uart AT commands.
 *
 * Called by modem_manager from within at_phase(), AFTER AT+CGNSPWR=1
 * and BEFORE the UART is handed to esp_modem for PPP.  This avoids
 * switching the modem out of PPP data mode for AT commands, which
 * would disrupt upload reliability.
 *
 * The node is stationary, so we acquire one fix and cache it forever.
 * Best-effort: if GNSS doesn't fix within max_attempts, uploads
 * proceed without node_position.
 *
 * @param max_attempts  Number of polling attempts (5s apart)
 */
void gnss_reader_acquire_fix(int max_attempts)
{
    char resp[RESP_BUF_SIZE];

    ESP_LOGI(TAG, "acquiring GNSS fix (up to %d attempts, %ds apart)...",
             max_attempts, 5);

    for (int i = 0; i < max_attempts; i++) {
        vTaskDelay(pdMS_TO_TICKS(5000));

        esp_err_t err = cellular_uart_send_at("AT+CGPSINF=0",
                                              resp, sizeof(resp), AT_TIMEOUT_MS);
        if (err == ESP_OK && parse_cgpsinf(resp)) {
            return;
        }

        if (i % 12 == 0 && i > 0) {
            ESP_LOGI(TAG, "GNSS: still acquiring (%d/%d)...", i, max_attempts);
        }
    }

    ESP_LOGW(TAG, "GNSS fix not acquired after %d attempts — "
             "node_position will be omitted from uploads", max_attempts);
}
