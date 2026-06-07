#include "status_led.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"

static const char *TAG = "STATUS_LED";

/* RED/YELLOW two-color LED — there is NO green element (red, yellow, black
 * common-cathode leads; no green wire). Common cathode, active HIGH. Verified
 * on hardware: driving GPIO8 lights RED, driving GPIO12 lights YELLOW. Driving
 * BOTH together makes red dominate (no clean blend), so every state drives
 * exactly one element (active HIGH, the other explicitly LOW) and blink rate
 * separates the look-alike colors. */
#define LED_RED_GPIO     8      /* header D8 — RED element    */
#define LED_YELLOW_GPIO  12     /* header D7 — YELLOW element */

/* Blink cadences: fast = urgent (degraded/red), slow = transient (warming/
 * offline yellow). Distinct enough to read at a glance from across a room. */
#define BLINK_FAST_US   150000  /* 150 ms  → ~3.3 Hz */
#define BLINK_SLOW_US   600000  /* 600 ms  → ~0.8 Hz */

static esp_timer_handle_t s_blink_timer;
static bool s_blink_on;
static bool s_blink_is_red;     /* which single element toggles while blinking */

/* Drive both pins to definite levels. Callers pass at most one `true` so the
 * red+yellow "both on" (red-dominant) state can never occur unintentionally. */
static void drive(bool red_on, bool yellow_on)
{
    gpio_set_level(LED_RED_GPIO,    red_on    ? 1 : 0);
    gpio_set_level(LED_YELLOW_GPIO, yellow_on ? 1 : 0);
}

static void blink_cb(void *arg)
{
    s_blink_on = !s_blink_on;
    if (s_blink_is_red) drive(s_blink_on, false);   /* red blink, yellow off  */
    else                drive(false, s_blink_on);    /* yellow blink, red off  */
}

esp_err_t status_led_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << LED_RED_GPIO) | (1ULL << LED_YELLOW_GPIO),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&io);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "gpio_config failed: %s", esp_err_to_name(err));
        return err;
    }

    const esp_timer_create_args_t targs = {
        .callback = blink_cb,
        .name     = "led_blink",
    };
    err = esp_timer_create(&targs, &s_blink_timer);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "blink timer create failed: %s", esp_err_to_name(err));
    }

    drive(false, false);
    ESP_LOGI(TAG, "init: red=GPIO%d yellow=GPIO%d (no green element)",
             LED_RED_GPIO, LED_YELLOW_GPIO);
    return ESP_OK;
}

void status_led_set(status_led_state_t state)
{
    /* Stop any active blink timer before changing state. */
    if (s_blink_timer) {
        esp_timer_stop(s_blink_timer);
    }

    switch (state) {
    case STATUS_LED_OFF:
        drive(false, false);
        break;

    case STATUS_LED_HEALTHY:        /* solid yellow */
        drive(false, true);
        break;

    case STATUS_LED_FAULT:          /* solid red */
        drive(true, false);
        break;

    case STATUS_LED_WARMING:        /* slow-blink yellow */
        s_blink_is_red = false;
        s_blink_on = true;
        drive(false, true);
        if (s_blink_timer) {
            esp_timer_start_periodic(s_blink_timer, BLINK_SLOW_US);
        }
        break;

    case STATUS_LED_DEGRADED:       /* fast-blink red */
        s_blink_is_red = true;
        s_blink_on = true;
        drive(true, false);
        if (s_blink_timer) {
            esp_timer_start_periodic(s_blink_timer, BLINK_FAST_US);
        }
        break;
    }
    ESP_LOGI(TAG, "state → %d", (int)state);
}
