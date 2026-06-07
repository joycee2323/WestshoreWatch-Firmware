#include "status_led.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"

static const char *TAG = "STATUS_LED";

/* The LED's leads are reversed vs the D7=red / D8=green silk. Verified on
 * hardware: at boot the firmware commands YELLOW (both pins HIGH) and the LED
 * shows yellow (both elements lit), but commanding GREEN (green pin HIGH, red
 * pin LOW) lit RED — so GPIO8 physically drives the RED element and GPIO12 the
 * GREEN element. (Boot-yellow rules out a common-anode part, which would read
 * dark with both pins HIGH.) The assignments below name the ACTUAL element. */
#define LED_RED_GPIO    8       /* header D8 — physically the RED element   */
#define LED_GREEN_GPIO  12      /* header D7 — physically the GREEN element */

static esp_timer_handle_t s_blink_timer;
static bool s_blink_on;
static bool s_blink_red;        /* element(s) lit on the blink's "on" phase */
static bool s_blink_green;

static void set_rgb(bool red, bool green)
{
    gpio_set_level(LED_RED_GPIO, red ? 1 : 0);
    gpio_set_level(LED_GREEN_GPIO, green ? 1 : 0);
}

static void blink_cb(void *arg)
{
    s_blink_on = !s_blink_on;
    set_rgb(s_blink_on && s_blink_red, s_blink_on && s_blink_green);
}

esp_err_t status_led_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << LED_RED_GPIO) | (1ULL << LED_GREEN_GPIO),
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

    set_rgb(false, false);
    ESP_LOGI(TAG, "init: red=GPIO%d green=GPIO%d", LED_RED_GPIO, LED_GREEN_GPIO);
    return ESP_OK;
}

void status_led_set(status_led_state_t state)
{
    /* Stop any active blink timer before changing state */
    if (s_blink_timer) {
        esp_timer_stop(s_blink_timer);
    }

    switch (state) {
    case STATUS_LED_OFF:
        set_rgb(false, false);
        break;
    case STATUS_LED_GREEN:
        set_rgb(false, true);
        break;
    case STATUS_LED_YELLOW:
        set_rgb(true, true);
        break;
    case STATUS_LED_RED:
        set_rgb(true, false);
        break;
    case STATUS_LED_BLINK_YELLOW:
        s_blink_red = true; s_blink_green = true;
        s_blink_on = true;
        set_rgb(true, true);
        if (s_blink_timer) {
            esp_timer_start_periodic(s_blink_timer, 500000); /* 500 ms = 1 Hz */
        }
        break;
    case STATUS_LED_BLINK_RED:
        s_blink_red = true; s_blink_green = false;
        s_blink_on = true;
        set_rgb(true, false);
        if (s_blink_timer) {
            esp_timer_start_periodic(s_blink_timer, 500000); /* 500 ms = 1 Hz */
        }
        break;
    }
    ESP_LOGI(TAG, "state → %d", (int)state);
}
