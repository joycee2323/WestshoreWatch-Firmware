#include "led.h"
#include "config.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "LED";

/* Duration of the OFF pulse fired on each detection event. */
#define LED_FLASH_OFF_MS  50

/* Level values that drive the LED on/off, accounting for the board's
 * wiring polarity (XIAO ESP32-C5 user LED is active LOW). */
#if WSD_LED_ACTIVE_HIGH
#  define LED_ON_LEVEL  1
#  define LED_OFF_LEVEL 0
#else
#  define LED_ON_LEVEL  0
#  define LED_OFF_LEVEL 1
#endif

/* Microsecond timestamp of the most recent detection event. Written
 * from detection callbacks (WiFi promisc, BLE scan) and read from the
 * LED task. A torn 64-bit read just delays the flash by one tick. */
static volatile int64_t s_last_flash_us = 0;
static TaskHandle_t     s_task          = NULL;

static inline void led_drive_on(void)  { gpio_set_level(WSD_LED_GPIO, LED_ON_LEVEL);  }
static inline void led_drive_off(void) { gpio_set_level(WSD_LED_GPIO, LED_OFF_LEVEL); }

/*
 * LED task — steady ON except during the ~50ms OFF window following a
 * detection event. Blocks on task notifications from led_flash_detection()
 * and polls at 10ms while a flash is in progress, so callbacks never
 * wait on GPIO.
 */
static void led_task(void *arg)
{
    led_drive_on();

    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        /* New detection event: drive OFF, then poll until the flash
         * window has elapsed relative to the most recent event. Further
         * events arriving during the window simply extend s_last_flash_us
         * and are absorbed into the same pulse. */
        led_drive_off();
        while (true) {
            int64_t since = esp_timer_get_time() - s_last_flash_us;
            if (since >= (int64_t)LED_FLASH_OFF_MS * 1000) break;
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        led_drive_on();

        /* Drain any notifications accumulated during the flash so the
         * next ulTaskNotifyTake blocks until a genuinely new event. */
        xTaskNotifyStateClear(NULL);
    }
}

esp_err_t led_init(void)
{
    ESP_LOGI(TAG, "led_init: GPIO%d, active %s",
             WSD_LED_GPIO, WSD_LED_ACTIVE_HIGH ? "HIGH" : "LOW");

    gpio_config_t io = {
        .pin_bit_mask = (1ULL << WSD_LED_GPIO),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&io);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "gpio_config failed: 0x%x (%s)",
                 err, esp_err_to_name(err));
        return err;
    }

    /* Start in the ON state so the LED lights up the instant the GPIO
     * is driven — no dark gap before the task scheduler picks up. */
    led_drive_on();

    BaseType_t ret = xTaskCreate(led_task, "led", 2048, NULL, 2, &s_task);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "xTaskCreate failed (ret=%d)", (int)ret);
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

/*
 * Non-blocking event post from detection callbacks. Records the event
 * timestamp and kicks the LED task; the callback returns immediately.
 */
void led_flash_detection(void)
{
    s_last_flash_us = esp_timer_get_time();
    if (s_task) xTaskNotifyGive(s_task);
}

/* Legacy pattern setter — retained for API compatibility so main.c's
 * BOOT/SCANNING/ERROR calls still compile. On the XIAO build the LED is
 * a pure detection indicator (steady on, flash off per event), so this
 * is a no-op. */
void led_set_pattern(led_pattern_t pattern)
{
    (void)pattern;
}

/* Legacy "continuous flash while detecting" setter. Deprecated on the
 * XIAO build — treated as a one-shot event pulse so any remaining
 * caller still produces a visible flicker. */
void led_set_detecting(bool detecting)
{
    if (detecting) led_flash_detection();
}
