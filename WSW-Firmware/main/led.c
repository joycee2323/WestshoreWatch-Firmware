#include "led.h"
#include "config.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "LED";

static volatile led_pattern_t s_pattern   = LED_PATTERN_OFF;
static volatile bool s_detecting = false;
static TaskHandle_t  s_task      = NULL;

static inline void led_on(void)  { gpio_set_level(WSD_LED_GPIO, 1); }
static inline void led_off(void) { gpio_set_level(WSD_LED_GPIO, 0); }

static void led_task(void *arg)
{
    ESP_LOGI(TAG, "led_task: entered, driving GPIO%d high", WSD_LED_GPIO);
    led_on();  /* Solid on at boot */

    while (true) {
        if (s_pattern == LED_PATTERN_CONFIG) {
            /* Config mode: double-blink every 2 seconds
             * blink-blink ... pause ... blink-blink ... */
            led_on();  vTaskDelay(pdMS_TO_TICKS(120));
            led_off(); vTaskDelay(pdMS_TO_TICKS(120));
            led_on();  vTaskDelay(pdMS_TO_TICKS(120));
            led_off(); vTaskDelay(pdMS_TO_TICKS(1500));
        } else if (s_detecting) {
            /* Continuous fast flash while drone is being detected */
            led_off(); vTaskDelay(pdMS_TO_TICKS(100));
            led_on();  vTaskDelay(pdMS_TO_TICKS(100));
        } else {
            /* Solid on when idle */
            led_on();
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
}

esp_err_t led_init(void)
{
    ESP_LOGI(TAG, "led_init: enter, GPIO%d", WSD_LED_GPIO);

    gpio_config_t io = {
        .pin_bit_mask = (1ULL << WSD_LED_GPIO),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_LOGI(TAG, "led_init: calling gpio_config");
    esp_err_t err = gpio_config(&io);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "gpio_config failed: 0x%x (%s)",
                 err, esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "led_init: gpio_config OK");

    led_off();
    ESP_LOGI(TAG, "led_init: initial led_off done");

    BaseType_t ret = xTaskCreate(led_task, "led", 2048, NULL, 2, &s_task);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "led_init: xTaskCreate failed (ret=%d)", (int)ret);
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "LED initialised on GPIO%d", WSD_LED_GPIO);
    return ESP_OK;
}

void led_set_pattern(led_pattern_t pattern)
{
    s_pattern = pattern;
    if (pattern == LED_PATTERN_CONFIG) {
        s_detecting = false;
    }
}

void led_flash_detection(void)
{
    /* Not used — kept for API compatibility */
}

void led_set_detecting(bool detecting)
{
    s_detecting = detecting;
}
