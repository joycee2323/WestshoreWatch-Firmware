#pragma once
#include "esp_err.h"
#include <stdbool.h>

/**
 * Initialise the XIAO ESP32-C5 user LED.
 * Pin and polarity come from WSD_LED_GPIO / WSD_LED_ACTIVE_HIGH in
 * config.h. Must be called before any led_* functions.
 */
esp_err_t led_init(void);

/* Legacy pattern IDs — retained so code shared with the custom-PCB
 * builds still compiles. No-ops on the XIAO build. */
typedef enum {
    LED_PATTERN_OFF       = 0,
    LED_PATTERN_BOOT      = 1,
    LED_PATTERN_SCANNING  = 2,
    LED_PATTERN_DETECTION = 3,
    LED_PATTERN_ERROR     = 4,
    LED_PATTERN_CONFIG    = 5,
} led_pattern_t;

/* No-op on the XIAO build, kept for API compatibility. */
void led_set_pattern(led_pattern_t pattern);

/**
 * Post a detection event. Non-blocking, safe to call from any task
 * context. Drives the user LED OFF for ~50ms then back to steady ON;
 * overlapping events extend the same pulse.
 */
void led_flash_detection(void);

/* Legacy continuous-flash setter — deprecated on the XIAO build. A
 * `true` arg is treated as a one-shot event for backward compat. */
void led_set_detecting(bool detecting);
