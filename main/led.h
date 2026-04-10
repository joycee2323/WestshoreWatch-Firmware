#pragma once
#include "esp_err.h"
#include <stdbool.h>

/**
 * Initialise the status LED on GPIO4.
 * Must be called before any led_set_* functions.
 */
esp_err_t led_init(void);

/**
 * LED blink patterns reflecting scan state:
 *   BOOT        - fast flicker while initialising
 *   SCANNING    - slow 1 Hz pulse while scanning with no detections
 *   DETECTION   - rapid 4-flash burst on each drone detected
 *   ERROR       - SOS pattern on hard fault
 */
typedef enum {
    LED_PATTERN_OFF       = 0,
    LED_PATTERN_BOOT      = 1,
    LED_PATTERN_SCANNING  = 2,
    LED_PATTERN_DETECTION = 3,
    LED_PATTERN_ERROR     = 4,
    LED_PATTERN_CONFIG    = 5,  /* slow double-blink: AP config mode active */
} led_pattern_t;

void led_set_pattern(led_pattern_t pattern);

/** Single-shot flash (non-blocking) called on each detection event. */
void led_flash_detection(void);

/** Set detecting state — LED flashes continuously while true, solid when false. */
void led_set_detecting(bool detecting);
