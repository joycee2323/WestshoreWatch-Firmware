#pragma once

#include "esp_err.h"

/**
 * Bi-color LED states for the Cellular X1 status indicator.
 * Common cathode to GND. The red/green leads are reversed vs the silk:
 * GPIO8 (header D8) physically drives RED, GPIO12 (header D7) drives GREEN
 * (verified empirically — see status_led.c). Both on = yellow.
 */
typedef enum {
    STATUS_LED_OFF,             /* system off / pre-init                 */
    STATUS_LED_GREEN,           /* healthy: online + recent upload       */
    STATUS_LED_YELLOW,          /* warming up: boot / registering        */
    STATUS_LED_RED,             /* error: data session down / hw fault   */
    STATUS_LED_BLINK_YELLOW,    /* offline: buffering detections         */
    STATUS_LED_BLINK_RED,       /* degraded: online but detections fail  */
} status_led_state_t;

/** Configure GPIO12/GPIO8 as push-pull outputs, set initial OFF. */
esp_err_t status_led_init(void);

/** Set the LED state.  Blinking states use an internal timer. */
void status_led_set(status_led_state_t state);
