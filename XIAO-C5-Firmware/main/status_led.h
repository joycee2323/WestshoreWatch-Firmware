#pragma once

#include "esp_err.h"

/**
 * RED/YELLOW two-color LED for the Cellular X1 status indicator.
 *
 * This part has NO green element — the two colors are RED and a traffic-light
 * orange-YELLOW (confirmed by the physical wiring: red, yellow, black common
 * cathode — no green lead). Common cathode, active HIGH. Driving both elements
 * together makes RED dominate (no clean blended color), so each state drives
 * exactly ONE element and uses blink rate to stay distinguishable — red and
 * yellow look similar, so the blink pattern is what makes a fault unmistakable
 * from healthy at a glance.
 *
 * GPIO8 (header D8) = RED, GPIO12 (header D7) = YELLOW (verified on hardware).
 */
typedef enum {
    STATUS_LED_OFF,         /* system off / pre-init                              */
    STATUS_LED_HEALTHY,     /* SOLID YELLOW — data session up, detections+HB OK   */
    STATUS_LED_WARMING,     /* SLOW-BLINK YELLOW — warming up / offline buffering  */
    STATUS_LED_DEGRADED,    /* FAST-BLINK RED — online but detection POSTs failing */
    STATUS_LED_FAULT,       /* SOLID RED — data session down / hardware fault      */
} status_led_state_t;

/** Configure GPIO12/GPIO8 as push-pull outputs, set initial OFF. */
esp_err_t status_led_init(void);

/** Set the LED state.  Blinking states use an internal timer. */
void status_led_set(status_led_state_t state);
