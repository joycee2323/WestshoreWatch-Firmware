#pragma once

#include "esp_err.h"
#include <stdbool.h>

/**
 * Cellular modem state machine.
 *
 * Drives the SIM7600G-H through boot → SIM check → network registration →
 * PPP connection.  Exposes the PPP netif so cellular_uploader can POST
 * over it.  Handles auto-recovery and hard reset on stuck states.
 */
typedef enum {
    MODEM_STATE_OFF,
    MODEM_STATE_BOOTING,
    MODEM_STATE_SIM_CHECK,
    MODEM_STATE_NETWORK_SEARCH,
    MODEM_STATE_REGISTERED,
    MODEM_STATE_PPP_CONNECTING,
    MODEM_STATE_PPP_CONNECTED,
    MODEM_STATE_ERROR,
} modem_state_t;

/** Current modem state (read from any task). */
modem_state_t modem_manager_get_state(void);

/** True when PPP is up and an IP address has been assigned. */
bool modem_manager_is_connected(void);

/**
 * Start the modem manager task.
 * Runs the full boot-to-PPP state machine, restarts on failure.
 */
esp_err_t modem_manager_start(void);

