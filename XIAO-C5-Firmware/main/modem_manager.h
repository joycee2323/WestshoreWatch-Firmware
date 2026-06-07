#pragma once

#include "esp_err.h"
#include <stdbool.h>

/**
 * Cellular modem state machine.
 *
 * Drives the SIM7600G-H through boot → SIM check → network registration →
 * data session (AT+NETOPEN) + TLS config.  The modem stays in AT command
 * mode for the whole session — uploads use the modem's native AT HTTP(S)
 * stack (modem_http), NOT PPP.  This manager and the uploader share the one
 * UART AT channel; cellular_uart's mutex serializes them.  Handles
 * auto-recovery and hard reset on stuck states.
 */
typedef enum {
    MODEM_STATE_OFF,
    MODEM_STATE_BOOTING,
    MODEM_STATE_SIM_CHECK,
    MODEM_STATE_NETWORK_SEARCH,
    MODEM_STATE_REGISTERED,
    MODEM_STATE_NET_OPENING,    /* AT+NETOPEN / TLS config in progress */
    MODEM_STATE_ONLINE,         /* data session up — native AT HTTP ready */
    MODEM_STATE_ERROR,
} modem_state_t;

/** Current modem state (read from any task). */
modem_state_t modem_manager_get_state(void);

/** True when the data session is up (NETOPEN succeeded, still registered)
 *  and the modem can serve native AT HTTP POSTs. */
bool modem_manager_is_connected(void);

/**
 * Start the modem manager task.
 * Runs the full boot-to-PPP state machine, restarts on failure.
 */
esp_err_t modem_manager_start(void);

