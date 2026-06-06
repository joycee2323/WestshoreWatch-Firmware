#pragma once

#include "esp_err.h"
#include "esp_modem_c_api_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Resume PPP DATA mode on a session that is ALREADY in data mode (the modem
 * has returned CONNECT to a manual ATD dial issued via esp_modem_at_raw).
 *
 * This performs esp_modem's RESUME_DATA_MODE transition — dte->set_mode(DATA)
 * + netif.start() — WITHOUT re-dialing and WITHOUT the mode-probe that the
 * AUTODETECT/DETECT path sends (its LCP echo consumed the modem's initial
 * LCP frames, so PPP never negotiated). It is the clean counterpart to a
 * manual explicit-cid dial.
 *
 * RESUME_DATA_MODE is not exposed by the esp_modem C API, so this small
 * C++ shim reaches the C++ DCE behind the C handle via
 * esp_private/c_api_wrapper.hpp.
 *
 * @return ESP_OK if the resume succeeded, ESP_FAIL otherwise.
 */
esp_err_t modem_resume_data_mode(esp_modem_dce_t *dce);

#ifdef __cplusplus
}
#endif
