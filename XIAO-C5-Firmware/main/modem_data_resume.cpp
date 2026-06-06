/*
 * Minimal C++ shim exposing esp_modem's RESUME_DATA_MODE transition to the
 * C firmware. See modem_data_resume.h for rationale.
 *
 * We deliberately do NOT reimplement the modem layer — this only calls the
 * existing esp_modem C++ DCE method that the C API doesn't surface.
 */
/* Include order mirrors esp_modem's own esp_modem_c_api.cpp: the private
 * c_api_wrapper.hpp references DCE / command_result / std::shared_ptr without
 * including their headers, so the C++ esp_modem headers (and <memory>) must
 * come first. */
#include <memory>
#include <utility>
#include "cxx_include/esp_modem_api.hpp"
#include "cxx_include/esp_modem_dce_factory.hpp"
#include "esp_modem_c_api_types.h"
#include "esp_private/c_api_wrapper.hpp"   /* struct esp_modem_dce_wrap { DCE *dce; ... } */
#include "modem_data_resume.h"

extern "C" esp_err_t modem_resume_data_mode(esp_modem_dce_t *dce_wrap)
{
    if (dce_wrap == nullptr || dce_wrap->dce == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    return dce_wrap->dce->set_mode(esp_modem::modem_mode::RESUME_DATA_MODE)
           ? ESP_OK : ESP_FAIL;
}
