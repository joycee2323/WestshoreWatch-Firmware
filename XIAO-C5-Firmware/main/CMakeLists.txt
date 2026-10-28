idf_component_register(
    SRCS
        "main.c"
        "nvs_config.c"
        "odid_decoder.c"
        "wifi_scanner.c"
        "ble_scanner.c"
        "ble_relay.c"
        "output.c"
        "led.c"
        "config_server.c"
        "dns_server.c"
        "ota_handler.c"
        "field_test_5ghz_log.c"  # TEMPORARY field-test instrumentation, see header
    INCLUDE_DIRS "."
    REQUIRES
        nvs_flash
        esp_wifi
        esp_netif
        esp_event
        esp_http_server
        esp_http_client
        esp_https_ota
        app_update
        bt
        driver
        mbedtls
        lwip
        spiffs  # TEMPORARY field-test instrumentation, see field_test_5ghz_log.h
)
