idf_component_register(
    SRCS
        "main.c"
        "nvs_config.c"
        "odid_decoder.c"
        "wifi_scanner.c"
        "ble_scanner.c"
        "ble_relay.c"
        "output.c"      # compiled but not started — no constructors, no boot-time side effects
        "led.c"         # compiled but not started — onboard LED unused, status_led replaces it
        "config_server.c"
        "dns_server.c"
        "ota_handler.c"
        "status_led.c"
        "cellular_uart.c"
        "modem_manager.c"
        "modem_http.c"            # native SIM7600 AT HTTP(S) POST transport
        "detection_queue.c"
        "cellular_uploader.c"
        "gnss_reader.c"
        # modem_data_resume.cpp — PPP-only esp_modem shim; not built on the
        # native-AT-HTTP transport (kept in tree for the PPP branch).
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
        spiffs
)
