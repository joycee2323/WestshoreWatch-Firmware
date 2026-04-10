#pragma once
#include "esp_err.h"
#include "esp_http_server.h"
#include <stddef.h>

/* HTTP POST handler for /ota — call from config_server httpd instance */
esp_err_t ota_upload_handler(httpd_req_t *req);

/* Fill running/next partition labels for display */
void ota_get_info(char *running_label, size_t label_len,
                  char *next_label,    size_t next_len);
