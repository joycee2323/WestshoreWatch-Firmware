cmake_minimum_required(VERSION 3.16)

# Single source of truth for firmware version.
# Baked into esp_app_desc_t (read via esp_app_get_description()->version)
# and shown in boot log and the config-portal System tab.
set(PROJECT_VER "1.2-westshore")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(westshore_remote_id)