# Parse version.json and generate version_info.h from template.
# This runs at cmake configure time, keeping macros in sync with the JSON version.
file(READ "${CMAKE_CURRENT_LIST_DIR}/version.json" _VER_JSON)
string(REGEX MATCH "\"version\"[^:]*:[^\"]*\"([^\"]+)\"" _ "${_VER_JSON}")
set(_FULL_VER "${CMAKE_MATCH_1}")
string(REGEX MATCH "^([0-9]+)\\.([0-9]+)\\.([0-9]+)$" _ "${_FULL_VER}")
set(APP_VER_MAJOR "${CMAKE_MATCH_1}")
set(APP_VER_MINOR "${CMAKE_MATCH_2}")
set(APP_VER_PATCH "${CMAKE_MATCH_3}")
execute_process(
    COMMAND git -C "${CMAKE_CURRENT_LIST_DIR}" rev-parse --short HEAD
    OUTPUT_VARIABLE APP_VER_GIT_HASH
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
)
if(NOT APP_VER_GIT_HASH)
    set(APP_VER_GIT_HASH "unknown")
endif()

configure_file(
    "${CMAKE_CURRENT_LIST_DIR}/version_info.h.in"
    "${CMAKE_CURRENT_LIST_DIR}/include/zoecare/tx/version_info.h"
    @ONLY
)

idf_component_register(
    SRCS
        "src/config.c"
        "src/err.c"
        "src/tx.c"
        "src/http_server.c"
        "src/zoecare_functions.c"
        "src/zoecare_init.c"
        "src/zoecare_utils.c"
        "src/cmd_router.c"
        "src/cmd_nvs.c"
        "src/cmd_system.c"
        "src/ota_update.c"
        "src/tx_cred_switch.c"
    INCLUDE_DIRS "include"
    REQUIRES
        zoefall_common
        esp_wifi
        esp_http_client
        esp_http_server
        esp_https_ota
        esp_timer
        nvs_flash
        cjson
        console
        driver
        esp_netif
        lwip
        spi_flash
        vfs
        app_update
        mbedtls
        fatfs
        pthread
        wpa_supplicant
        log
        esp_partition
        zc_err
        zc_logs
        zc_report
        zc_wifi
)
