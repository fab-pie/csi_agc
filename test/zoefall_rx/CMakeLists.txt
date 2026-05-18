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
    "${CMAKE_CURRENT_LIST_DIR}/include/zoecare/rx/version_info.h"
    @ONLY
)

idf_component_register(
    SRCS
        "src/config.c"
        "src/err.c"
        "src/rx.c"
        "src/dapd.c"
        "src/svm_rbf.c"
        "src/svm_arrays.c"
        "src/fall_validation.c"
        "src/ota_update.c"
        "src/utils.c"
    INCLUDE_DIRS "include"
    REQUIRES
        zoefall_common
        esp_csi_gain_ctrl
        esp_wifi
        esp_http_client
        esp_https_ota
        esp_timer
        nvs_flash
        cjson
        driver
        esp_netif
        esp_system
        lwip
        log
        app_update
        mbedtls
        esp_psram
        esp_partition
        zc_err
        zc_logs
        zc_report
        zc_memory
        zc_wifi
)

target_compile_definitions(${COMPONENT_LIB} PRIVATE
    DAPD_TW_MAX=400
    DAPD_M_ACTIVE_MAX=64
    DAPD_NUM_POINTS=200
    DAPD_FAST_EXP=0
    DAPD_INCREMENTAL_KDE=0
    BINNED_KDE_CONV=1
    DAPD_DIAG_METRICS=0
    CSI_RAW_STREAM=0
    DAPD_PROFILE=0
    DAPD_COMPARE=0
)
