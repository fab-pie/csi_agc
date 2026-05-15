# Select source file based on TARGET_TYPE (rx or tx)
if(NOT DEFINED TARGET_TYPE)
    set(TARGET_TYPE "rx")
endif()

if(TARGET_TYPE STREQUAL "rx")
    set(MAIN_SRC "main_rx.c")
elseif(TARGET_TYPE STREQUAL "tx")
    set(MAIN_SRC "main_tx.c")
else()
    message(FATAL_ERROR "TARGET_TYPE must be 'rx' or 'tx'")
endif()

message(STATUS "Building CSI ${TARGET_TYPE} - using ${MAIN_SRC}")

idf_component_register(SRCS "${MAIN_SRC}"
                    INCLUDE_DIRS ".")
