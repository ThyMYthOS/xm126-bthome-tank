# SPDX-License-Identifier: BSD-3-Clause

cmake_minimum_required(VERSION 3.20.0)

set(ACCONEER_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../acconeer)
set(BOARD_ROOT ${ACCONEER_ROOT})
set(DTS_ROOT ${ACCONEER_ROOT})

set(SHIELD acconeer_xb122)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(acconeer)

set(ACCONEER_RSS_LIB_DIR       ${ACCONEER_ROOT}/lib)
set(ACCONEER_RSS_INCLUDE_DIR   ${ACCONEER_ROOT}/include)
set(ACCONEER_INTEGRATION_DIR   ${ACCONEER_ROOT}/integration)
set(ACCONEER_COMMON_SOURCE_DIR ${ACCONEER_ROOT}/source)

target_sources(app
    PRIVATE
        ${ACCONEER_INTEGRATION_DIR}/acc_hal_integration_zephyr_121.c
        ${ACCONEER_INTEGRATION_DIR}/acc_integration_zephyr.c
        ${ACCONEER_COMMON_SOURCE_DIR}/acc_integration_log.c
        ${ACCONEER_COMMON_SOURCE_DIR}/acc_algorithm.c
        ${ACCONEER_COMMON_SOURCE_DIR}/acc_control_helper.c
        ${ACCONEER_COMMON_SOURCE_DIR}/acc_processing_helpers.c
        ${ACCONEER_COMMON_SOURCE_DIR}/acc_battery_info.c
        ${ACCONEER_COMMON_SOURCE_DIR}/acc_mcu_temperature.c
        ${ACCONEER_COMMON_SOURCE_DIR}/acc_pm.c
        ${CMAKE_CURRENT_SOURCE_DIR}/xm126_bthome_tank.c
        ${CMAKE_CURRENT_SOURCE_DIR}/acc_bthome_ble.c
        ${CMAKE_CURRENT_SOURCE_DIR}/acc_tank_config_ble.c
        ${CMAKE_CURRENT_SOURCE_DIR}/acc_serial_console_commands.c
)

target_include_directories(app PRIVATE ${ACCONEER_INTEGRATION_DIR} ${CMAKE_CURRENT_SOURCE_DIR})

add_library(libacconeer_a121 STATIC IMPORTED GLOBAL)
add_library(libacc_detector_distance_a121 STATIC IMPORTED GLOBAL)

set_target_properties(libacconeer_a121 PROPERTIES INTERFACE_INCLUDE_DIRECTORIES ${ACCONEER_RSS_INCLUDE_DIR})
set_target_properties(libacconeer_a121 PROPERTIES IMPORTED_LOCATION ${ACCONEER_RSS_LIB_DIR}/libacconeer_a121.a)
set_target_properties(libacc_detector_distance_a121 PROPERTIES INTERFACE_INCLUDE_DIRECTORIES ${ACCONEER_RSS_INCLUDE_DIR})
set_target_properties(libacc_detector_distance_a121 PROPERTIES IMPORTED_LOCATION ${ACCONEER_RSS_LIB_DIR}/libacc_detector_distance_a121.a)


target_link_libraries(app PUBLIC libacc_detector_distance_a121 libacconeer_a121)
