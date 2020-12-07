#
# Copyright (c) 2019 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
#

cmake_minimum_required(VERSION 3.13.1)

list(APPEND mcuboot_OVERLAY_CONFIG
  "${CMAKE_CURRENT_SOURCE_DIR}/child_mcuboot.conf"
  )

set(spm_CONF_FILE
  prj.conf
  ${CMAKE_CURRENT_SOURCE_DIR}/child_secure_partition_manager.conf
  )

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(demo_IoTConnect)



# NORDIC SDK APP START
target_sources(app PRIVATE src/main.c)
target_sources(app PRIVATE src/slm_at_host.c)
target_sources(app PRIVATE src/slm_util.c)
target_sources(app PRIVATE src/slm_at_gps.c)
target_sources(app PRIVATE src/orientation_tap_detector.c)
target_sources(app PRIVATE src/pir_detector.c)
target_sources(app PRIVATE src/rotation_detector.c)
target_sources(app PRIVATE src/env_sensors.c)
target_sources(app PRIVATE src/gps.c)
zephyr_library_sources_ifdef(CONFIG_SUPL_CLIENT_LIB src/supl_support.c)

target_link_libraries(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/IoTConnect/libiotconnect.a)

zephyr_include_directories(IoTConnect)
# NORDIC SDK APP END
