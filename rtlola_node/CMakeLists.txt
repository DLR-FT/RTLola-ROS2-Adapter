# SPDX-FileCopyrightText: 2023 German Aerospace Center (DLR)
# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.5)
project(rtlola_node)

find_package(ament_cmake REQUIRED)

set(MONITOR_EXECUTABLE ${CMAKE_CURRENT_SOURCE_DIR}/../monitor/target/release/rtlola_ros2_monitor)
if(EXISTS ${MONITOR_EXECUTABLE})
  install(
    PROGRAMS
      ${MONITOR_EXECUTABLE}
    DESTINATION lib/${PROJECT_NAME}
  )
else()
  set(MONITOR_EXECUTABLE ${CMAKE_CURRENT_SOURCE_DIR}/../monitor/target/debug/rtlola_ros2_monitor)
  if(EXISTS ${MONITOR_EXECUTABLE})
    install(
      PROGRAMS
        ${MONITOR_EXECUTABLE}
      DESTINATION lib/${PROJECT_NAME}
    )
  else()
    message(FATAL_ERROR "Executable not found: ${MONITOR_EXECUTABLE}. Did you run 'cargo build'?")
  endif()
endif()

ament_package()
