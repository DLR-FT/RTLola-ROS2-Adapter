# SPDX-FileCopyrightText: 2023 German Aerospace Center (DLR)
# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.5)
project(rtlola_testnode)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)

set(msg_files 
  "msg/Adc.msg"
  "msg/GpsPos.msg"
  "msg/RTLolaOutput.msg"
  "srv/RTLolaService.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME} ${msg_files})
ament_export_dependencies(rosidl_default_runtime)

add_executable(testnode src/testnode.cpp)

rosidl_target_interfaces(testnode ${PROJECT_NAME} "rosidl_typesupport_cpp")
ament_target_dependencies(testnode "rclcpp" sensor_msgs)



install(TARGETS testnode
  DESTINATION lib/${PROJECT_NAME})

ament_package()
