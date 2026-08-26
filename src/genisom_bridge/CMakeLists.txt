cmake_minimum_required(VERSION 3.8)
project(genisom_bridge)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Threads REQUIRED)

set(GENISOM_SDK_ROOT
  "${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/genisom_L1_sdk")

if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64|ARM64")
  set(GENISOM_SDK_ARCH "aarch64")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|amd64|AMD64")
  set(GENISOM_SDK_ARCH "x86_64")
else()
  message(FATAL_ERROR "Unsupported GENISOM SDK architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

set(GENISOM_SDK_LIBRARY
  "${GENISOM_SDK_ROOT}/lib/${GENISOM_SDK_ARCH}/libzsibot.a")

if(NOT EXISTS "${GENISOM_SDK_ROOT}/include/zsibot_sdk/zsibot_api.h")
  message(FATAL_ERROR "GENISOM SDK headers not found: ${GENISOM_SDK_ROOT}")
endif()
if(NOT EXISTS "${GENISOM_SDK_LIBRARY}")
  message(FATAL_ERROR "GENISOM SDK library not found: ${GENISOM_SDK_LIBRARY}")
endif()

add_library(genisom_sdk STATIC IMPORTED GLOBAL)
set_target_properties(genisom_sdk PROPERTIES
  IMPORTED_LOCATION "${GENISOM_SDK_LIBRARY}"
  INTERFACE_INCLUDE_DIRECTORIES "${GENISOM_SDK_ROOT}/include")

add_executable(genisom_state_node src/genisom_state_node.cpp)
target_compile_features(genisom_state_node PUBLIC cxx_std_20)
ament_target_dependencies(genisom_state_node PUBLIC rclcpp geometry_msgs std_msgs std_srvs)
target_link_libraries(genisom_state_node PRIVATE genisom_sdk Threads::Threads)

install(TARGETS genisom_state_node
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
