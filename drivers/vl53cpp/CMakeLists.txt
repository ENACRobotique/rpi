cmake_minimum_required(VERSION 3.0)
set(CMAKE_FIND_PACKAGE_PREFER_CONFIG ON)

project(vl53_driver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()


find_package(eCAL REQUIRED)
find_package(Protobuf REQUIRED)

set(source_files
  src/main.cpp
  src/vl53.cpp
  src/uld-driver/src/vl53l5cx_api.c
  src/uld-driver/src/vl53l5cx_plugin_detection_thresholds.c
  src/uld-driver/src/vl53l5cx_plugin_motion_indicator.c
  src/uld-driver/src/vl53l5cx_plugin_xtalk.c
  src/platform/platform.c
)

set(PROTO_DIR  ${CMAKE_CURRENT_SOURCE_DIR}/../../proto)

set(protobuf_files
	${PROTO_DIR}/lidar_data.proto
)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/src/uld-driver/inc
  ${CMAKE_CURRENT_SOURCE_DIR}/src/platform
)

add_executable(${PROJECT_NAME} ${source_files})

PROTOBUF_TARGET_CPP(${PROJECT_NAME} ${PROTO_DIR} ${protobuf_files})

target_link_libraries(${PROJECT_NAME}
  eCAL::core
  protobuf::libprotobuf
  gpiod
)
