cmake_minimum_required(VERSION 3.0)
set(CMAKE_FIND_PACKAGE_PREFER_CONFIG ON)

project(arucoFinderCpp)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS_RELEASE "-Ofast")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 -Wall")

find_package(eCAL REQUIRED)
find_package(Protobuf REQUIRED)
find_package(CLI11 REQUIRED)
find_package(OpenCV REQUIRED)


set(source_files
  src/arucoFinder.cpp
  src/main.cpp
)

set(PROTO_DIR  ${CMAKE_CURRENT_SOURCE_DIR}/../../../proto)

set(protobuf_files
	${PROTO_DIR}/robot_state.proto
  ${PROTO_DIR}/common.proto
  ${PROTO_DIR}/CompressedImage.proto
)


add_executable(${PROJECT_NAME} ${source_files})

PROTOBUF_TARGET_CPP(${PROJECT_NAME} ${PROTO_DIR} ${protobuf_files})

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
  eCAL::core
  protobuf::libprotobuf
  CLI11::CLI11
)
