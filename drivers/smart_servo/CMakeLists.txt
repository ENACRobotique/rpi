cmake_minimum_required(VERSION 3.0)
set(CMAKE_FIND_PACKAGE_PREFER_CONFIG ON)

project(smartServoDriver)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()

set(CMAKE_CXX_FLAGS_RELEASE "-Ofast")


find_package(eCAL REQUIRED)
find_package(Protobuf REQUIRED)

set(source_files
  src/main.cpp
  src/serial.cpp
  src/smart_servo.cpp
  src/Dynamixel.cpp
  src/STS3032.cpp
)

set(PROTO_DIR  ${CMAKE_CURRENT_SOURCE_DIR}/../../proto)

set(protobuf_files
  ${PROTO_DIR}/actionneurs.proto
)


add_executable(${PROJECT_NAME} ${source_files})

PROTOBUF_TARGET_CPP(${PROJECT_NAME} ${PROTO_DIR} ${protobuf_files})

target_link_libraries(${PROJECT_NAME}
  eCAL::core
  protobuf::libprotobuf
)
