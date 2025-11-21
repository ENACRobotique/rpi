cmake_minimum_required(VERSION 3.16)
project(hello_qt6_eglfs LANGUAGES CXX)

set(CMAKE_AUTOMOC ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Qt6 REQUIRED COMPONENTS Widgets)

add_executable(hello_qt6 main.cpp)
target_link_libraries(hello_qt6 PRIVATE Qt6::Widgets)

