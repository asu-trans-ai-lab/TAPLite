cmake_minimum_required(VERSION 3.10.0)

project(TAPLite VERSION 0.1.0 LANGUAGES CXX)

option(BUILD_EXE "BUILD Executable or not" ON)
configure_file(build_config.h.in build_config.h)

if(BUILD_EXE)
    message("Building executable of ${PROJECT_NAME} on ${CMAKE_SYSTEM_NAME}")
else()
    message("Building shared library of ${PROJECT_NAME} on ${CMAKE_SYSTEM_NAME}")
endif()

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Release)

string(TOLOWER ${PROJECT_NAME} TARGET_NAME)

if(BUILD_EXE)
    add_executable(${TARGET_NAME} src/TAPLite.cpp)
else()
    add_library(${TARGET_NAME} src/TAPLite.cpp)
endif()

find_package(OpenMP REQUIRED)
target_link_libraries(${TARGET_NAME} PRIVATE OpenMP::OpenMP_CXX)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
