cmake_minimum_required(VERSION 3.10.0)

project(TAPLite VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)

set(CMAKE_BUILD_TYPE Release)
add_executable(${PROJECT_NAME} src/TAPLite.cpp)

find_package(OpenMP REQUIRED)
target_link_libraries(${PROJECT_NAME} PRIVATE OpenMP::OpenMP_CXX)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
