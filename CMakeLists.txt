cmake_minimum_required(VERSION 3.10)
project(InverseKinematics)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
set(CMAKE_CXX_EXTENSIONS OFF)

message(STATUS "Make version: ${CMAKE_VERSION}")
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

add_subdirectory(${PROJECT_SOURCE_DIR}/src)
add_subdirectory(${PROJECT_SOURCE_DIR}/tests)
add_subdirectory(${PROJECT_SOURCE_DIR}/libs/eigen)

# Include directories for external libraries
include_directories(${PROJECT_SOURCE_DIR}/libs/eigen)