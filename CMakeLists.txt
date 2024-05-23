cmake_minimum_required(VERSION 3.10)
project(InverseKinematics7dof)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

message(STATUS "CMake version: ${CMAKE_VERSION}")
message(STATUS "Source directory: ${PROJECT_SOURCE_DIR}")

# Include Eigen
include(FetchContent)
FetchContent_Declare(
  eigen
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG 3.4.0  # Use the appropriate version tag or branch
)
FetchContent_MakeAvailable(eigen)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)


# Include directories
include_directories(${eigen_SOURCE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/include)

# Add subdirectories
add_subdirectory(${PROJECT_SOURCE_DIR}/src)

FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)


# Unit tests
enable_testing()

# Outsource tests to another folder 
add_subdirectory(${PROJECT_SOURCE_DIR}/tests)




