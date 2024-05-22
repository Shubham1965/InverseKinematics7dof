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

# include_directories(${EIGEN3_INCLUDE_DIR})

# Manually set Eigen3 include directory
set(EIGEN3_INCLUDE_DIR ${eigen_SOURCE_DIR})
include_directories(${EIGEN3_INCLUDE_DIR})

# Debugging: Print Eigen version
message(STATUS "Eigen version: ${EIGEN3_VERSION}")

# Debugging: Print Eigen include directory
message(STATUS "Eigen include directory: ${EIGEN3_INCLUDE_DIR}")

include_directories(${PROJECT_SOURCE_DIR}/include)
add_subdirectory(${PROJECT_SOURCE_DIR}/src)

