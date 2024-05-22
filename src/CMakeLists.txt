add_library(Robot Robot.cpp)
add_library(InverseKinematics InverseKinematics.cpp)

add_executable(bin_test test.cpp)
target_link_libraries(bin_test Robot InverseKinematics Eigen3::Eigen)



