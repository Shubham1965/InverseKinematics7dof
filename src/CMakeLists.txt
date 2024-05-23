add_library(Robot Robot.cpp)
add_library(InverseKinematics InverseKinematics.cpp)

add_executable(bin_main main.cpp)
target_link_libraries(bin_main Robot InverseKinematics Eigen3::Eigen)


