add_executable(test_ik test_inverse_kinematics.cpp)
target_link_libraries(test_ik Robot InverseKinematics Eigen3::Eigen gtest gtest_main)

include(GoogleTest)
add_test(test_ik tests/test_ik)