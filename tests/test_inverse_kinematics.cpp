#include <gtest/gtest.h>
#include "Robot.h"
#include "InverseKinematics.h"

TEST(InverseKinematicsTest, TestInverseKinematics) {
    Robot robot;
    InverseKinematics ik(robot);

    robot.initializeJointLimits();

    // Target position (p_x, p_y, p_z) and orientation (e_x, e_y, e_z, eta) in quaternion form
    Eigen::Vector3d target_position(0.5, 0.3, 0.6); // Example target position describing the target position is at (0.5, 0.3, 0.6) in the 3D space
    Eigen::Quaterniond target_orientation(0.7071, 0, 0, 0.7071); // Example quaternion (w, x, y, z) describing the target orientation is a 90-degree rotation around the z-axis

    Eigen::VectorXd initial_theta(7);
    initial_theta << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0; // Zero initial joint angles

    Eigen::VectorXd final_theta = ik.solve(target_position, target_orientation, initial_theta);

    EXPECT_TRUE(7 == final_theta.size());
}
