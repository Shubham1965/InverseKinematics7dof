#include <iostream>
#include "Robot.h"
#include "InverseKinematics.h"


int main() {
    Robot robot;
    InverseKinematics ik(robot);

    robot.initializeJointLimits();

    // Target position (p_x, p_y, p_z) and orientation (e_x, e_y, e_z, eta) in quaternion form
    Eigen::Vector3d target_position(0.5, 0.3, 0.6); // Example target position describing the target position is at (0.5, 0.3, 0.6) in the 3D space
    Eigen::Quaterniond target_orientation(0.7071, 0, 0, 0.7071); // Example quaternion (w, x, y, z) describing the target orientation is a 90-degree rotation around the z-axis

    Eigen::VectorXd initial_theta(7);
    initial_theta << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    Eigen::VectorXd final_theta = ik.solve(target_position, target_orientation, initial_theta);

    std::cout << "Final joint angles (radians): " << final_theta.transpose() << std::endl;
    std::cout << "Final joint angles (degrees): " << (final_theta * 180.0 / PI).transpose() << std::endl;
    std::cout << "Final transformation:\n" << robot.forwardKinematics(final_theta) << std::endl;

    // Compute the target transformation matrix for comparison
    Eigen::Matrix4d target_transform = Eigen::Matrix4d::Identity();
    target_transform.block<3, 1>(0, 3) = target_position;
    target_transform.block<3, 3>(0, 0) = target_orientation.toRotationMatrix();

    std::cout << "Target transformation:\n" << target_transform << std::endl;

    return 0;
}
