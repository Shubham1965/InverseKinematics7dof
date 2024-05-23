#include <iostream>
#include "Robot.h"
#include "InverseKinematics.h"


int main() {
    Robot robot;
    InverseKinematics ik(robot);

    robot.initializeJointLimits();

    // Target position and orientation
    Eigen::Vector3d target_position(1.0, 0.3, 0.6); // Example target position describing the target position is at (0.5m, 0.3m, 0.6m) in the 3D space
    Eigen::Quaterniond target_orientation(-0.7071, 0, 0, 0.7071); // Example quaternion (w, x, y, z) describing the target orientation as a 90-degree rotation around the z-axis


    // Compute the target transformation matrix for comparison
    Eigen::Matrix4d target_transform = Eigen::Matrix4d::Identity();
    target_transform.block<3, 1>(0, 3) = target_position;
    target_transform.block<3, 3>(0, 0) = target_orientation.toRotationMatrix();
    std::cout << "Target transformation was:\n" << target_transform << std::endl;

    // Solve inverse kinematics
    Eigen::VectorXd initial_theta(7);
    initial_theta << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0; // Zero initial joint angles
    Eigen::VectorXd final_theta = ik.solve(target_position, target_orientation, initial_theta);
    
    std::cout << "\nFound joint angles (degrees): " << (final_theta * 180.0 / PI).transpose() << std::endl;
    std::cout << "\nPredicted transformation is:\n" << robot.forwardKinematics(final_theta) << std::endl;

    Eigen::Matrix4d diff = target_transform - robot.forwardKinematics(final_theta);
    std::cout << "Success rate: " << 100 - diff.norm()/target_transform.norm() *100 << std::endl;
    return 0;
}
