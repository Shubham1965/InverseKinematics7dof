#include "InverseKinematics.h"
#include <Eigen/QR>

InverseKinematics::InverseKinematics(Robot& robot) : robot(robot) {}

Eigen::VectorXd InverseKinematics::solve(const Eigen::Vector3d &target_position, const Eigen::Quaterniond &target_orientation, const Eigen::VectorXd &initial_theta) {
    Eigen::VectorXd theta = initial_theta;

    for (int iter = 0; iter < InverseKinematics::maxIterations; ++iter) {
        Eigen::Matrix4d T = robot.forwardKinematics(theta);
        Eigen::Vector3d current_position = T.block<3, 1>(0, 3);
        Eigen::Matrix3d current_orientation = T.block<3, 3>(0, 0);

        Eigen::Vector3d position_error = target_position - current_position;
        Eigen::Quaterniond q_error = robot.errorQuaternion(current_orientation, target_orientation.toRotationMatrix()); // Orientation error in quaternions
        Eigen::Vector3d orientation_error = robot.quaternion_to_rotation_vector(q_error);

        Eigen::VectorXd error(6);
        error << position_error, orientation_error;

        if (error.norm() < InverseKinematics::tolerance){
            std::cout<<"Solution found in " << iter+1 << " iterations." << std::endl;
            return theta;
        }

        Eigen::MatrixXd J = robot.jacobian(theta);
        Eigen::MatrixXd J_pseudo_inv = (J.transpose() * J + InverseKinematics::damping * Eigen::MatrixXd::Identity(theta.size(), theta.size())).inverse() * J.transpose();

        Eigen::VectorXd theta_diff = theta - (robot.JOINT_MAX_ + robot.JOINT_MIN_) / 2.0;

        Eigen::VectorXd delta_theta = J_pseudo_inv * error - 0.0001 * theta_diff;

        theta += delta_theta;

        // Joint limits enforcement
        theta = theta.cwiseMax(robot.JOINT_MIN_).cwiseMin(robot.JOINT_MAX_);
    }

    std::cout<<"Max number of iterations reached. Solution not found." << std::endl;
    return theta;
}

