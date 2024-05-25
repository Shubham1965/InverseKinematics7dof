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
            std::cout<<"\nSolution found in " << iter+1 << " iterations." << std::endl;
            return theta;
        }

        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(theta.size(), theta.size());
        Eigen::MatrixXd J = robot.jacobian(theta);
        Eigen::MatrixXd J_pseudo_inv = (J.transpose() * J + InverseKinematics::damping * Eigen::MatrixXd::Identity(theta.size(), theta.size())).inverse() * J.transpose();
        Eigen::MatrixXd N = I - J_pseudo_inv * J;


        Eigen::VectorXd theta_diff = theta - (robot.JOINT_MAX_ + robot.JOINT_MIN_) / 2.0;
        Eigen::VectorXd secondary_objective = -0.01 * theta_diff;

        Eigen::VectorXd delta_theta = J_pseudo_inv * error + N * secondary_objective;

        theta += delta_theta;

        // Joint limits enforcement
        theta = theta.cwiseMax(robot.JOINT_MIN_).cwiseMin(robot.JOINT_MAX_);


        Eigen::Matrix4d target_transform = Eigen::Matrix4d::Identity();
        target_transform.block<3, 1>(0, 3) = target_position;
        target_transform.block<3, 3>(0, 0) = target_orientation.toRotationMatrix();

        Eigen::Matrix4d diff = target_transform - T;
        std::cout << "Iteration :" << iter+1 << << ", error: " << error.norm() << ", condition number: " << robot.conditionNumber(J) << ", success rate: " << 100 - diff.norm()/target_transform.norm() *100<< std::endl;
    }

    std::cout<<"\nMax number of iterations reached. Solution found is proximal." << std::endl;
    return theta;
}

