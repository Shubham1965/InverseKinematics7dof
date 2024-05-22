#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>

const double PI = 3.14159265358979323846;
const double DEG2RAD = PI / 180.0;
const double RAD2DEG = 180.0 / PI;

class Robot {

private:
    double DEFAULT_THETA_[7] = {0, 0, 0, 0, 0, 0, 0}; //rad
    double DEFAULT_D_[7] = {0.438, 0.0, 0.7, 0.0, 0.7, 0.0, 0.115}; //m
    double DEFAULT_ALPHA_[7] = {-PI/2, PI/2, -PI/2, PI/2, -PI/2, PI/2, 0}; //rad
    double DEFAULT_A_[7] = {0, 0, 0, 0, 0, 0, 0};

public:
    Eigen::VectorXd JOINT_MIN_ = Eigen::VectorXd::Zero(7), JOINT_MAX_ = Eigen::VectorXd::Zero(7);
    void initializeJointLimits();
    Eigen::Matrix4d DH(double a, double alpha, double d, double theta);
    Eigen::Matrix4d forwardKinematics(const Eigen::VectorXd& theta);
    Eigen::MatrixXd jacobian(const Eigen::VectorXd& theta);
    Eigen::Quaterniond Robot::errorQuaternion(const Eigen::Matrix3d &R_current, const Eigen::Matrix3d &R_target);
    Eigen::Vector3d Robot::quaternion_to_rotation_vector(const Eigen::Quaterniond &q);
    double conditionNumber(const Eigen::MatrixXd& J);
};