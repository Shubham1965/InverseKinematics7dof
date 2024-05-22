#include "Robot.h"

void Robot::initializeJointLimits() {
    Robot::JOINT_MIN_ << -180.0 * DEG2RAD, -120.0 * DEG2RAD, -180.0 * DEG2RAD, -150.0 * DEG2RAD, -180.0 * DEG2RAD, -180.0 * DEG2RAD, -180.0 * DEG2RAD;
    Robot::JOINT_MAX_ << 180.0 * DEG2RAD, 120.0 * DEG2RAD, 180.0 * DEG2RAD, 150.0 * DEG2RAD, 180.0 * DEG2RAD, 180.0 * DEG2RAD, 180.0 * DEG2RAD;
}

Eigen::Matrix4d Robot::DH(double a, double alpha, double d, double theta) {
    Eigen::Matrix4d T;
    T << cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta),
         sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta),
         0,          sin(alpha),             cos(alpha),            d,
         0,          0,                      0,                     1;
    return T;
}


Eigen::Matrix4d Robot::forwardKinematics(const Eigen::VectorXd& theta) {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    for (int i = 0; i < theta.size(); ++i) {
        T *= DH(Robot::DEFAULT_A_[i], Robot::DEFAULT_ALPHA_[i], Robot::DEFAULT_D_[i], theta[i]);
    }

    return T;
}

Eigen::MatrixXd Robot::jacobian(const Eigen::VectorXd& theta) {
    size_t n = theta.size();
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    Eigen::MatrixXd J(6, n);
    std::vector<Eigen::Matrix4d> transforms(n + 1, Eigen::Matrix4d::Identity());

    // Compute the transformation matrix for each joint
    for (size_t i = 0; i < n; ++i) {
        T *= DH(Robot::DEFAULT_A_[i], Robot::DEFAULT_ALPHA_[i], Robot::DEFAULT_D_[i], theta[i]);
        transforms[i + 1] = T;
    }

    Eigen::Vector3d pe = transforms.back().block<3,1>(0,3); // Position of end-effector

    for (size_t i = 0; i < n; ++i) {
        Eigen::Vector3d pi = transforms[i].block<3,1>(0,3); // Position of joint
        Eigen::Vector3d zi = transforms[i].block<3,1>(0,2); // Z-axis of joint

        J.block<3,1>(0,i) = zi.cross(pe - pi); // position part of geometric jacobian
        J.block<3,1>(3,i) = zi; // orientation part of geometric jacobian   
    }

    return J;
}

Eigen::Quaterniond Robot::errorQuaternion(const Eigen::Matrix3d &R_current, const Eigen::Matrix3d &R_target) {
    Eigen::Quaterniond q_current(R_current);
    Eigen::Quaterniond q_target(R_target);
    return q_target * q_current.inverse();
}


Eigen::Vector3d Robot::quaternion_to_rotation_vector(const Eigen::Quaterniond &q) {
    Eigen::AngleAxisd angle_axis(q);
    return angle_axis.angle() * angle_axis.axis();
}

double Robot::conditionNumber(const Eigen::MatrixXd& J) {
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(J);
    double cond = svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size() - 1);
    return cond;
}


