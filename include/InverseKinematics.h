#pragma once

#include<iostream>
#include <Eigen/Dense>
#include "Robot.h"


class InverseKinematics {

private:
    Robot& robot;
    int maxIterations = 50;
    double tolerance = 1e-6;
    double damping = 0.01;

public:
    Eigen::VectorXd targetOperationalSpace = Eigen::VectorXd::Zero(6);
    InverseKinematics(Robot& robot);
    Eigen::VectorXd solve(const Eigen::Vector3d &target_position, const Eigen::Quaterniond &target_orientation, const Eigen::VectorXd &initial_theta);
};
