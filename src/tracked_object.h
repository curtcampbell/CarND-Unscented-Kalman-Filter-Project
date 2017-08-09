#pragma once

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>

struct TrackedObject
{
    ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
    Eigen::VectorXd x_;

    ///* state covariance matrix
    Eigen::MatrixXd P_;

};
