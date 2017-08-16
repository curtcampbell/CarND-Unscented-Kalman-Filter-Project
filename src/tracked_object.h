#pragma once

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>

const int state_dimension = 5;

struct TrackedObject
{
  using TStateVector = Eigen::Matrix<double, state_dimension, 1>;
  using TCovarianceMatrix = Eigen::Matrix<double, state_dimension, state_dimension>;

  ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  TStateVector x_;

  ///* state covariance matrix
  TCovarianceMatrix P_;

  ///* time when the state is true, in us
  long long timestamp_;

};
