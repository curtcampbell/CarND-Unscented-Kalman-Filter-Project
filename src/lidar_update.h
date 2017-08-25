#pragma once

#include "ukf_update.h"

///
// This class has the lidar specific Unscented Kalman Filter update logic.
class LaserUpdate : virtual public UnscentedKalmanUpdateBase
{
public:
  static const int measurment_dim = LidarMeasurement::measurment_dim;

  using TLidarVector = Eigen::Matrix<double, measurment_dim, 1>;
  using TLidarCovarianceMatrix = Eigen::Matrix<double, measurment_dim, measurment_dim>;
  using TLidarSigmaPointMatrix = Eigen::Matrix<double, measurment_dim, sigma_point_dimension>;

  LaserUpdate(double std_a, double std_yawd, double std_laspx, double std_laspy, double lambda);
  virtual ~LaserUpdate();

  virtual void InitialUpdate(TrackedObject* tracked_object, const MeasurementPackage& meas_mackage);

  /**
  * Updates the state and the state covariance matrix using a laser measurement
  * @param meas_package The measurement at k+1
  */
  virtual void Update(TrackedObject* tracked_object, const MeasurementPackage& meas_package);

private:
  ///* Laser measurement noise standard deviation position1 in m
  double std_laspx_;

  ///* Laser measurement noise standard deviation position2 in m
  double std_laspy_;

  void PredictMeasurement(TLidarSigmaPointMatrix& Zsig, TLidarVector& z_pred, TLidarCovarianceMatrix& S);

  void UpdateState(const TLidarVector& measurement, const TLidarSigmaPointMatrix& Zsig, const TLidarVector& z_pred, const TLidarCovarianceMatrix& S, TrackedObject* tracked_object);

};

