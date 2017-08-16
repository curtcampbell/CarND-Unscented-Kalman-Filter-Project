#pragma once

#include "ukf_update.h"

///
// This class has the lidar specific Unscented Kalman Filter update logic.
class LaserUpdate : virtual public UnscentedKalmanUpdateBase
{
public:

  LaserUpdate(const TNoiseCovarianceMatrix& process_noise_covariance, double std_laspx, double std_laspy, double lambda);
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
};

