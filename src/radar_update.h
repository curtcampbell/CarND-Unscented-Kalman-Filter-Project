#pragma once

#include "ukf_update.h"

///
// This class has the Radar specific Unscented Kalman Filter update logic.
//
class RadarUpdate : virtual public UnscentedKalmanUpdateBase
{
public:
  static const int measurment_dim = RadarMeasurement::measurment_dim;

  using TRadarVector = Eigen::Matrix<double, measurment_dim, 1>;
  using TRadarCovarianceMatrix = Eigen::Matrix<double, measurment_dim, measurment_dim>;
  using TRadarSigmaPointMatrix = Eigen::Matrix<double, measurment_dim, sigma_point_dimension>;

  RadarUpdate(double std_a, double std_yawd, double std_radr, double std_radphi, double std_radrd, double lambda);
  virtual ~RadarUpdate();

  virtual void InitialUpdate(TrackedObject* tracked_object, const MeasurementPackage& meas_mackage);

  /**
  * Updates the state and the state covariance matrix using a radar measurement
  * @param meas_package The measurement at k+1
  */
  virtual void Update(TrackedObject* tracked_object, const MeasurementPackage& meas_package);

private:
  ///* Radar measurement noise standard deviation radius in m
  double std_radr_;

  ///* Radar measurement noise standard deviation angle in rad
  double std_radphi_;

  ///* Radar measurement noise standard deviation radius change in m/s
  double std_radrd_;

  void PredictMeasurement(TRadarSigmaPointMatrix& Zsig, TRadarVector& z_pred, TRadarCovarianceMatrix& S);

  void UpdateState(const TRadarVector& measurement, 
                   const TRadarSigmaPointMatrix& Zsig, 
                   const TRadarVector& z_pred, 
                   const TRadarCovarianceMatrix& S, 
                   TrackedObject::TStateVector& x, 
                   TrackedObject::TCovarianceMatrix& P);

};
