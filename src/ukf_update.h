#pragma once

#include "measurement_package.h"
#include "tracked_object.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>

//The dimensions of our state vectors and the noise portion of our 
//augmented state vector are known at compile time, so why not use them.
const int noise_state_dimension = 2;

const int aug_state_dimension = state_dimension + noise_state_dimension;
const int sigma_point_dimension = 2 * aug_state_dimension + 1;

using TSigmapointsMatrix = Eigen::Matrix<double, state_dimension, sigma_point_dimension>;

using TNoiseCovarianceMatrix = Eigen::Matrix<double, noise_state_dimension, noise_state_dimension>;

using TAugStateVector = Eigen::Matrix<double, aug_state_dimension, 1>;
using TAugCovarianceMatrix = Eigen::Matrix<double, aug_state_dimension, aug_state_dimension>;
using TAugSigmapointMatrix = Eigen::Matrix<double, aug_state_dimension, sigma_point_dimension>;


///
// Interface used for all sensor update classes.
//
struct UKF_update
{
  virtual void InitialUpdate(TrackedObject* tracked_object, const MeasurementPackage& meas_mackage) = 0;

  /**
  * Prediction Predicts sigma points, the state, and the state covariance
  * matrix
  * @param delta_t Time between k and k+1 in s
  */
  virtual void Prediction(TrackedObject* tracked_object, double delta_t) = 0;

  /**
  * Updates the state and the state covariance matrix using a laser measurement
  * @param meas_package The measurement at k+1
  */
  virtual void Update(TrackedObject* tracked_object, const MeasurementPackage& meas_package) = 0;

};


///
// Base class for all unscented kalman filter prediction and updates.
// Processing common to all sensor types happens here.
class UnscentedKalmanUpdateBase: virtual public UKF_update
{
public:
  using TWeightVector = Eigen::Matrix<double, sigma_point_dimension, 1>;

  UnscentedKalmanUpdateBase(double std_a, double std_yawd, double lambda);

  //Prediction step common to all sensor types.
  virtual void Prediction(TrackedObject* tracked_object, double delta_t);

protected:
  ///* Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_2;

  ///* Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_2;

  ///* Sigma point spreading parameter
  double lambda_;

  //Predicted sigma points
  TSigmapointsMatrix Xsig_;

  TWeightVector weights_;

private:
  void GenerateAugmentedSigmaPoints(const TrackedObject::TStateVector& X, 
                                    const TrackedObject::TCovarianceMatrix& P, 
                                    TAugSigmapointMatrix & augSigmaPoints);

  void PredictSigmaPoints(const TAugSigmapointMatrix& augSigmaPoints, 
                          double delta_t, 
                          TSigmapointsMatrix& sigmaPoints);

  void CalculateMeanAndCovariance(const TSigmapointsMatrix& predictedSigmaPoints, 
                                  TrackedObject::TStateVector& x, 
                                  TrackedObject::TCovarianceMatrix& P);
};

