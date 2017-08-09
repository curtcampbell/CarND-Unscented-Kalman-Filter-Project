#ifndef UKF_H
#define UKF_H

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

//The dimensions of our state vectors and the noise portion of our 
//augmented state vector are known at compile time, so why not use them.
const int state_dimension = 5;
const int noise_state_dimension = 2;

const int aug_state_dimension = state_dimension + noise_state_dimension;

typedef Eigen::Matrix<double, state_dimension, 1> TStateVector;
typedef Eigen::Matrix<double, state_dimension, state_dimension> TCovarianceMatrix;
typedef Eigen::Matrix<double, state_dimension, 2 * aug_state_dimension + 1> TSigmapointsMatrix;

typedef Eigen::Matrix<double, noise_state_dimension, noise_state_dimension> TNoiseCovarianceMatrix;

typedef Eigen::Matrix<double, aug_state_dimension, 1> TAugStateVector;
typedef Eigen::Matrix<double, aug_state_dimension, aug_state_dimension> TAugCovarianceMatrix;
typedef Eigen::Matrix<double, aug_state_dimension, 2 * aug_state_dimension + 1> TAugSigmapointMatrix;


class UKF {
public:

  ///* initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  ///* if this is false, laser measurements will be ignored (except for init)
  bool use_laser_;

  ///* if this is false, radar measurements will be ignored (except for init)
  bool use_radar_;

  ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  TStateVector x_;

  ///* state covariance matrix
  TCovarianceMatrix P_;

  ///* predicted sigma points matrix
  TSigmapointsMatrix Xsig_pred_;

  ///* time when the state is true, in us
  long long time_us_;

  ///* Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_;

  ///* Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_;

  ///* Laser measurement noise standard deviation position1 in m
  double std_laspx_;

  ///* Laser measurement noise standard deviation position2 in m
  double std_laspy_;

  ///* Radar measurement noise standard deviation radius in m
  double std_radr_;

  ///* Radar measurement noise standard deviation angle in rad
  double std_radphi_;

  ///* Radar measurement noise standard deviation radius change in m/s
  double std_radrd_ ;

  ///* Weights of sigma points
  VectorXd weights_;

  ///* Sigma point spreading parameter
  double lambda_;


  /**
   * Constructor
   */
  UKF();

  /**
   * Destructor
   */
  virtual ~UKF();

  /**
   * ProcessMeasurement
   * @param meas_package The latest measurement data of either radar or laser
   */
  void ProcessMeasurement(MeasurementPackage meas_package);

  /**
   * Prediction Predicts sigma points, the state, and the state covariance
   * matrix
   * @param delta_t Time between k and k+1 in s
   */
  void Prediction(double delta_t);

  /**
   * Updates the state and the state covariance matrix using a laser measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateLidar(MeasurementPackage meas_package);

  /**
   * Updates the state and the state covariance matrix using a radar measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateRadar(MeasurementPackage meas_package);
};


///
// Base class for all unscented kalman filter prediction and updates
// Processing common to all sensor types happens here.
class UnscentedKalmanUpdateBase
{
public:

    UnscentedKalmanUpdateBase(UKF& ukf);

    /**
    * Prediction Predicts sigma points, the state, and the state covariance
    * matrix
    * @param delta_t Time between k and k+1 in s
    */
    void Prediction(double delta_t);

    /**
    * Updates the state and the state covariance matrix using a laser measurement
    * @param meas_package The measurement at k+1
    */
    virtual void Update(MeasurementPackage meas_package) = 0;

protected:
    ///* Process noise standard deviation longitudinal acceleration in m/s^2
    double std_a_;

    ///* Process noise standard deviation yaw acceleration in rad/s^2
    double std_yawdd_;

    UKF& ukf_;

    //Q - noise covariance
    TNoiseCovarianceMatrix noise_covariance_;

private:
    void GenerateAugmentedSigmaPoints(TAugCovarianceMatrix& augSigmaPoints);
    void PredictSigmaPoints(const TAugCovarianceMatrix& augSigmaPoints, long delta_t, TSigmapointsMatrix& sigmaPoints);

};

///
// This class has the lidar specific Unscented Kalman Filter update logic.
class LaserUpdate : virtual public UnscentedKalmanUpdateBase
{
    /**
    * Updates the state and the state covariance matrix using a laser measurement
    * @param meas_package The measurement at k+1
    */
    virtual void Update(MeasurementPackage meas_package) = 0;

private:
    ///* Laser measurement noise standard deviation position1 in m
    double std_laspx_;

    ///* Laser measurement noise standard deviation position2 in m
    double std_laspy_;
};

///
// This class has the Radar specific Unscented Kalman Filter update logic.
//
class RadarUpdate : virtual public UnscentedKalmanUpdateBase
{
public:
    /**
    * Updates the state and the state covariance matrix using a laser measurement
    * @param meas_package The measurement at k+1
    */
    virtual void Update(MeasurementPackage meas_package) = 0;

private:
    ///* Radar measurement noise standard deviation radius in m
    double std_radr_;

    ///* Radar measurement noise standard deviation angle in rad
    double std_radphi_;

    ///* Radar measurement noise standard deviation radius change in m/s
    double std_radrd_;

};

#endif /* UKF_H */
