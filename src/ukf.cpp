#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 30;

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;

  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
}

UnscentedKalmanUpdateBase::UnscentedKalmanUpdateBase(UKF& ukf) :
    ukf_(ukf)
{
    noise_covariance_
        << ukf_.std_a_ * ukf_.std_a_, 0.0,
        0.0, ukf_.std_yawdd_;
}


void UnscentedKalmanUpdateBase::GenerateAugmentedSigmaPoints(TAugCovarianceMatrix & augSigmaPoints)
{
    //create augmented state covariance
    TAugCovarianceMatrix P_aug = TAugCovarianceMatrix::Zero();
    P_aug.topLeftCorner<state_dimension, state_dimension>() = ukf_.P_;
    P_aug.bottomRightCorner<noise_state_dimension, noise_state_dimension>() = noise_covariance_;

    //Create square root matrix
    TAugCovarianceMatrix A_aug = P_aug.llt().matrixL();

    double scale = sqrt(ukf_.lambda_ + aug_state_dimension);

    TAugCovarianceMatrix A_scaled = scale * A_aug;

    //Augmented mean vector
    TAugStateVector x_aug = TAugStateVector::Zero();
    x_aug.head<state_dimension>() = ukf_.x_;

    //Fill in the sigma point matrix
    augSigmaPoints.col(0) = x_aug;
    for (int j = 0; j < aug_state_dimension; ++j)
    {
        augSigmaPoints.col(j + 1) = x_aug + A_scaled.col(j);
        augSigmaPoints.col(j + 1 + aug_state_dimension) = x_aug + A_scaled.col(j);
    }
}

void UnscentedKalmanUpdateBase::PredictSigmaPoints(const TAugCovarianceMatrix& augSigmaPoints, long delta_t, TSigmapointsMatrix& predictedSigmaPoints)
{
    //predict sigma points
    for (int i = 0, size = 2 * aug_state_dimension + 1; i< size; i++)
    {
        //extract values for better readability
        double p_x = augSigmaPoints(0, i);
        double p_y = augSigmaPoints(1, i);
        double v = augSigmaPoints(2, i);
        double yaw = augSigmaPoints(3, i);
        double yawd = augSigmaPoints(4, i);
        double nu_a = augSigmaPoints(5, i);
        double nu_yawdd = augSigmaPoints(6, i);

        //predicted state values
        double px_p, py_p;

        //avoid division by zero
        if (fabs(yawd) > 0.001) {
            px_p = p_x + v / yawd * (sin(yaw + yawd*delta_t) - sin(yaw));
            py_p = p_y + v / yawd * (cos(yaw) - cos(yaw + yawd*delta_t));
        }
        else {
            px_p = p_x + v*delta_t*cos(yaw);
            py_p = p_y + v*delta_t*sin(yaw);
        }

        double v_p = v;
        double yaw_p = yaw + yawd*delta_t;
        double yawd_p = yawd;

        //add noise
        px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
        py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
        v_p = v_p + nu_a*delta_t;

        yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
        yawd_p = yawd_p + nu_yawdd*delta_t;

        //write predicted sigma point into right column
        predictedSigmaPoints(0, i) = px_p;
        predictedSigmaPoints(1, i) = py_p;
        predictedSigmaPoints(2, i) = v_p;
        predictedSigmaPoints(3, i) = yaw_p;
        predictedSigmaPoints(4, i) = yawd_p;
    }

}

void UnscentedKalmanUpdateBase::Prediction(double delta_t)
{
}