#include "ukf_update.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

UnscentedKalmanUpdateBase::UnscentedKalmanUpdateBase(const TNoiseCovarianceMatrix& process_noise_covariance, double lambda) :
  noise_covariance_(process_noise_covariance),
  lambda_(lambda),
  Xsig_(TSigmapointsMatrix::Zero())
{
  //Initialize weights for sigma points.
  weights_.fill(0.5 / (aug_state_dimension + lambda_));
  weights_(0) = lambda_ / (aug_state_dimension + lambda_);
}

void UnscentedKalmanUpdateBase::Prediction(TrackedObject* tracked_object, double delta_t)
{
  TAugSigmapointMatrix augSigmaPoints;

  GenerateAugmentedSigmaPoints(tracked_object, augSigmaPoints);
  PredictSigmaPoints(augSigmaPoints, delta_t, Xsig_);
  CalculateMeanAndCovariance(Xsig_, tracked_object);
}

void UnscentedKalmanUpdateBase::GenerateAugmentedSigmaPoints(const TrackedObject* tracked_object, TAugSigmapointMatrix & augSigmaPoints)
{
  const TrackedObject::TStateVector& x = tracked_object->x_;
  const TrackedObject::TCovarianceMatrix& P = tracked_object->P_;

  //create augmented state covariance
  TAugCovarianceMatrix P_aug = TAugCovarianceMatrix::Zero();
  P_aug.topLeftCorner<state_dimension, state_dimension>() = P;
  P_aug.bottomRightCorner<noise_state_dimension, noise_state_dimension>() = noise_covariance_;

  //Create square root matrix
  TAugCovarianceMatrix A_aug = P_aug.llt().matrixL();

  double scale = sqrt(lambda_ + aug_state_dimension);

  TAugCovarianceMatrix A_scaled = scale * A_aug;

  //Augmented mean vector
  TAugStateVector x_aug = TAugStateVector::Zero();
  x_aug.head<state_dimension>() = x;

  //Fill in the sigma point matrix
  augSigmaPoints.col(0) = x_aug;
  for (int j = 0; j < aug_state_dimension; ++j)
  {
    augSigmaPoints.col(j + 1)                       = x_aug + A_scaled.col(j);
    augSigmaPoints.col(j + 1 + aug_state_dimension) = x_aug - A_scaled.col(j);
  }
}

void UnscentedKalmanUpdateBase::PredictSigmaPoints(const TAugSigmapointMatrix& augSigmaPoints, long delta_t, TSigmapointsMatrix& predictedSigmaPoints)
{
  //predict sigma points
  for (int i = 0; i < sigma_point_dimension; ++i)
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

void UnscentedKalmanUpdateBase::CalculateMeanAndCovariance(const TSigmapointsMatrix& predictedSigmaPoints, TrackedObject* tracked_object)
{
  //Set some references
  TrackedObject::TStateVector&x = tracked_object->x_;
  TrackedObject::TCovarianceMatrix& P = tracked_object->P_;

  //predicted state mean
  x.fill(0.0);
  for (int i = 0; i < sigma_point_dimension; ++i) {  //iterate over sigma points
    x = x + weights_(i) * predictedSigmaPoints.col(i);
  }

  //predicted state covariance matrix
  P.fill(0.0);
  for (int i = 0; i < sigma_point_dimension; ++i) 
  {  
    // state difference
    TrackedObject::TStateVector x_diff = predictedSigmaPoints.col(i) - x;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3) -= 2.0*M_PI;
    while (x_diff(3)< -M_PI) x_diff(3) += 2.0*M_PI;

    P = P + weights_(i) * x_diff * x_diff.transpose();
  }
}



