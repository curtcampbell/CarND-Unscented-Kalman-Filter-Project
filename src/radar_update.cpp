#include "radar_update.h"


using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;


RadarUpdate::RadarUpdate(const TNoiseCovarianceMatrix& process_noise_covariance, double std_radr, double std_radphi, double std_radrd, double lambda) :
  UnscentedKalmanUpdateBase(process_noise_covariance, lambda),
  std_radr_(std_radr),
  std_radphi_(std_radphi),
  std_radrd_(std_radrd)
{

}

RadarUpdate::~RadarUpdate() {}

void RadarUpdate::InitialUpdate(TrackedObject* tracked_object, const MeasurementPackage& meas_mackage)
{
  tracked_object->P_ = TrackedObject::TCovarianceMatrix::Identity();

  //Todo change this to real initilization.
  tracked_object->x_ = TrackedObject::TStateVector::Zero();
}

/**
* Updates the state and the state covariance matrix using a radar measurement.
* @param {MeasurementPackage} meas_package
*/
void RadarUpdate::Update(TrackedObject* tracked_object, const MeasurementPackage& meas_package)
{
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */

  const RadarMeasurement& measurement = dynamic_cast<const RadarMeasurement&>(meas_package);

  TRadarSigmaPointMatrix Zsig;
  TRadarVector z_pred;
  TRadarVector z = measurement.GetVector();
  TRadarCovarianceMatrix S;

  PredictMeasurement(Zsig, z_pred, S);
  UpdateState(z, Zsig, z_pred, S, tracked_object);
}

void RadarUpdate::PredictMeasurement(TRadarSigmaPointMatrix& Zsig, TRadarVector& z_pred, TRadarCovarianceMatrix& S)
{
  //transform sigma points into measurement space
  for (int i = 0; i < sigma_point_dimension; i++) {  //2n+1 simga points

                                                     // extract values for better readibility
    double p_x = Xsig_(0, i);
    double p_y = Xsig_(1, i);
    double v = Xsig_(2, i);
    double yaw = Xsig_(3, i);

    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;

    // measurement model
    Zsig(0, i) = sqrt(p_x*p_x + p_y*p_y);                        //r
    Zsig(1, i) = atan2(p_y, p_x);                                 //phi
    Zsig(2, i) = (p_x*v1 + p_y*v2) / sqrt(p_x*p_x + p_y*p_y);   //r_dot

  }

  //mean predicted measurement
  z_pred = TRadarVector::Zero();

  TWeightVector weights;
  weights.fill(0.5 / (aug_state_dimension + lambda_));

  for (int i = 0; i < sigma_point_dimension; i++) {
    z_pred = z_pred + weights(i) * Zsig.col(i);
  }

  //measurement covariance matrix S
  S.fill(0.0);

  for (int i = 0; i < sigma_point_dimension; i++) {
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    //angle normalization
    //while (z_diff(1)> M_PI) z_diff(1) -= 2.*M_PI;
    //while (z_diff(1)<-M_PI) z_diff(1) += 2.*M_PI;

    S = S + weights(i) * z_diff * z_diff.transpose();
  }

  //add measurement noise covariance matrix
  TRadarCovarianceMatrix R = TRadarCovarianceMatrix::Zero();
  R(0, 0) = std_radr_*std_radr_;
  R(1, 1) = std_radphi_ * std_radphi_;
  R(2, 2) = std_radrd_ * std_radrd_;

  S = S + R;
}

void RadarUpdate::UpdateState(const TRadarVector& measurement, const TRadarSigmaPointMatrix& Zsig, const TRadarVector& z_pred, const TRadarCovarianceMatrix& S, TrackedObject* tracked_object)
{
  using TCrossCorrelationMatrix = Eigen::Matrix<double, state_dimension, 3>;

  TrackedObject::TStateVector& x = tracked_object->x_;
  TrackedObject::TCovarianceMatrix& P = tracked_object->P_;

  //set vector for weights
  double weight = 0.5 / (aug_state_dimension + lambda_);

  TWeightVector weights;
  weights(0) = lambda_ / (lambda_ + aug_state_dimension);
  for (int i = 1; i<sigma_point_dimension; i++) {
    weights(i) = weight;
  }

  //calculate cross correlation matrix
  TCrossCorrelationMatrix Tc = TCrossCorrelationMatrix::Zero();
  for (int i = 0; i < sigma_point_dimension; i++) {  //2n+1 simga points

                                                     //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    //angle normalization
    //while (z_diff(1)> M_PI) z_diff(1) -= 2.*M_PI;
    //while (z_diff(1)<-M_PI) z_diff(1) += 2.*M_PI;

    // state difference
    VectorXd x_diff = Xsig_.col(i) - x;
    //angle normalization
    //while (x_diff(3)> M_PI) x_diff(3) -= 2.*M_PI;
    //while (x_diff(3)<-M_PI) x_diff(3) += 2.*M_PI;

    Tc = Tc + weights(i) * x_diff * z_diff.transpose();
  }

  //Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  //residual
  VectorXd z_diff = measurement - z_pred;

  //angle normalization
  //while (z_diff(1)> M_PI) z_diff(1) -= 2.*M_PI;
  //while (z_diff(1)<-M_PI) z_diff(1) += 2.*M_PI;

  //update state mean and covariance matrix
  x = x + K * z_diff;
  P = P - K*S*K.transpose();
}
