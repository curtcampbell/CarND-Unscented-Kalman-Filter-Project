#include "lidar_update.h"

LaserUpdate::LaserUpdate(const TNoiseCovarianceMatrix& process_noise_covariance, double std_laspx, double std_laspy, double lambda) :
  UnscentedKalmanUpdateBase(process_noise_covariance, lambda),
  std_laspx_(std_laspx),
  std_laspy_(std_laspy)
{
}

LaserUpdate::~LaserUpdate() {}

void LaserUpdate::InitialUpdate(TrackedObject* tracked_object, const MeasurementPackage& meas_mackage)
{
  const LidarMeasurement& measurement = dynamic_cast<const LidarMeasurement&>(meas_mackage);
  tracked_object->P_ =  TrackedObject::TCovarianceMatrix::Identity();

  tracked_object->x_ << measurement.GetX(), measurement.GetY(), 0, 0, 0;
}

/**
* Updates the state and the state covariance matrix using a laser measurement.
* @param {MeasurementPackage} meas_package
*/
void LaserUpdate::Update(TrackedObject* tracked_object, const MeasurementPackage& meas_package)
{
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
  const LidarMeasurement& measurement = dynamic_cast<const LidarMeasurement&>(meas_package);

  TLidarSigmaPointMatrix Zsig;
  TLidarVector z_pred;
  TLidarVector z = measurement.GetVector();
  TLidarCovarianceMatrix S;

  PredictMeasurement(Zsig, z_pred, S);
  UpdateState(z, Zsig, z_pred, S, tracked_object);

}

void LaserUpdate::PredictMeasurement(TLidarSigmaPointMatrix& Zsig, TLidarVector& z_pred, TLidarCovarianceMatrix& S)
{
  //transform sigma points into measurement space
  for (int i = 0; i < sigma_point_dimension; i++) {

    double p_x = Xsig_(0, i);
    double p_y = Xsig_(1, i);
    double v = Xsig_(2, i);
    double yaw = Xsig_(3, i);

    // measurement model
    Zsig(0, i) = p_x;    //px
    Zsig(1, i) = p_y;    //py
  }

  //mean predicted measurement
  z_pred = TLidarVector::Zero();

  for (int i = 0; i < sigma_point_dimension; i++) {
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  //measurement covariance matrix S
  S.fill(0.0);

  for (int i = 0; i < sigma_point_dimension; i++) {
    //residual
    TLidarVector z_diff = Zsig.col(i) - z_pred;

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  //add measurement noise covariance matrix
  TLidarCovarianceMatrix R = TLidarCovarianceMatrix::Zero();
  R(0, 0) = std_laspx_ * std_laspx_;
  R(1, 1) = std_laspy_ * std_laspy_;

  S = S + R;
}

void LaserUpdate::UpdateState(const TLidarVector& measurement, const TLidarSigmaPointMatrix& Zsig, const TLidarVector& z_pred, const TLidarCovarianceMatrix& S, TrackedObject* tracked_object)
{
  using TCrossCorrelationMatrix = Eigen::Matrix<double, state_dimension, measurment_dim>;
  using TKalmanGainMatrix = Eigen::Matrix<double, state_dimension, measurment_dim>;

  TrackedObject::TStateVector& x = tracked_object->x_;
  TrackedObject::TCovarianceMatrix& P = tracked_object->P_;

  //calculate cross correlation matrix
  TCrossCorrelationMatrix Tc = TCrossCorrelationMatrix::Zero();
  for (int i = 0; i < sigma_point_dimension; ++i) {  //2n+1 simga points
                                                     //residual
    TLidarVector z_diff = Zsig.col(i) - z_pred;

    // state difference
    TrackedObject::TStateVector x_diff = Xsig_.col(i) - x;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3) -= 2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3) += 2.*M_PI;

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  //Kalman gain K;
  TKalmanGainMatrix K = Tc * S.inverse();

  //residual
  TLidarVector z_diff = measurement - z_pred;

  //update state mean and covariance matrix
  x = x + K * z_diff;
  P = P - K*S*K.transpose();
}
